"""
L1層: 安全ラッパ層（MaxArmRobot）

Hiwonder MaxArm（ESP32 + リポジトリ管理の MicroPython コマンドサーバ
"saigen-maxarm"）を LabRobot と同じ安全方針でラップする。

- すべての目標座標を、コマンド送信前に可動域バリデータで検証する
  （可動域違反ならアームは動かない）。接続時には同じ可動域をファームウェア側にも
  設定する（LIMITS。PC 側の検証が第一、ファームウェア側は二重の備え）。
  可動域バリデータの無い MaxArmRobot は initialize() を拒否する。
- ブロッキングな移動（MOVE → 到達確認）はワーカースレッドで実行し、
  Ctrl+C / キャンセル時は STOP（force_stop）を1回送ってその場で止める
  （InterruptibleDriverMixin）。
- 移動が可動域検証以外の理由で失敗した場合（MOVE の応答が失われた、到達誤差が
  許容差を超えた、タイムアウト、通信エラー）も、アームが動いている可能性があるため
  STOP を1回送ってから姿勢不明として例外を送出する。
- 中断・失敗・緊急停止の後は姿勢不明として扱い、次の移動の前に位置を読み直して
  静止を確認する。公開操作の途中で緊急停止されたら（読み直し・ラッチ解除・移動の
  どの段階でも）、それ以上コマンドを送らずに RuntimeError を送出する。
- エラー後の自動ホーム復帰は行わない（MaxArm には検証済みの安全な復帰経路が無い。
  どうするかはセッション側が決める）。
- cleanup() はアームを動かさない。姿勢不明・緊急停止の後はまず STOP を送る。
  ポンプは正常終了のときだけ止める（吸着ポリシーは cleanup() を参照）。

相対移動（move_z）と go_home の最初の上昇の基準は「最後に完了した移動の指令値」
（位置を読み直した後は読み直した値）であり、read-back 値ではない。read-back の
ノイズを積算しないための方針で、指令値と実位置の差は各軸 move_tolerance_mm 以内に
保証される（超えたら移動は失敗扱い）。精度が要る場合は move_tolerance_mm を狭める。
"""

import asyncio
import logging
import math
import threading
from typing import Any, Callable, Dict, List, Optional

from src.devices.safety.interruptible import InterruptibleDriverMixin
from src.devices.safety.validators import ValidationError, WorkspaceValidator

logger = logging.getLogger(__name__)

#: MaxArm で使える flow のアクション（executor / run_flow の能力チェック用）
MAXARM_SUPPORTED_ACTIONS = frozenset({"move_xyz", "move_z", "go_home", "suction", "wait"})

#: MOVE の所要時間の許容範囲（ms、ファームウェアの BADARG 範囲と同じ）
MOVE_MS_MIN = 100
MOVE_MS_MAX = 30000

#: 緊急停止・異常終了の後、吸着を維持したまま切断するときの警告
SUCTION_KEPT_WARNING = (
    "吸着を維持したまま切断します。保持物を手で回収し、PUMP を手動で止めてください"
)


def validate_move_ms(move_ms) -> int:
    """move_ms を検証して int で返す（100..30000 の整数）"""
    if isinstance(move_ms, bool):
        raise ValueError(f"move_ms は整数で指定してください（指定値: {move_ms!r}）")
    try:
        value = int(move_ms)
    except (TypeError, ValueError):
        raise ValueError(f"move_ms は整数で指定してください（指定値: {move_ms!r}）")
    if value != move_ms and not (isinstance(move_ms, float) and move_ms.is_integer()):
        raise ValueError(f"move_ms は整数で指定してください（指定値: {move_ms!r}）")
    if not MOVE_MS_MIN <= value <= MOVE_MS_MAX:
        raise ValueError(
            f"move_ms は {MOVE_MS_MIN}〜{MOVE_MS_MAX} の範囲で指定してください（指定値: {move_ms}）"
        )
    return value


def validate_move_tolerance(value) -> float:
    """move_tolerance_mm を検証して float で返す（有限かつ > 0）

    NaN / inf の許容差は到達判定（誤差 > 許容差）を常に偽にして、到達確認を
    無効にしてしまうため拒否する。
    """
    message = f"move_tolerance_mm は 0 より大きい有限の数値で指定してください（指定値: {value!r}）"
    if isinstance(value, bool):
        raise ValueError(message)
    try:
        tol = float(value)
    except (TypeError, ValueError):
        raise ValueError(message)
    if not math.isfinite(tol) or tol <= 0:
        raise ValueError(message)
    return tol


def parse_home_position(value) -> Optional[List[float]]:
    """home_position 設定値を [x, y, z] に変換する（None はそのまま、4要素目以降は無視）"""
    if value is None:
        return None
    try:
        pose = [float(v) for v in list(value)[:3]]
    except (TypeError, ValueError):
        raise ValueError(f"home_position は [x, y, z] で指定してください（指定値: {value!r}）")
    if len(pose) != 3 or not all(math.isfinite(v) for v in pose):
        raise ValueError(f"home_position は [x, y, z] で指定してください（指定値: {value!r}）")
    return pose


def plan_home_legs(base, home, validate, validate_current=None) -> List[tuple]:
    """go_home() の区間を決め、全区間を ``validate(x, y, z)`` に通してから返す（実機・モック共通）

    ``validate_current(x, y, z)`` を渡すと、現在姿勢 ``base`` もそれで検証する（箱・半径だけの
    検査を渡す: 手で動かされた等で箱の外にいるなら、ホームへ向かう経路が未知なので拒否する。
    逆運動学は要求しない。現在姿勢は実際にアームがいる場所であり、工場出荷のホーミング直後の
    ように「いるが戻れない」姿勢からホームへ動くのは正当）。

    (1) 現在 Z がホームより低ければ、その場（現在 XY）で Z を上げる。上げ先はホームの
        高さだが、その XY で実機の逆運動学が届かない高さなら、現在の高さ以上で届く最高の
        高さまでにする（ホームの高さは基部の近くでしか届かないことがある）。現在の高さ
        自体が届かない姿勢（工場出荷のホーミング直後など）なら上昇は省く。
    (2) ホーム XYZ へ移動。

    上昇を減らす方向にしか変えない。各区間はバリデータ（箱・半径・逆運動学）で検証する。
    """
    from src.devices.maxarm import kinematics  # 純粋な計算（ハードウェア非依存）
    if validate_current is not None:
        validate_current(float(base[0]), float(base[1]), float(base[2]))
    legs: List[tuple] = []
    if base[2] < home[2] - 0.1:
        lift_z: Optional[float] = float(home[2])
        if not kinematics.is_reachable(base[0], base[1], lift_z):
            lift_z = kinematics.highest_reachable_z(base[0], base[1], float(base[2]), float(home[2]))
        if lift_z is not None and lift_z > base[2] + 0.1:
            legs.append((float(base[0]), float(base[1]), float(lift_z)))
    legs.append((float(home[0]), float(home[1]), float(home[2])))
    for leg in legs:
        validate(*leg)
    return legs


def device_limits_from_validator(validator: Optional[WorkspaceValidator]) -> Optional[Dict[str, float]]:
    """バリデータの可動域をファームウェアの LIMITS 形式（xmin..zmax, rmin/rmax）に変換する

    r_min / r_max の片方だけが設定されている場合、もう片方は箱の制約を変えない値
    （rmin=0、rmax=箱の最遠点までの距離）で補う（ファームウェアは両方を必要とする）。
    """
    if validator is None:
        return None
    lim = validator.get_limits()
    out = {
        "xmin": float(lim["x"]["min"]), "xmax": float(lim["x"]["max"]),
        "ymin": float(lim["y"]["min"]), "ymax": float(lim["y"]["max"]),
        "zmin": float(lim["z"]["min"]), "zmax": float(lim["z"]["max"]),
    }
    r = lim.get("r")
    if r:
        far = math.hypot(max(abs(out["xmin"]), abs(out["xmax"])),
                         max(abs(out["ymin"]), abs(out["ymax"])))
        out["rmin"] = float(r.get("min", 0.0))
        out["rmax"] = float(r.get("max", far))
    return out


def _status_fields(status) -> str:
    """STATUS の内容をログ用の文字列にする"""
    names = ("x", "y", "z", "moving", "stopped", "pump", "vin_mv", "pulses")
    return " ".join(f"{n}={getattr(status, n)!r}" for n in names if hasattr(status, n))


class MaxArmRobot(InterruptibleDriverMixin):
    """Hiwonder MaxArm の安全ラッパ（吸着ノズル付き）

    公開 API は flow executor が使う move_xyz / move_z / go_home / suction と、
    セッションが使う initialize / set_current_position_as_home / emergency_stop /
    cleanup。ピペット系の操作は持たない（SUPPORTED_ACTIONS 参照）。

    Args:
        maxarm_port: シリアルポート（例: 'COM11'）
        workspace_validator: 可動域バリデータ。None のままでは initialize() が
            RuntimeError を送出する（検証なしでは接続しない）
        client_factory: テスト用。``client_factory(port)`` が MaxArmClient 互換の
            オブジェクトを返す。None なら src.devices.maxarm.client.MaxArmClient。
            呼び出しごとに**新しい**クライアントを返すこと（initialize() は放棄した
            ワーカーの追跡をリセットする。古いワーカーが持つ切断済みのクライアントは
            以後のコマンドを拒否するので、新しい接続と重ならない）
        **device_configs:
            - move_ms: int (1500) 1回の MOVE の所要時間（100..30000 ms）
            - move_tolerance_mm: float (10.0) 到達判定の許容差（mm、各軸。有限かつ > 0）
            - pose_settle_reads: int (5) / pose_settle_interval: float (0.2 s) /
              pose_settle_tolerance: float (3.0 mm) 中断後の静止確認
            - stop_wait_timeout: float (5.0 s) 強制停止後にワーカーを待つ上限
            - watchdog_ms: int (1000) ファームウェアの通信途絶ウォッチドッグ
            - wait_after_move: float (0.3 s) 移動完了後の待機
            - pump_off_on_cleanup: bool (True) 正常終了時、切断前にポンプを止める
            - pump_off_after_estop: bool (False) 緊急停止・移動失敗の後でも切断前に
              ポンプを止める（既定では吸着を維持して警告する）
            - home_position: [x, y, z] または None。設定した場合はそれがホーム
              （セッションは現在位置で上書きせず、home_is_within_limits() で検証する）
    """

    ROBOT_TYPE = "maxarm"
    SUPPORTED_ACTIONS = MAXARM_SUPPORTED_ACTIONS

    def __init__(self,
                 maxarm_port: str,
                 workspace_validator: Optional[WorkspaceValidator] = None,
                 client_factory: Optional[Callable[[str], Any]] = None,
                 **device_configs):
        self.maxarm_port = maxarm_port
        self.workspace_validator = workspace_validator
        self.client_factory = client_factory
        self.device_configs = device_configs
        self._arm_name = "MaxArm"

        self.move_ms = validate_move_ms(device_configs.get("move_ms", 1500))
        self.move_tolerance_mm = validate_move_tolerance(device_configs.get("move_tolerance_mm", 10.0))
        self.watchdog_ms = int(device_configs.get("watchdog_ms", 1000))
        self.wait_after_move = float(device_configs.get("wait_after_move", 0.3))
        self.pump_off_on_cleanup = bool(device_configs.get("pump_off_on_cleanup", True))
        self.pump_off_after_estop = bool(device_configs.get("pump_off_after_estop", False))
        self.home_position = parse_home_position(device_configs.get("home_position"))
        #: home_position が設定で与えられたか（セッションはこれを上書きしない）
        self.home_configured = self.home_position is not None

        # 姿勢トラッキング（InterruptibleDriverMixin が使う）
        self._pose_unknown = False
        self._estop_generation = 0
        # 接続時点の緊急停止の世代（cleanup がこの接続中の緊急停止を判定する）
        self._connect_generation = 0
        self.pose_settle_reads = device_configs.get("pose_settle_reads", 5)
        self.pose_settle_interval = device_configs.get("pose_settle_interval", 0.2)
        self.pose_settle_tolerance = device_configs.get("pose_settle_tolerance", 3.0)
        self.stop_wait_timeout = device_configs.get("stop_wait_timeout", 5.0)

        self.client = None
        self.firmware_info: Dict[str, Any] = {}
        self._initialized = False
        self._last_status = None
        # 最後に読んだ位置（read-back 値）
        self._pose: Optional[List[float]] = None
        # 相対移動の基準: 最後に完了した移動の指令値（read-back のノイズを積算しない）。
        # 位置を読み直した後は read-back 値
        self._commanded: Optional[List[float]] = None
        # 吸着（ポンプ）状態。None = 不明（緊急停止後・失敗後）
        self._suction: Optional[bool] = None
        # 自分で STOP を送り、ファームウェアの停止ラッチが立っている可能性がある
        self._stop_latched = False
        # 緊急停止の状態遷移（世代・姿勢不明）と、それに依存する判断（cleanup の
        # ポンプ停止、CLEAR 後のラッチ解除）を直列化する。STOP の送信はロックの外。
        # RLock: 同じスレッドからの再入でデッドロックしない
        self._state_lock = threading.RLock()

    # ========================================
    # 初期化
    # ========================================

    def _make_client(self):
        if self.client_factory is not None:
            return self.client_factory(self.maxarm_port)
        from src.devices.maxarm.client import MaxArmClient  # 遅延 import（pyserial 不要のため）
        return MaxArmClient(self.maxarm_port)

    async def initialize(self) -> bool:
        """MaxArm に接続し、状態を読んで停止ラッチを解除する

        停止ラッチ（前回の STOP・ウォッチドッグ等）が立っていれば、アームが静止して
        いるときだけ CLEAR する（STATUS の内容を WARNING で記録する）。動作中なら
        解除せず、次の移動の前に静止を確認してから解除する。

        Returns:
            bool: 成功したら True（失敗時は例外を送出する）

        接続を始めてから初期化が完了するまでの間は、姿勢・吸着状態を不明として扱う。
        途中（状態取得・位置の解析・CLEAR）で失敗した場合、cleanup() は異常終了として
        STOP を送り、吸着を維持する（P1）。

        Raises:
            RuntimeError: 可動域バリデータが無い場合、既に初期化済み（接続中）の場合
                （どちらも接続しない）
            RuntimeError / MaxArmError: 接続・状態取得に失敗した場合（切断してから送出）
        """
        if self.workspace_validator is None:
            raise RuntimeError(
                "MaxArm: 可動域バリデータ（workspace）がありません。検証なしでは接続しません"
                "（config.yaml の robots.<id>.workspace を設定してください）"
            )
        if self.client is not None:
            raise RuntimeError(
                "MaxArm: 既に初期化されています。再接続する場合は先に cleanup() を呼んでください"
            )
        try:
            logger.info(f"=== MaxArm 初期化開始 (Port: {self.maxarm_port}) ===")
            self._worker_done = None
            self.client = self._make_client()
            limits = device_limits_from_validator(self.workspace_validator)
            # 初期化が完了するまでは不明（失敗時の cleanup を異常終了として扱わせる）。
            # 成功した場合は下で STATUS から設定し直す
            with self._state_lock:
                self._pose_unknown = True
                self._suction = None
                # 接続を始める時点の世代（接続中の緊急停止もこの接続の異常終了に数える）
                self._connect_generation = self._estop_generation
            self.client.connect(limits=limits, watchdog_ms=self.watchdog_ms)
            self.firmware_info = dict(getattr(self.client, "info", None) or {})
            logger.info(
                "MaxArm ファームウェア: "
                + (" ".join(f"{k}={v}" for k, v in self.firmware_info.items()) or "(情報なし)")
            )

            status = self.client.status()
            self._last_status = status
            self._set_pose_from_status(status)
            self._stop_latched = False
            if status.stopped and status.moving:
                logger.warning(
                    "MaxArm: ファームウェアの停止ラッチが立っていますが、アームが動作中のため"
                    "解除しません。次の移動の前に静止を確認してから解除します。"
                    f" STATUS: {_status_fields(status)}"
                )
                self._stop_latched = True
            elif status.stopped:
                logger.warning(
                    "MaxArm: ファームウェアの停止ラッチが立っています（前回の STOP・"
                    "ウォッチドッグ・緊急停止など）。アームは静止しているため解除します（CLEAR）。"
                    f" STATUS: {_status_fields(status)}"
                    f" INFO: {' '.join(f'{k}={v}' for k, v in self.firmware_info.items())}"
                )
                self.client.clear()
            self._commanded = list(self._pose)
            with self._state_lock:
                # 初期化中に（別スレッドから）緊急停止されていたら、状態を既知にしない
                if self._estop_generation != self._connect_generation:
                    raise RuntimeError("MaxArm: 初期化中に緊急停止されました")
                self._suction = bool(status.pump)
                self._pose_unknown = bool(status.moving)
            if status.moving:
                # 前のセッションの移動が残っている: 次の移動の前に静止を確認させる
                logger.warning("MaxArm: 接続時にアームが動作中でした。次の移動の前に位置を読み直します")
            vin = f", 電源 {status.vin_mv / 1000:.1f}V" if status.vin_mv else ""
            logger.info(
                f"✓ MaxArm 接続成功: 現在位置 X={self._pose[0]:.1f}, Y={self._pose[1]:.1f}, "
                f"Z={self._pose[2]:.1f}, ポンプ={'ON' if self._suction else 'OFF'}{vin}"
            )
            self._initialized = True
            logger.info("=== MaxArm 初期化完了 ===\n")
            return True
        except BaseException as e:  # 中断（KeyboardInterrupt）でも切断してから再送出する
            logger.error(f"MaxArm初期化エラー: {e!r}")
            await self.cleanup()
            raise

    # ========================================
    # InterruptibleDriverMixin のフック
    # ========================================

    def _force_stop_arm(self) -> bool:
        """STOP をベストエフォートで送る（例外を送出しない）"""
        if self.client is None:
            return False
        self._stop_latched = True
        try:
            return bool(self.client.force_stop())
        except Exception as e:
            logger.error(f"MaxArm 強制停止エラー: {e}")
            return False

    def _driver_pose(self) -> List[float]:
        status = self.client.status()
        self._last_status = status
        self._set_pose_from_status(status)
        return list(self._pose)

    def _driver_reports_idle(self) -> bool:
        return self._last_status is None or not self._last_status.moving

    def _on_pose_reread(self, pose) -> None:
        self._pose = [float(v) for v in pose[:3]]
        self._commanded = list(self._pose)

    def _set_pose_from_status(self, status):
        pose = [float(status.x), float(status.y), float(status.z)]
        if not all(math.isfinite(v) for v in pose):
            raise RuntimeError(f"MaxArm: 読み取った位置が有限の数値ではありません: {pose}")
        self._pose = pose

    # ========================================
    # 内部ヘルパ
    # ========================================

    def _require_client(self):
        if self.client is None:
            raise RuntimeError("MaxArmが初期化されていません")
        if self.workspace_validator is None:
            raise RuntimeError("MaxArm: 可動域バリデータがありません。移動しません")

    def _begin_motion(self) -> int:
        """公開の移動操作の入口: 放棄したワーカーを確認し、緊急停止の世代を返す"""
        self._require_client()
        self._refuse_if_worker_alive()
        return self._estop_generation

    async def _base_pose(self, gen) -> List[float]:
        """相対移動・ホーム復帰の基準位置（不明なら静止を確認して読み直す）"""
        if self._pose_unknown or self._commanded is None:
            self._pose_unknown = True
            await self._read_pose(gen)
        return list(self._commanded)

    def _prepare_move(self, gen=None):
        """自分で送った STOP の停止ラッチを解除する（位置の再確認後にのみ呼ぶ）

        操作の入口の世代 gen から緊急停止が入っていれば、解除せずに RuntimeError。
        CLEAR の送信中に緊急停止が割り込んだ場合は、STOP を送り直してラッチを
        立て直してから RuntimeError（緊急停止の STOP を CLEAR で消したままにしない）。
        """
        self._check_estop(gen, "移動の準備")
        self._refuse_if_worker_alive()
        if self._stop_latched:
            if self._pose_unknown:
                raise RuntimeError("MaxArm: 停止後の位置が未確認のため、停止ラッチを解除しません")
            logger.warning("MaxArm: 停止後の位置を確認しました。停止ラッチを解除します（CLEAR）")
            self.client.clear()
            # ラッチ解除の記録を先に書き、その後で世代を確認する。emergency_stop() は
            # 世代を増やしてから STOP を送る（ラッチを立てる）ので、この確認が世代の
            # 変化を見落とすことはなく、緊急停止のラッチを False で上書きしたままにしない
            with self._state_lock:
                self._stop_latched = False
                raced = gen is not None and self._estop_generation != gen
                if raced:
                    self._stop_latched = True
                    self._pose_unknown = True
            if raced:
                self._force_stop_arm()
                raise RuntimeError("MaxArm: 停止ラッチの解除中に緊急停止されました")

    async def _move_to(self, x: float, y: float, z: float, gen=None):
        """検証済みの目標へ1回移動する（到達確認まで待つ）"""
        logger.info(f"MaxArm: XYZ({x:.1f}, {y:.1f}, {z:.1f})へ移動中...")
        status = await self._run_blocking(
            self.client.move_xyz_and_wait, x, y, z, self.move_ms, self.move_tolerance_mm,
            estop_before=gen,
        )
        self._commanded = [float(x), float(y), float(z)]
        if status is not None:
            self._last_status = status
            self._set_pose_from_status(status)
        else:
            self._pose = list(self._commanded)
        await asyncio.sleep(self.wait_after_move)
        # 待機中の緊急停止（GUI の Stop）を取りこぼさない: 移動は完了扱いにしない
        self._check_estop(gen, "移動後の待機")

    def _validate_xyz(self, x, y, z):
        if self.workspace_validator is None:
            raise RuntimeError("MaxArm: 可動域バリデータがありません。移動しません")
        self.workspace_validator.validate_xyz(x, y, z)

    def _move_failed(self, label: str, e: BaseException, gen: int):
        """移動の失敗処理: 必要なら STOP を1回送り、姿勢不明にする（自動ホーム復帰はしない）

        - 操作中に緊急停止された: 緊急停止が STOP を送り済みなので送らない
        - ファームウェアが停止ラッチ中のため MOVE を拒否した（ERR STOPPED）: アームは
          動いていないので送らない（他者のラッチを自分のものとして扱わない）
        - それ以外（MOVE の応答喪失、到達誤差、タイムアウト、通信エラー、静止しない）:
          アームが動いている可能性があるので STOP を送る
        """
        self._pose_unknown = True
        if self._estop_generation != gen:
            note = "緊急停止済み"
        elif getattr(e, "code", None) == "STOPPED":
            note = "停止ラッチ中のため MOVE は拒否されました"
        else:
            stopped = self._force_stop_arm()
            note = "STOP を送りました" if stopped else "STOP を送れませんでした"
        logger.error(
            f"MaxArm {label}エラー: {e}（{note}。姿勢不明として扱います。自動復帰はしません）"
        )

    # ========================================
    # 高レベル操作（flow executor から呼ばれる）
    # ========================================

    async def move_xyz(self, x: float, y: float, z: float):
        """XYZ 絶対位置へ安全に移動

        Raises:
            WorkspaceViolationError: 目標座標が可動域外の場合（コマンドは送信されない）
            RuntimeError: 未初期化、位置が安定しない、緊急停止された場合
            MaxArmError: 通信・到達失敗（STOP を送ってから送出）
        """
        self._require_client()
        self._validate_xyz(x, y, z)
        gen = self._begin_motion()
        try:
            if self._pose_unknown:
                await self._read_pose(gen)
            self._prepare_move(gen)
            await self._move_to(x, y, z, gen)
            logger.info("✓ MaxArm: XYZ移動完了")
        except ValidationError:
            raise
        except Exception as e:
            self._move_failed("XYZ移動", e, gen)
            raise
        except BaseException:
            # 中断（移動後の待機中のキャンセルなど）: 操作は完了していない。姿勢不明にして
            # 次の移動の前に読み直させ、cleanup を異常終了（STOP、吸着維持）にする
            self._pose_unknown = True
            raise

    async def move_z(self, distance: float):
        """Z 軸の相対移動（正=上昇、負=下降）

        MaxArm に相対移動コマンドは無いため、(X, Y, Z+distance) への絶対移動として送る。
        基準は最後に完了した移動の指令値（位置の読み直し後は読み直した値）。
        read-back 値との差は各軸 move_tolerance_mm 以内（モジュールの説明を参照）。

        Raises:
            WorkspaceViolationError: 移動後が可動域外の場合（コマンドは送信されない）
        """
        gen = self._begin_motion()
        try:
            base = await self._base_pose(gen)
            self.workspace_validator.validate_z_relative(base[2], distance)
            self._validate_xyz(base[0], base[1], base[2] + distance)
            direction = "上昇" if distance > 0 else "下降"
            logger.info(f"MaxArm: Z軸 {abs(distance):.1f}mm {direction}中...")
            self._prepare_move(gen)
            await self._move_to(base[0], base[1], base[2] + distance, gen)
            logger.info("✓ MaxArm: Z軸移動完了")
        except ValidationError:
            raise
        except Exception as e:
            self._move_failed("Z軸移動", e, gen)
            raise
        except BaseException:
            # 中断（移動後の待機中のキャンセルなど）: 操作は完了していない。姿勢不明にして
            # 次の移動の前に読み直させ、cleanup を異常終了（STOP、吸着維持）にする
            self._pose_unknown = True
            raise

    def _home_legs(self, base) -> List[tuple]:
        """go_home() の各区間を返す。全区間を最初のコマンドの前に検証する

        (1) 現在 Z がホームより低ければ、その場で Z を上げる（届く高さまで、plan_home_legs 参照）
        (2) ホーム XYZ へ移動
        """
        return plan_home_legs(base, self.home_position, self._validate_xyz, self._validate_box)

    def _validate_box(self, x, y, z):
        """現在姿勢の検査: 箱・半径のみ（逆運動学は要求しない。plan_home_legs 参照）"""
        if self.workspace_validator is None:
            raise RuntimeError("MaxArm: 可動域バリデータがありません。移動しません")
        WorkspaceValidator.validate_xyz(self.workspace_validator, x, y, z)

    async def go_home(self):
        """ホームポジションへ復帰（Z上昇 → XYZ）

        Raises:
            RuntimeError: ホーム未設定・未初期化の場合（アームは動かない）
            WorkspaceViolationError: いずれかの区間が可動域外の場合（アームは動かない）
        """
        self._require_client()
        if self.home_position is None:
            raise RuntimeError(
                "MaxArm: ホームポジションが設定されていません。"
                "set_current_position_as_home() または home_position を設定してください"
            )
        gen = self._begin_motion()
        try:
            logger.info("MaxArm: ホームポジションへ復帰中...")
            base = await self._base_pose(gen)
            try:
                legs = self._home_legs(base)
            except ValidationError as e:
                logger.error(f"ホーム復帰を拒否しました（可動域外）: {e}")
                raise
            self._prepare_move(gen)
            for leg in legs:
                await self._move_to(*leg, gen=gen)
            logger.info("✓ MaxArm: ホームポジション復帰完了")
        except ValidationError:
            raise
        except Exception as e:
            self._move_failed("ホーム復帰", e, gen)
            raise
        except BaseException:
            # 中断（移動後の待機中のキャンセルなど）: 操作は完了していない。姿勢不明にして
            # 次の移動の前に読み直させ、cleanup を異常終了（STOP、吸着維持）にする
            self._pose_unknown = True
            raise

    async def suction(self, on: bool = True):
        """吸着ノズルのポンプを ON / OFF する

        失敗・中断した場合、吸着状態は不明（suction_state = None）になる。
        """
        self._require_client()
        on = bool(on)
        logger.info(f"MaxArm: 吸着 {'ON' if on else 'OFF'}...")
        try:
            self.client.pump(on)
        except BaseException as e:
            self._suction = None
            logger.error(f"MaxArm 吸着{'ON' if on else 'OFF'}エラー: {e!r}（吸着状態は不明です）")
            raise
        self._suction = on
        logger.info(f"✓ MaxArm: 吸着 {'ON' if on else 'OFF'} 完了")

    @property
    def suction_state(self) -> Optional[bool]:
        """吸着（ポンプ）状態。None は不明（緊急停止後・失敗後）"""
        return self._suction

    # ========================================
    # 位置・ホーム
    # ========================================

    def get_current_position(self) -> List[float]:
        """最後に読んだ位置 [X, Y, Z]（mm）を返す（通信はしない）

        姿勢不明の間は最後に確認できた値を返す（次の移動の前に読み直される）。
        """
        self._require_client()
        if self._pose is None:
            raise RuntimeError("MaxArm: 現在位置がまだ読まれていません")
        return list(self._pose)

    def set_current_position_as_home(self) -> List[float]:
        """現在位置をアームから読み、ホームポジション [X, Y, Z] として設定する

        Raises:
            RuntimeError: 未初期化、またはアームが動作中の場合
        """
        self._require_client()
        status = self.client.status()
        self._last_status = status
        if status.moving:
            raise RuntimeError("MaxArm: アームが動作中のため、ホームポジションを設定できません")
        self._set_pose_from_status(status)
        if not self._pose_unknown:
            # 静止中の read-back 値を相対移動の基準にもする
            self._commanded = list(self._pose)
        self.home_position = list(self._pose)
        if self.home_is_within_limits():
            logger.info(
                f"✓ MaxArm: ホームポジション設定完了: X={self.home_position[0]:.1f}, "
                f"Y={self.home_position[1]:.1f}, Z={self.home_position[2]:.1f}"
            )
        return list(self.home_position)

    def home_is_within_limits(self) -> bool:
        """ホームポジション（設定値または現在位置から設定した値）が可動域内か検証する

        可動域外、または可動域バリデータが無い場合は警告を出して False。
        ホーム未設定なら True（検証する対象が無い。go_home() は未設定を拒否する）。
        """
        if self.home_position is None:
            return True
        if self.workspace_validator is None:
            logger.warning("警告: MaxArm に可動域バリデータが無いため、ホームポジションを検証できません")
            return False
        home = self.home_position
        try:
            self.workspace_validator.validate_xyz(home[0], home[1], home[2])
        except ValidationError as e:
            logger.warning(
                "警告: MaxArm のホームポジションが workspace 可動域外です。"
                "go_home() はこのホームへの移動を拒否します。home_position の設定を直すか、"
                f"アームを可動域内へ手動で移動してから再初期化してください。\n{e}"
            )
            return False
        return True

    # ========================================
    # 緊急停止・切断
    # ========================================

    def emergency_stop(self):
        """MaxArm をその場で止める（同期・ベストエフォート、例外を送出しない）

        STOP を送る（ポンプはそのまま: 吸着中のワークを落とさない）。
        停止位置と吸着状態は不明として扱う。切断はしないので、この後 cleanup() を呼ぶこと。
        """
        logger.warning("=== MaxArm 緊急停止を実行します ===")
        if self.client is None:
            logger.warning("MaxArm は接続されていません")
            return
        # 順序が重要: 世代を先に増やしてから姿勢不明にする（InterruptibleDriverMixin 参照）。
        # ロックで cleanup のポンプ停止の判断と直列化する（STOP の送信はロックの外）
        with self._state_lock:
            self._estop_generation += 1
            self._pose_unknown = True
            self._suction = None
        if self._force_stop_arm():
            logger.warning("✓ MaxArm: 停止しました（STOP）")
        else:
            logger.error("MaxArm の強制停止に失敗しました（未接続または通信エラー）")
        logger.warning("=== MaxArm 緊急停止処理完了 ===")

    @property
    def ended_abnormally(self) -> bool:
        """この接続中に緊急停止があったか、最後の移動が中断・失敗して姿勢不明か"""
        return self._estop_generation != self._connect_generation or self._pose_unknown

    async def cleanup(self):
        """切断する。アームは動かさない

        1. 緊急停止の後・姿勢不明（移動の中断・失敗）の後は、まず STOP を送る
           （動いている可能性のあるアームを止めてからウォッチドッグを外す）。
        2. 吸着ポリシー:
           - 正常終了（緊急停止なし・姿勢既知）: pump_off_on_cleanup（既定 True）なら
             ポンプを止める。
           - 異常終了: 既定では吸着を維持して（保持物を落とさない）大きな警告を出す。
             pump_off_after_estop=True のときだけポンプを止める。
        3. 切断する。

        STOP・ポンプ停止の失敗や中断（CancelledError / KeyboardInterrupt）で切断が
        飛ばされないよう、手順ごとに BaseException を捕捉して続行する。
        中断はすべての手順を試みた後に再送出する。

        正常終了かどうかの判定とポンプ停止のコマンドは emergency_stop() と同じロックの
        中で行う（判定の後に割り込んだ緊急停止で保持物を離さない）。ポンプ停止の途中で
        緊急停止された場合、その STOP はポンプ停止の応答の後に送られる。
        """
        client = self.client
        if client is None:
            return
        logger.info("=== MaxArm 切断開始 ===")
        interrupted = None
        abnormal = self.ended_abnormally
        connected = getattr(client, "connected", True)

        if self._worker_alive():
            logger.error(
                f"MaxArm: 中断した {self._worker_name} がまだ戻っていないまま切断します"
            )

        if abnormal:
            try:
                logger.warning("MaxArm: 緊急停止・移動失敗の後のため、切断前に STOP を送ります")
                self._force_stop_arm()
            except BaseException as e:  # 中断されても切断は続ける
                logger.error(f"MaxArm STOP 中に中断されました: {e!r}")
                interrupted = e

        with self._state_lock:
            # 判定をロックの中で取り直す（一度異常と判定したら異常のまま）
            abnormal = abnormal or self.ended_abnormally
            want_pump_off = self.pump_off_after_estop if abnormal else self.pump_off_on_cleanup
            pump_off = want_pump_off and connected
            if pump_off:
                try:
                    client.pump(False)
                    self._suction = False
                    logger.info("✓ MaxArm: ポンプを停止しました")
                except Exception as e:
                    self._suction = None
                    logger.error(f"MaxArm ポンプ停止エラー（切断は続行します）: {e}")
                except BaseException as e:  # 中断されても切断は続ける
                    self._suction = None
                    logger.error(f"MaxArm ポンプ停止中に中断されました: {e!r}")
                    if interrupted is None:
                        interrupted = e
        if not pump_off and abnormal and self._suction is not False:
            logger.warning("!" * 60)
            logger.warning(f"!!! MaxArm: {SUCTION_KEPT_WARNING} !!!")
            logger.warning("!" * 60)

        try:
            client.close()
            logger.info("✓ MaxArm切断完了")
        except Exception as e:
            logger.error(f"MaxArm切断エラー: {e}")
        except BaseException as e:
            logger.error(f"MaxArm切断中に中断されました: {e!r}")
            if interrupted is None:
                interrupted = e
        self.client = None
        self._initialized = False

        logger.info("=== MaxArm 切断完了 ===")
        if interrupted is not None:
            raise interrupted

    async def __aenter__(self):
        if not await self.initialize():
            raise RuntimeError("MaxArmの初期化に失敗しました")
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await self.cleanup()
