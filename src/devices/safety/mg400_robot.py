"""
L1層: 安全ラッパ層（MG400Robot）

Dobot MG400（TCP/IP）を SAIGEN のロボット種別 ``type: mg400`` として LabRobot /
MaxArmRobot と同じ安全方針でラップする。flow plan Revision 3（2026-10-30、Astra 承認）の
実装。I/O はすべて ``src.devices.mg400.control.Controller`` のワーカースレッド 1 本が
持ち、このクラスはそのワーカーへ要求を出して結果を待つだけ（F2）。

- 使えるアクションは move_xyz / move_z / go_home / wait だけ（F1）。
- 目標は MovL（直線移動）で 1 回ずつ送り、到達（RobotMode 5 と、コントローラの
  PositiveSolution で計算した姿勢が 0.5 mm / 0.2 deg 以内）を確認してから次へ進む。
- 姿勢は毎回 ``PositiveSolution(GetAngle(), 0, 0)`` で測る（F4）。GetPose() は記録だけ。
- 各 MovL の前に、測った開始姿勢から 6 段階の検証（F5）: 開始姿勢が箱の中、目標が箱の中、
  最小移動長 1.0 mm、最大移動長 ``max_move_mm``、線分の水平半径（解析的）、5 mm ごとの
  InverseSolution サンプルが ``joint_limits`` の中で隣接差 5 deg 未満。どれかに外れたら
  何も送らずに拒否する。
- 停止ラッチ: emergency_stop() は ``request_stop()`` を呼ぶだけで即返る。ワーカーが
  jog ツールと同じ停止シーケンス（ResetRobot, DisableRobot, 確認ポーリング, 必要なら
  EmergencyStop）を自分の 15 s の期限で走らせる（F3）。
- エラー後の自動ホーム復帰はしない（NO_RECOVERY_HOME_TYPES）。
- 実機は、呼び出し側（run_flow の confirm_tty / GUI の確認ダイアログ）が ``confirm()`` を
  呼んだときだけ initialize() が制御コマンド（ResetRobot 以降）へ進む（F7）。
"""
import asyncio
import logging
import math
import os
import threading
import time
from typing import Callable, Dict, List, Optional, Sequence

from src.devices.mg400 import control, protocol
from src.devices.mg400.control import StopRequested
from src.devices.safety.validators import ValidationError, ViolationDetail, WorkspaceValidator, WorkspaceViolationError

logger = logging.getLogger(__name__)

#: MG400 で使える flow のアクション（executor / run_flow の能力チェック用）
MG400_SUPPORTED_ACTIONS = frozenset({"move_xyz", "move_z", "go_home", "wait"})

#: v1 の速度比の上限（protocol.MAX_SPEED_RATIO 30 はツール全体の上限。C3 の記録まで 10）
MAX_SPEED_RATIO_V1 = 10
#: 1 回の MovL の最小・最大の移動長（mm、ユークリッド距離）
MIN_MOVE_MM = 1.0
MAX_MOVE_MM_CEILING = 50.0
#: 線分上の InverseSolution サンプル間隔と、隣接サンプルの関節角の差の上限
SAMPLE_MM = 5.0
SAMPLE_JOINT_STEP_DEG = 5.0
#: go_home で「すでにホーム」とみなす距離（= MIN_MOVE_MM）
AT_HOME_MM = MIN_MOVE_MM
#: 停止後に cleanup() がワーカーの停止シーケンスを待つ上限（F3）
CLEANUP_JOIN_S = control.STOP_TOTAL_S + control.TIMEOUT_S + 1.0
#: cleanup() waits this much longer than the shutdown's own bound before reporting it unfinished
CLEANUP_MARGIN_S = 5.0

#: ``joint_limits`` が収まっていなければならない範囲（deg）。Dobot の公開仕様の値で、
#: この研究室で測ったものではない。J4 は ±180（Dobot のページは ±180 と ±360 の両方を
#: 載せているので小さい方）。config.yaml の値はこれを狭める方向にしか置けない。
JOINT_ENVELOPE_DEG = ((-160.0, 160.0), (-25.0, 85.0), (-25.0, 105.0), (-180.0, 180.0))


def _finite_number(value, what: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{what} は数値で指定してください（指定値: {value!r}）")
    v = float(value)
    if not math.isfinite(v):
        raise ValueError(f"{what} は有限の数値で指定してください（指定値: {value!r}）")
    return v


def validate_speed_ratio(value) -> int:
    """``speed_ratio``: 1..MAX_SPEED_RATIO_V1 の整数"""
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"speed_ratio は整数で指定してください（指定値: {value!r}）")
    if not 1 <= value <= MAX_SPEED_RATIO_V1:
        raise ValueError(f"speed_ratio は 1〜{MAX_SPEED_RATIO_V1} で指定してください（v1 の上限; "
                         f"指定値: {value}）")
    return value


def validate_max_move_mm(value) -> float:
    v = _finite_number(value, "max_move_mm")
    if not MIN_MOVE_MM <= v <= MAX_MOVE_MM_CEILING:
        raise ValueError(f"max_move_mm は {MIN_MOVE_MM:g}〜{MAX_MOVE_MM_CEILING:g} mm で指定してください"
                         f"（指定値: {value}）")
    return v


def parse_home_position(value) -> List[float]:
    """``home_position``: [x, y, z]（必須）"""
    if value is None:
        raise ValueError("home_position は [x, y, z] で指定してください（MG400 では必須）")
    try:
        items = list(value)
    except TypeError:
        raise ValueError(f"home_position は [x, y, z] で指定してください（指定値: {value!r}）")
    if len(items) != 3:
        raise ValueError(f"home_position は [x, y, z] で指定してください（指定値: {value!r}）")
    return [_finite_number(v, f"home_position[{i}]") for i, v in enumerate(items)]


def parse_joint_limits(value) -> List[List[float]]:
    """``joint_limits``: 4 つの [min, max]（deg）。JOINT_ENVELOPE_DEG の中で、min < max"""
    try:
        items = list(value)
    except TypeError:
        raise ValueError(f"joint_limits は 4 つの [min, max] で指定してください（指定値: {value!r}）")
    if len(items) != 4:
        raise ValueError(f"joint_limits は J1..J4 の 4 つの [min, max] で指定してください（指定値: {value!r}）")
    out = []
    for j, pair in enumerate(items, 1):
        try:
            lo, hi = (float(_finite_number(v, f"joint_limits J{j}")) for v in list(pair))
        except (TypeError, ValueError) as e:
            if isinstance(e, ValueError) and "joint_limits" in str(e):
                raise
            raise ValueError(f"joint_limits の J{j} は [min, max] で指定してください（指定値: {pair!r}）")
        if not lo < hi:
            raise ValueError(f"joint_limits の J{j} は min < max で指定してください（指定値: {pair!r}）")
        env_lo, env_hi = JOINT_ENVELOPE_DEG[j - 1]
        if lo < env_lo or hi > env_hi:
            raise ValueError(f"joint_limits の J{j} {[lo, hi]} は公開仕様の範囲 {[env_lo, env_hi]} の外です"
                             "（狭める方向にしか設定できません）")
        out.append([lo, hi])
    return out


def segment_radius_range(a: Sequence[float], b: Sequence[float]):
    """線分 a→b の水平半径 hypot(x, y) の最小・最大（解析的）。最小は xy 投影上で z 軸に
    最も近い点（線分内に挟む）、最大は端点。投影が 1 点（純粋な z 移動）ならその点の半径"""
    ax, ay, bx, by = float(a[0]), float(a[1]), float(b[0]), float(b[1])
    dx, dy = bx - ax, by - ay
    length2 = dx * dx + dy * dy
    r_a, r_b = math.hypot(ax, ay), math.hypot(bx, by)
    if length2 == 0.0:
        return r_a, r_a
    t = -(ax * dx + ay * dy) / length2
    t = min(1.0, max(0.0, t))
    r_min = math.hypot(ax + t * dx, ay + t * dy)
    return min(r_min, r_a, r_b), max(r_a, r_b)


def plan_samples(a: Sequence[float], b: Sequence[float], step_mm: float = SAMPLE_MM) -> List[List[float]]:
    """線分 a→b 上の点を ``step_mm`` ごとに（両端を含めて）返す（50 mm なら最大 11 点）"""
    dist = math.dist(a[:3], b[:3])
    n = max(1, int(math.ceil(dist / step_mm - 1e-9)))
    return [[a[i] + (b[i] - a[i]) * k / n for i in range(3)] for k in range(n + 1)]


def _violation(message: str, target: dict, axis: str, value: float, lo: float, hi: float) -> WorkspaceViolationError:
    return WorkspaceViolationError(message=message, target_position=target,
                                   violations=[ViolationDetail(axis=axis, current_value=value,
                                                               min_value=lo, max_value=hi)])


class MG400Robot:
    """Dobot MG400 の安全ラッパ（flow plan F1〜F7）

    Args:
        mg400_address: アームの IP アドレス（数値リテラル。--mock では使わない）
        workspace_validator: このロボット専用の可動域（robots.<id>.workspace だけから作る）。
            None のままでは initialize() を拒否する
        controller_factory: テスト・mock 用。``controller_factory(address, folder)`` が
            ``control.Controller`` 互換を返す。None なら実機へ接続する Controller
        record_folder: 記録フォルダ（report.json, feedback.*）。None なら logs/mg400_flow/<UTC>
        confirmed: 人の Enter / 確認ダイアログを経たか。False のままでは initialize() が
            制御コマンドの前で拒否する（mock は True で作る）
        **device_configs: home_position（必須）, joint_limits（必須）, speed_ratio（既定 10）,
            max_move_mm（既定 50）, playback_note（記録だけ）, controller_version（記録だけ）
    """
    #: latch_stop() only sets a latch, without logging: the session and run_flow's Ctrl+C relay
    #: send it before any stop that may block (a log call, a serial STOP)
    STOP_NEVER_BLOCKS = True


    ROBOT_TYPE = "mg400"
    SUPPORTED_ACTIONS = MG400_SUPPORTED_ACTIONS

    def __init__(self, mg400_address: str, workspace_validator: Optional[WorkspaceValidator] = None,
                 controller_factory: Optional[Callable[[str, str], object]] = None,
                 record_folder: Optional[str] = None, confirmed: bool = False, **device_configs):
        self.mg400_address = mg400_address
        self.workspace_validator = workspace_validator
        self.controller_factory = controller_factory
        self.record_folder = record_folder
        self.device_configs = device_configs
        self.confirmed = bool(confirmed)
        self.home_position = parse_home_position(device_configs.get("home_position"))
        self.home_configured = True
        self.joint_limits = parse_joint_limits(device_configs.get("joint_limits"))
        self.speed_ratio = validate_speed_ratio(device_configs.get("speed_ratio", MAX_SPEED_RATIO_V1))
        self.max_move_mm = validate_max_move_mm(device_configs.get("max_move_mm", MAX_MOVE_MM_CEILING))
        self.playback_note = device_configs.get("playback_note")
        self.controller_version = device_configs.get("controller_version")
        self._ctl = None                      # the Controller while connected
        self._stop_before_connect: Optional[str] = None   # a stop requested before the controller existed (F6)
        self._pose: Optional[List[float]] = None   # the last measured [x, y, z, r]
        self._r_at_connect: Optional[float] = None
        self._initialized = False
        self._lock = threading.Lock()
        self._shutdown_thread: Optional[threading.Thread] = None   # the one shutdown of this connection (cleanup)
        self._shutdown_box: List[dict] = []

    # ------------------------------------------------------------------
    # 設定・確認
    # ------------------------------------------------------------------
    def confirm(self):
        """人の Enter（run_flow）または確認ダイアログ（GUI）を経た: 制御コマンドへ進んでよい"""
        self.confirmed = True

    def _default_folder(self) -> str:
        import datetime
        return os.path.join(control.LOG_ROOT_FLOW,
                            datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ"))

    def _make_controller(self):
        folder = self.record_folder or self._default_folder()
        self.record_folder = folder
        if self.controller_factory is not None:
            return self.controller_factory(self.mg400_address, folder)
        address = control.fixed_address(self.mg400_address)
        return control.Controller(address, folder)

    # ------------------------------------------------------------------
    # ワーカーへの要求
    # ------------------------------------------------------------------
    async def _await(self, fut, what: str):
        """ワーカーの Future を待つ。キャンセル / Ctrl+C は停止ラッチにしてから再送出する"""
        try:
            return await asyncio.wrap_future(fut)
        except (asyncio.CancelledError, KeyboardInterrupt):
            self._force_stop_arm(f"{what} の途中で中断")
            raise

    def _force_stop_arm(self, reason: str = "cancelled") -> bool:
        """停止ラッチを立てる（ベストエフォート、ブロックしない、例外を送出しない）。
        コントローラがまだ無ければ、接続前の停止として記録する（initialize() が最初に見る）"""
        with self._lock:
            ctl = self._ctl
            if ctl is None:
                if self._stop_before_connect is None:
                    self._stop_before_connect = reason
                return False
        try:
            ctl.request_stop(reason)
            return True
        except Exception as e:  # noqa: BLE001
            logger.error(f"MG400: 停止要求に失敗しました: {e!r}")
            return False

    def _require(self):
        if self._ctl is None or not self._initialized:
            raise RuntimeError("MG400 が初期化されていません")
        if self.workspace_validator is None:
            raise RuntimeError("MG400: 可動域バリデータがありません。移動しません")

    # ------------------------------------------------------------------
    # 初期化
    # ------------------------------------------------------------------
    async def initialize(self) -> bool:
        """接続、ゲート、開始姿勢、人の確認、ResetRobot、EnableRobot（F6 の順）

        各段階の前に停止ラッチを確かめる（ワーカーが要求ごとに見る）。失敗・中断では
        cleanup() を呼んでから再送出する（停止シーケンスはワーカーが済ませている）。
        """
        if self.workspace_validator is None:
            raise RuntimeError("MG400: 可動域バリデータ（workspace）がありません。検証なしでは接続しません"
                               "（config.yaml の robots.<id>.workspace を設定してください）")
        if self._ctl is not None:
            raise RuntimeError("MG400: 既に初期化されています。再接続する場合は先に cleanup() を呼んでください")
        logger.info(f"=== MG400 初期化開始 (record: {self.record_folder or '(default)'}) ===")
        try:
            with self._lock:
                if self._stop_before_connect is not None:
                    raise RuntimeError(f"MG400: 接続前に停止要求がありました（{self._stop_before_connect}）: 接続しません")
                self._ctl = self._make_controller()
                ctl = self._ctl
            ctl.start()
            ctl.note(dict(tool="src.devices.safety.mg400_robot", controller_version_typed=self.controller_version,
                          playback_note_typed=self.playback_note, speed_ratio=self.speed_ratio,
                          max_move_mm=self.max_move_mm, joint_limits=self.joint_limits,
                          home_position=self.home_position,
                          workspace=self.workspace_validator.get_limits()))
            gate = await self._await(ctl.open(), "接続")      # an ErrorID -1 names the remote-control mode
            logger.info(f"MG400: 接続 OK（first packet: {gate.get('first_packet')}）")
            pose = await self._await(ctl.pose(), "開始姿勢の取得")
            self._pose, self._r_at_connect = list(pose), float(pose[3])
            self._check_box(pose, "開始姿勢")
            if not self.confirmed:
                raise RuntimeError("MG400: 実機ランの確認（ターミナルの Enter / GUI の確認ダイアログ）を経て"
                                   "いません。制御コマンドは送りません")
            logger.info(f"MG400: 開始姿勢 X={pose[0]:.2f}, Y={pose[1]:.2f}, Z={pose[2]:.2f}, R={pose[3]:.2f}"
                        f"（PositiveSolution(GetAngle(),0,0)）。ResetRobot / EnableRobot を送ります")
            await self._await(ctl.take_control(), "ResetRobot")
            enabled = await self._await(ctl.enable(), "EnableRobot")
            self._pose = list(enabled["pose"])
            self._check_box(self._pose, "有効化後の姿勢")
            self._initialized = True
            logger.info(f"✓ MG400 接続・有効化完了 (RobotMode 5 after {enabled['enable_s']:.2f} s)")
            return True
        except BaseException as e:
            logger.error(f"MG400 初期化エラー: {e!r}")
            await self.cleanup()
            raise

    # ------------------------------------------------------------------
    # 検証（F5）
    # ------------------------------------------------------------------
    def _check_box(self, pose: Sequence[float], what: str):
        """箱・半径だけ（WorkspaceValidator.validate_xyz）"""
        try:
            self.workspace_validator.validate_xyz(float(pose[0]), float(pose[1]), float(pose[2]))
        except WorkspaceViolationError as e:
            raise WorkspaceViolationError(message=f"MG400: {what}が可動域外です: {e.message}",
                                          target_position=e.target_position, violations=e.violations)

    def _check_segment(self, start: Sequence[float], target: Sequence[float]):
        """F5 の 3〜5: 最小・最大の移動長、線分の水平半径"""
        dist = math.dist(start[:3], target[:3])
        tgt = {"x": target[0], "y": target[1], "z": target[2]}
        if dist < MIN_MOVE_MM:
            raise _violation(f"MG400: 移動長 {dist:.3f} mm が最小 {MIN_MOVE_MM:g} mm 未満です（到達判定の箱 "
                             f"0.866 mm より短い移動は受け付けません）", tgt, "distance", dist, MIN_MOVE_MM,
                             self.max_move_mm)
        if dist > self.max_move_mm:
            raise _violation(f"MG400: 移動長 {dist:.2f} mm が max_move_mm {self.max_move_mm:g} mm を超えます",
                             tgt, "distance", dist, MIN_MOVE_MM, self.max_move_mm)
        v = self.workspace_validator
        if v.has_radius_limits:
            r_lo, r_hi = segment_radius_range(start, target)
            lo = v.r_min if v.r_min is not None else 0.0
            hi = v.r_max if v.r_max is not None else math.inf
            if r_lo < lo:
                raise _violation(f"MG400: 線分の水平半径の最小 {r_lo:.2f} mm が r_min {lo:g} mm を下回ります",
                                 tgt, "R", r_lo, lo, hi)
            if r_hi > hi:
                raise _violation(f"MG400: 線分の水平半径の最大 {r_hi:.2f} mm が r_max {hi:g} mm を超えます",
                                 tgt, "R", r_hi, lo, hi)
        return dist

    async def _check_joints_along(self, start: Sequence[float], target: Sequence[float]):
        """F5 の 6: 5 mm ごとの InverseSolution サンプルが joint_limits の中、隣接差 5 deg 未満"""
        samples = plan_samples(start, target)
        previous = None
        for k, p in enumerate(samples):
            try:
                joints = await self._await(self._ctl.inverse([p[0], p[1], p[2], self._r_at_connect]),
                                           "InverseSolution")
            except protocol.MG400CommandError as e:
                raise _violation(f"MG400: コントローラが InverseSolution を拒否しました（サンプル {k + 1}/"
                                 f"{len(samples)}: {[round(x, 3) for x in p]}）: {e}",
                                 {"x": p[0], "y": p[1], "z": p[2]}, "reachability", float("nan"),
                                 float("nan"), float("nan"))
            for j, (angle, (lo, hi)) in enumerate(zip(joints, self.joint_limits), 1):
                if angle < lo or angle > hi:
                    raise _violation(f"MG400: サンプル {k + 1}/{len(samples)} {[round(x, 3) for x in p]} の "
                                     f"J{j} {angle:.3f} deg が joint_limits {[lo, hi]} の外です",
                                     {"x": p[0], "y": p[1], "z": p[2]}, f"J{j}", angle, lo, hi)
            if previous is not None:
                step = max(abs(a - b) for a, b in zip(joints[:4], previous[:4]))
                if step >= SAMPLE_JOINT_STEP_DEG:
                    raise _violation(f"MG400: サンプル {k}/{len(samples)} → {k + 1} で関節角が {step:.2f} deg "
                                     f"変わります（上限 {SAMPLE_JOINT_STEP_DEG:g} deg: 特異点や急な姿勢変化の疑い）",
                                     {"x": p[0], "y": p[1], "z": p[2]}, "joint_step", step, 0.0,
                                     SAMPLE_JOINT_STEP_DEG)
            previous = joints
        return samples

    async def _measured_pose(self) -> List[float]:
        pose = await self._await(self._ctl.pose(), "姿勢の取得")
        self._pose = list(pose)
        return self._pose

    async def _move_to(self, target_xyz: Sequence[float], label: str):
        """検証済みでない目標 1 点へ: 測定 → F5 の 1〜6 → MovL → 到達確認"""
        start = await self._measured_pose()
        self._check_box(start, "開始姿勢")                                  # 1
        self._check_box(target_xyz, "目標")                                 # 2
        dist = self._check_segment(start, target_xyz)                       # 3, 4, 5
        samples = await self._check_joints_along(start, target_xyz)         # 6
        target = [float(target_xyz[0]), float(target_xyz[1]), float(target_xyz[2]), self._r_at_connect]
        logger.info(f"MG400: {label} ({target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f}) へ MovL "
                    f"{dist:.2f} mm（サンプル {len(samples)} 点 OK, SpeedL=AccL={self.speed_ratio}）...")
        result = await self._await(self._ctl.move_l(target, self.speed_ratio, dist), "MovL")
        self._pose = list(result["pose"])
        logger.info(f"✓ MG400: {label} 到達 ({result['arrived_s']:.2f} s, "
                    f"X={self._pose[0]:.2f}, Y={self._pose[1]:.2f}, Z={self._pose[2]:.2f})")
        return result

    # ------------------------------------------------------------------
    # 高レベル操作（flow executor から呼ばれる）
    # ------------------------------------------------------------------
    async def move_xyz(self, x: float, y: float, z: float):
        self._require()
        self.workspace_validator.validate_xyz(x, y, z)
        await self._move_to([x, y, z], "XYZ移動")

    async def move_z(self, distance: float):
        self._require()
        _finite_number(distance, "distance")
        start = await self._measured_pose()
        self.workspace_validator.validate_z_relative(start[2], distance)
        await self._move_to([start[0], start[1], start[2] + distance], f"Z軸 {distance:+.2f} mm")

    async def go_home(self):
        self._require()
        home = self.home_position
        self.workspace_validator.validate_xyz(*home)
        start = await self._measured_pose()
        if math.dist(start[:3], home) < AT_HOME_MM:
            logger.info(f"MG400: すでにホーム（{math.dist(start[:3], home):.3f} mm）: 送信なし (already at home)")
            return
        await self._move_to(home, "ホーム復帰")

    # ------------------------------------------------------------------
    # 位置・ホーム
    # ------------------------------------------------------------------
    def get_current_position(self) -> List[float]:
        self._require()
        if self._pose is None:
            raise RuntimeError("MG400: 現在位置がまだ読まれていません")
        return list(self._pose[:3])

    def set_current_position_as_home(self) -> List[float]:
        """MG400 のホームは config.yaml の home_position だけ（現在位置で上書きしない）"""
        raise RuntimeError("MG400: ホームは config.yaml の home_position で指定します（現在位置では設定しません）")

    def home_is_within_limits(self) -> bool:
        if self.workspace_validator is None:
            logger.warning("警告: MG400 に可動域バリデータが無いため、ホームポジションを検証できません")
            return False
        try:
            self.workspace_validator.validate_xyz(*self.home_position)
        except ValidationError as e:
            logger.warning(f"警告: MG400 のホームポジションが可動域外です。go_home() は拒否します。\n{e}")
            return False
        return True

    # ------------------------------------------------------------------
    # 緊急停止・切断
    # ------------------------------------------------------------------
    def latch_stop(self) -> bool:
        """停止ラッチを立てるだけ（ログを出さない・I/O をしない・送出しない）。接続前なら接続前の
        停止として記録する。ラッチが立ったら True"""
        return self._force_stop_arm("emergency_stop")

    def emergency_stop(self):
        """停止ラッチを立てて即返る（I/O はワーカーが行う; 15 s の期限はワーカーの中）。
        ラッチが先、ログは後（ログのロック待ちで停止が遅れない）"""
        latched = self._force_stop_arm("emergency_stop")
        logger.warning("=== MG400 停止要求（ResetRobot, DisableRobot, 確認; 必要なら EmergencyStop） ===")
        if not latched:
            logger.warning("MG400 は接続されていません（接続前の停止として記録: 以後の initialize() は拒否）")

    @property
    def ended_abnormally(self) -> bool:
        """A stop (requested or the controller's own) during this connection, or before it"""
        ctl = self._ctl
        if ctl is not None:
            return bool(ctl.stopped or self._stop_before_connect is not None)
        last = self._last_report or {}
        return bool(last.get("stopped") or self._stop_before_connect is not None)

    def run_report(self) -> Optional[dict]:
        """セッションのデバイス報告（stop_state, disabled, 理由, 記録フォルダ）。I/O はしない。
        切断の終わりを確認できなかったときは、その旨を ``failed_reason`` に入れる"""
        ctl = self._ctl
        report = self._last_report if ctl is None else ctl.summary()
        if self._incomplete is None:
            return report
        report = dict(report or {})
        if not report.get("failed_reason"):
            report["failed_reason"] = self._incomplete
        report["cleanup_incomplete"] = self._incomplete
        return report

    _last_report: Optional[dict] = None
    #: 切断（停止シーケンス／DisableRobot・最終記録）の終わりを確認できなかった理由
    _incomplete: Optional[str] = None

    def _start_shutdown(self, ctl) -> threading.Thread:
        """切断の専用スレッドを（この接続で一度だけ）作って始める。作った時点で記録し、
        まだ始まっていなければ（開始の途中で中断された）ここで始める"""
        th = self._shutdown_thread
        if th is None:
            logger.info("=== MG400 切断開始 ===")
            box: List[dict] = []
            th = threading.Thread(target=lambda: box.append(ctl.shutdown(CLEANUP_JOIN_S)),
                                  name="mg400-shutdown", daemon=False)
            self._shutdown_box = box
            self._shutdown_thread = th
        if th.ident is None:
            th.start()
        return th

    @staticmethod
    def _shutdown_done(ctl, th: threading.Thread) -> bool:
        return not th.is_alive() and not ctl.summary().get("worker_alive")

    async def _join_shutdown(self, ctl, th: threading.Thread, timeout: float) -> None:
        """切断スレッドと、その後のワーカーの終了を ``timeout`` まで待つ（イベントループは止めない）"""
        def wait():
            end = time.monotonic() + timeout
            th.join(max(0.0, end - time.monotonic()))
            ctl.wait_closed(end - time.monotonic())
        await asyncio.get_running_loop().run_in_executor(None, wait)

    async def cleanup(self):
        """通常は DisableRobot（確認つき）、停止後はワーカーの停止シーケンスを待ち、最終の読み取り、
        切断、記録。アームを動かすコマンドは送らない。

        切断（``Controller.shutdown``）は専用スレッドで始め、ここではその終了（とワーカーの終了）
        を ``CLEANUP_JOIN_S + CLEANUP_MARGIN_S`` s まで待つ。待ちのキャンセル（GUI の Stop、2 回目の
        キャンセル）は停止ラッチにして待ち続け、終わってから再送出する。それ以外の中断
        （KeyboardInterrupt など）は停止ラッチにしてすぐ再送出する。

        「切断の終わりを確認できなかった」（``run_report()`` の ``cleanup_incomplete`` と
        ``failed_reason``）は入口で立て、終わりを確認してから外す: どこで中断されても、報告が
        終わったように見えることはない。終わりを確認できないまま戻るときは、コントローラを
        保持する（停止要求は届く; 次の ``cleanup()`` は同じ切断を待つ）"""
        ctl = self._ctl
        if ctl is None:
            return
        self._initialized = False                     # no more moves from here on
        self._incomplete = ("MG400 の切断（停止シーケンス／DisableRobot の確認・最終記録）の終わりを"
                            "確認できませんでした（切断の途中で中断）。アームの状態を DobotStudio Pro で"
                            "確かめてください")
        interrupted: Optional[BaseException] = None
        escape: Optional[BaseException] = None
        th: Optional[threading.Thread] = None
        try:
            try:
                th = self._start_shutdown(ctl)
            except BaseException as e:  # noqa: BLE001 - interrupted while the helper was being started
                self._force_stop_arm(f"切断開始の中断: {type(e).__name__}")
                interrupted = e
                th = self._start_shutdown(ctl)
            deadline = time.monotonic() + CLEANUP_JOIN_S + CLEANUP_MARGIN_S
            while not self._shutdown_done(ctl, th):
                left = deadline - time.monotonic()
                if left <= 0:
                    self._incomplete = ("MG400 の切断（停止シーケンス／DisableRobot の確認・最終記録）の"
                                        f"終わりを確認できませんでした（{CLEANUP_JOIN_S + CLEANUP_MARGIN_S:g} s "
                                        "以内に終わらず）。アームの状態を DobotStudio Pro で確かめてください")
                    break
                try:
                    await self._join_shutdown(ctl, th, left)
                except asyncio.CancelledError as e:
                    logger.error("MG400 切断待ちのキャンセル: 停止要求にして、切断の終わりを待ち続けます")
                    self._force_stop_arm("切断待ちの中断: CancelledError")
                    if interrupted is None:
                        interrupted = e
                except Exception as e:  # noqa: BLE001 - the wait itself failed: latch, report unfinished
                    logger.error(f"MG400 切断待ちのエラー: {e!r}（切断はワーカーが続けます）")
                    self._force_stop_arm(f"切断待ちのエラー: {type(e).__name__}")
                    break
        except BaseException as e:  # noqa: BLE001 - KeyboardInterrupt etc., anywhere above: latch, re-raise
            logger.error(f"MG400 切断待ちの中断: {e!r}（切断はワーカーが続けます）")
            self._force_stop_arm(f"切断待ちの中断: {type(e).__name__}")
            escape = e
        finally:
            self._finish_cleanup(ctl, th)
        if escape is not None:
            raise escape
        if interrupted is not None and not isinstance(interrupted, Exception):
            raise interrupted

    def _finish_cleanup(self, ctl, th: Optional[threading.Thread]) -> None:
        """切断の終わりを確認できたら印を外してコントローラを放す。できなければ保持する"""
        done = th is not None and self._shutdown_done(ctl, th)
        if done:
            self._incomplete = None
        report = ctl.summary()
        self._last_report = report
        stop = (report or {}).get("stop") or {}
        if stop:
            logger.warning(f"MG400 停止シーケンス: {stop.get('stop_state')} (last RobotMode {stop.get('last_mode')}). "
                           f"{stop.get('guidance', '')}")
        elif report and report.get("disabled_confirmed"):
            logger.info("✓ MG400: DisableRobot 後に RobotMode 4 を確認")
        if done:
            with self._lock:
                self._ctl = None
            self._shutdown_thread = None
            logger.info(f"=== MG400 切断完了 (record: {self.record_folder}) ===")
        else:
            logger.error(f"{self._incomplete}（ワーカーが停止シーケンス／切断を続けています; "
                         "次の cleanup() がその終了を待ちます）")

    async def __aenter__(self):
        await self.initialize()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await self.cleanup()


class MockMG400Robot(MG400Robot):
    """--mock: 実機と同じ MG400Robot を、このプロセス内の FakeMG400（loopback）に接続する。
    コードの経路は実機と同じ 1 本; 違いは接続先と、確認（Enter）を経たものとして作ること"""

    def __init__(self, workspace_validator: Optional[WorkspaceValidator] = None,
                 start_pose: Optional[Sequence[float]] = None, record_folder: Optional[str] = None,
                 **device_configs):
        from src.devices.mg400.fake import LOOPBACK, FakeMG400
        self.fake = FakeMG400(ik_model=True)
        if start_pose is not None:
            joints = self.fake.ik(*list(start_pose)[:4]) if len(list(start_pose)) >= 4 else \
                self.fake.ik(start_pose[0], start_pose[1], start_pose[2], None)
            if joints is None:
                raise ValueError(f"MG400 の mock_start_pose {list(start_pose)} は fake の関節モデルで解けません")
            self.fake.angles[:4] = joints
            self.fake.pose = self.fake.fk(joints)
        fake = self.fake
        ports = {control.DASHBOARD_PORT: fake.dashboard_port, control.FEEDBACK_PORT: fake.feedback_port,
                 control.MOTION_PORT: fake.motion_port}

        def connect(host, port, timeout):
            return control._connect_fixed(LOOPBACK, ports[port], timeout)

        def factory(address, folder):
            return control.Controller(LOOPBACK, folder, connect=connect)

        super().__init__(LOOPBACK, workspace_validator=workspace_validator, controller_factory=factory,
                         record_folder=record_folder, confirmed=True, **device_configs)
        self.mock = True

    async def cleanup(self):
        try:
            await super().cleanup()
        finally:
            if self._ctl is None:
                # the shutdown finished; while it still runs (an interrupted wait) the fake
                # stays up, as the arm would, for the stop sequence to reach it
                self.fake.close()
