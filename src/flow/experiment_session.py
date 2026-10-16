"""実験セッションのライフサイクル管理

「ロボット初期化 → 実行 → 中断時は緊急停止 / エラー時はホーム復帰 → 後始末」
の骨格を一元化する。安全系（緊急停止・クリーンアップ）の実装が複数の
ランナーに分かれて将来片方だけ修正される事故を防ぐのが目的。

実機モードと Mock モードで同じ安全枠を使うため、ロボット／共有デバイスの
生成はファクトリ経由で差し替えられる（``mock=True`` で Mock 実装に切り替え）。

使用例:
    # プリフライトが作ったロボットごとの可動域（MaxArm は robots.<id>.workspace
    # だけから作る）を実行時にもそのまま使う
    report = require_preflight(steps, validator, robot_ports=robot_ports)
    session = ExperimentSession(mock=args.mock, robot_ports=robot_ports,
                                workspace_validator=validator,
                                robot_validators=report.validators_by_robot)

    async def body():
        robot = await session.add_robot(1, use_picus2=True)
        arm = await session.add_robot(4)          # type: maxarm（専用の可動域）
        shared = await session.add_shared(use_scale=True)
        ...実験手順...
        return results

    try:
        results = await session.run(body)
    finally:
        exp_logger.finalize(session.status, session.error)
"""
import asyncio
import logging
import os
import threading
from typing import Awaitable, Callable, Dict, Optional

from src import config as lab_config
from src.devices.safety.lab_robot import LabRobot
from src.devices.safety.mock_robot import MockLabRobot, MockSharedDevices, parse_start_pose
from src.devices.safety.shared_devices import SharedDevices
from src.devices.safety.validators import ValidationError, default_workspace_validator


class SafetyAbort(RuntimeError):
    """A run must stop immediately, without any recovery motion.

    Raised by safety gates (e.g. the GUI's sensor interlock in src/gui/runner.py)
    when continuing is unsafe. ExperimentSession.run treats it like a validation
    error: hardware is stopped in place (emergency stop) and go_home_all() is NOT
    attempted, because a "go home" move may itself be the unsafe action.
    """

logger = logging.getLogger(__name__)

#: ロボット設定（config.yaml の robots.<id>）のうち、ドライバのオプションではない
#: キー。それ以外（move_ms, pose_settle_tolerance など）は MaxArm の
#: device_configs としてそのまま渡す。
_ROBOT_CONFIG_KEYS = {"type", "dobot_port", "picus2_address", "maxarm_port", "workspace"}

#: エラー時の自動ホーム復帰（go_home_all）をしないロボット種別。MaxArm は
#: 検証済みの安全な復帰経路を持たないため、エラー後に自動では動かさない。
NO_RECOVERY_HOME_TYPES = frozenset({"maxarm"})


def robot_type(ports: Optional[dict]) -> str:
    """ロボット設定の種別（``type`` が無ければ ``"dobot"``）"""
    return (ports or {}).get("type") or lab_config.DEFAULT_ROBOT_TYPE


def robot_options(ports: Optional[dict]) -> dict:
    """ロボット設定からポート・種別・可動域を除いたドライバオプション"""
    return {k: v for k, v in (ports or {}).items() if k not in _ROBOT_CONFIG_KEYS}


def _flatten_limits(limits: dict) -> dict:
    """``WorkspaceValidator.get_limits()`` の {"x": {"min", "max"}} を x_min 形式にする"""
    flat = {}
    for axis, bounds in (limits or {}).items():
        if isinstance(bounds, dict):
            for key in ("min", "max"):
                if bounds.get(key) is not None:
                    flat[f"{axis}_{key}"] = bounds[key]
    return flat


def validator_for_robot(robot_id, shared_validator=None, robot_ports: Optional[dict] = None):
    """ロボットごとの可動域バリデータを返す

    - MaxArm（``type: maxarm``）: 座標系が Dobot と異なるため、そのロボットの
      ``robots.<id>.workspace`` **だけ**から作る（共通の可動域にはマージしない）。
      x/y/z の min/max がすべて必要で、r_min / r_max は任意。欠けている・null の
      値がある場合は ``lab_config.ConfigError``（robot_id を含む）。共通の
      （Dobot の）バリデータに黙って戻ることはない。
    - Dobot で ``robots.<id>.workspace`` を持つもの: 共通の可動域にそのロボットの
      値を上書きしたバリデータ（``lab_config.get_robot_workspace`` と同じマージ）。
    - それ以外（Dobot 1〜3 の通常の設定）: ``shared_validator`` をそのまま返す。

    Args:
        robot_id: ロボット ID
        shared_validator: 共通の可動域バリデータ。省略時は config.yaml の workspace。
            渡した場合は Dobot のマージの土台にする（呼び出し側が狭めた値を保つ）
        robot_ports: robot_id → ロボット設定。省略時は config.yaml

    Raises:
        lab_config.ConfigError: 可動域の設定が使えない場合（デバイスを開く前に送出）
    """
    if robot_ports is None:
        robot_ports = lab_config.get_robot_ports()
    ports = robot_ports.get(robot_id) or robot_ports.get(str(robot_id)) or {}
    # 設定 dict のマージと未知キーの警告を行う純関数（safety 層）。遅延 import
    from src.devices.safety.validators import (
        maxarm_workspace_validator_from_config,
        workspace_validator_from_config,
    )
    if robot_type(ports) == "maxarm":
        own = lab_config.maxarm_workspace(robot_id, ports)
        try:
            # 箱・半径＋実機の逆運動学（届かない目標は送信前に拒否）
            return maxarm_workspace_validator_from_config(own)
        except ValueError as e:
            raise lab_config.ConfigError(
                f"robots.{robot_id}.workspace（MaxArm）が不正です: {e}") from e
    override = ports.get("workspace")
    if not override:
        return shared_validator if shared_validator is not None else default_workspace_validator()
    if not isinstance(override, dict):
        raise lab_config.ConfigError(
            f"robots.{robot_id}.workspace must be a mapping, got {type(override).__name__}"
        )
    if shared_validator is not None:
        base = _flatten_limits(shared_validator.get_limits())
    else:
        base = lab_config.get_workspace()
    try:
        return workspace_validator_from_config(base, dict(override))
    except ValueError as e:
        raise lab_config.ConfigError(f"robots.{robot_id}.workspace が不正です: {e}") from e


def default_robot_factory(robot_id, *, use_picus2, ports, workspace_validator):
    """実機ロボットを生成する既定ファクトリ（種別で LabRobot / MaxArmRobot を切り替え）"""
    if robot_type(ports) == "maxarm":
        from src.devices.safety.maxarm_robot import MaxArmRobot  # 遅延 import
        return MaxArmRobot(
            maxarm_port=ports["maxarm_port"],
            workspace_validator=workspace_validator,
            **robot_options(ports),
        )
    return LabRobot(
        use_dobot=True,
        use_picus2=use_picus2,
        dobot_port=ports["dobot_port"],
        picus2_address=ports.get("picus2_address", ""),
        picus2_connection_type="usb",
        workspace_validator=workspace_validator,
    )


def mock_start_pose(robot_id=None, shared_config: Optional[dict] = None):
    """Mock の開始姿勢 (x, y, z, r) を返す（未設定なら None = 既定の姿勢）

    実機はアームの「現在位置」をホームとし、相対移動をそこから検証する。
    Mock を同じ姿勢から始めるための設定で、優先順位は
    環境変数 ``MOCK_START_POSE`` → config.yaml の ``shared_devices.mock_start_pose``。
    値は "x,y,z,r" 文字列または 4 要素のリスト。config.yaml では
    robot_id をキーにした辞書でロボットごとに指定することもできる。

    Raises:
        ValueError: 値を解釈できない場合（黙って既定の姿勢に戻さない）
    """
    env = os.environ.get("MOCK_START_POSE", "").strip()
    if env:
        return parse_start_pose(env)
    if shared_config is None:
        try:
            shared_config = lab_config.get_shared_devices()
        except Exception as e:  # noqa: BLE001 - 設定が読めなければ既定の姿勢
            logger.warning(f"mock_start_pose の設定を読めませんでした: {e}")
            return None
    value = (shared_config or {}).get("mock_start_pose")
    if isinstance(value, dict):
        value = value.get(robot_id, value.get(str(robot_id)))
    return parse_start_pose(value)


def maxarm_mock_start_pose(robot_id, shared_config: Optional[dict] = None):
    """MaxArm の Mock 開始姿勢 (x, y, z)。未設定なら None（= MaxArm の ORIGIN）

    座標系が Dobot と異なるため、環境変数 ``MOCK_START_POSE``（Dobot 用の
    単一値）は使わない。config.yaml の ``shared_devices.mock_start_pose`` が
    robot_id をキーにした辞書で、このロボットの値があるときだけ使う。
    値は "x,y,z" / "x,y,z,r" 文字列または 3〜4 要素のリスト（r は無視）。

    Raises:
        ValueError: 値を解釈できない場合
    """
    if os.environ.get("MOCK_START_POSE", "").strip():
        logger.info(f"[MOCK] Robot {robot_id} は MaxArm のため MOCK_START_POSE を使いません"
                    "（shared_devices.mock_start_pose の robot_id 別の値だけを使います）")
    if shared_config is None:
        try:
            shared_config = lab_config.get_shared_devices()
        except Exception as e:  # noqa: BLE001 - 設定が読めなければ既定の姿勢
            logger.warning(f"mock_start_pose の設定を読めませんでした: {e}")
            return None
    value = (shared_config or {}).get("mock_start_pose")
    if not isinstance(value, dict):
        return None
    from src.devices.safety.mock_maxarm import parse_maxarm_start_pose  # 遅延 import
    return parse_maxarm_start_pose(value.get(robot_id, value.get(str(robot_id))))


def mock_robot_factory(robot_id, *, use_picus2, ports, workspace_validator):
    """MockLabRobot を生成するファクトリ（実機なしでの動作確認用）

    可動域バリデータは実機と同じものを渡す。Mock でも可動域違反が
    実行前に検出されるため、フロー JSON の安全確認に使える。
    開始姿勢は mock_start_pose()（MOCK_START_POSE / config.yaml）から取る。
    """
    if robot_type(ports) == "maxarm":
        from src.devices.safety.mock_maxarm import MockMaxArmRobot  # 遅延 import
        start_pose = maxarm_mock_start_pose(robot_id)
        if start_pose is not None:
            logger.info(f"[MOCK] Robot {robot_id}（MaxArm）の開始姿勢: {start_pose}")
        return MockMaxArmRobot(
            workspace_validator=workspace_validator,
            start_pose=start_pose,
            **robot_options(ports),
        )
    start_pose = mock_start_pose(robot_id)
    if start_pose is not None:
        logger.info(f"[MOCK] Robot {robot_id} の開始姿勢: {start_pose}")
    return MockLabRobot(
        use_dobot=True,
        use_picus2=use_picus2,
        dobot_port=ports.get("dobot_port", ""),
        picus2_address=ports.get("picus2_address", ""),
        workspace_validator=workspace_validator,
        start_pose=start_pose,
    )


def default_shared_factory(*, use_scale, use_camera, config, use_microscope=False,
                           use_microscope_serial=False, use_dmm=False, use_lockin=False):
    """実機 SharedDevices を生成する既定ファクトリ"""
    return SharedDevices(
        use_scale=use_scale,
        use_camera=use_camera,
        use_microscope=use_microscope,
        use_microscope_serial=use_microscope_serial,
        use_dmm=use_dmm,
        use_lockin=use_lockin,
        scale_port=config["scale_port"],
        camera_index=config["camera_index"],
        microscope_index=config.get("microscope_index", 2),
        microscope_port=config.get("microscope_port", ""),
        microscope_resolution=config.get("microscope_resolution", ""),
        dmm_resource=config.get("dmm_resource", ""),
        dmm_timeout_ms=config.get("dmm_timeout_ms", 10000),
        wait_after_dmm=config.get("wait_after_dmm", 0.0),
        lockin_resource=config.get("lockin_resource", ""),
        lockin_timeout_ms=config.get("lockin_timeout_ms", 5000),
        wait_after_lockin=config.get("wait_after_lockin", 0.0),
    )


def mock_shared_factory(*, use_scale, use_camera, config, use_microscope=False,
                        use_microscope_serial=False, use_dmm=False, use_lockin=False):
    """MockSharedDevices を生成するファクトリ"""
    return MockSharedDevices(use_scale=use_scale, use_camera=use_camera,
                             use_microscope=use_microscope,
                             use_microscope_serial=use_microscope_serial,
                             use_dmm=use_dmm, use_lockin=use_lockin)


class ExperimentSession:
    """ロボット群・共有デバイスの初期化から後始末までを管理するセッション

    Args:
        mock: True なら Mock 実装（MockLabRobot / MockSharedDevices）を使う
        robot_factory: ロボット生成関数の差し替え（テスト・GUI 用）
        shared_factory: 共有デバイス生成関数の差し替え
        robot_ports: robot_id → ポート設定。省略時は config.yaml
        shared_config: 共有デバイス設定。省略時は config.yaml
        workspace_validator: 可動域バリデータ。省略時は config.yaml の workspace
        robot_validators: robot_id → そのロボット専用の可動域バリデータ。
            含まれないロボットは :func:`validator_for_robot` で作る（MaxArm は
            robots.<id>.workspace だけから、他は ``workspace_validator``）
            （run_flow.run_preflight が返す ``report.validators_by_robot`` を渡す）

    Attributes:
        robots: robot_id → 初期化済みロボット（初期化に失敗したものは入らない）
        shared: 初期化済み共有デバイス（add_shared 未呼び出しなら None）
        status: 実行結果 ("completed" / "aborted" / "failed")。run() が設定する
        error: エラーメッセージ（completed 時は None）
    """

    def __init__(
        self,
        *,
        mock: bool = False,
        robot_factory: Optional[Callable] = None,
        shared_factory: Optional[Callable] = None,
        robot_ports: Optional[dict] = None,
        shared_config: Optional[dict] = None,
        workspace_validator=None,
        robot_validators: Optional[dict] = None,
    ):
        self.mock = mock
        self.robot_factory = robot_factory or (
            mock_robot_factory if mock else default_robot_factory
        )
        self.shared_factory = shared_factory or (
            mock_shared_factory if mock else default_shared_factory
        )
        self._robot_ports = robot_ports
        self._shared_config = shared_config
        self.workspace_validator = workspace_validator or default_workspace_validator()
        self.robot_validators: Dict[int, object] = dict(robot_validators or {})
        self._robot_types: Dict[int, str] = {}

        self.robots: Dict[int, object] = {}
        self.shared = None
        self.status: str = "failed"
        self.error: Optional[str] = "実行が中断されました"

        # 緊急停止済みのロボット（id）。GUI の Stop は UI スレッドから
        # emergency_stop_all() を直接呼ぶため、ロックで保護する。
        self._estop_lock = threading.Lock()
        self._estopped_ids = set()

    # ------------------------------------------------------------------
    # 設定
    # ------------------------------------------------------------------
    def robot_ports(self) -> dict:
        if self._robot_ports is None:
            self._robot_ports = lab_config.get_robot_ports()
        return self._robot_ports

    @property
    def robot_types(self) -> Dict[int, str]:
        """登録済みロボットの robot_id → 種別（"dobot" / "maxarm"）"""
        return dict(self._robot_types)

    def validator_for(self, robot_id: int):
        """robot_id に使う可動域バリデータ

        ``robot_validators`` に無ければ :func:`validator_for_robot` で作って記録する。
        MaxArm は共通の（Dobot の）バリデータには決して戻らず、専用の可動域が
        無ければ ``lab_config.ConfigError`` になる。

        Raises:
            lab_config.ConfigError: 可動域の設定が使えない場合
        """
        if robot_id not in self.robot_validators:
            self.robot_validators[robot_id] = validator_for_robot(
                robot_id, self.workspace_validator, self.robot_ports())
        return self.robot_validators[robot_id]

    def shared_config(self) -> dict:
        if self._shared_config is None:
            self._shared_config = lab_config.get_shared_devices()
        return self._shared_config

    # ------------------------------------------------------------------
    # デバイス登録
    # ------------------------------------------------------------------
    async def add_robot(self, robot_id: int, *, use_picus2: bool = False):
        """設定のポートでロボットを生成・初期化して登録する

        ホーム: 現在位置をホームとして設定する。ただし MaxArm で config.yaml に
        ``home_position`` があるときはそれを保ち、可動域内か検査する（外なら
        RuntimeError）。Dobot の挙動は従来どおり（常に現在位置）。

        失敗時は例外を送出し、ロボットは登録されない。初期化中の中断、初期化が
        False を返した場合、ホーム設定の失敗（中断・キャンセルを含む）では
        ロボットを切断してから送出する（initialize() が例外を送出した場合の
        切断は initialize() 自身が行う）。

        Raises:
            lab_config.ConfigError: 可動域の設定が使えない（デバイスは開かない）
            RuntimeError: ポート未設定、初期化失敗、ホームが可動域外など
        """
        ports = self.robot_ports().get(robot_id)
        rtype = robot_type(ports)
        if rtype == "maxarm":
            if not self.mock and not (ports or {}).get("maxarm_port"):
                raise RuntimeError(
                    f"Robot {robot_id}（MaxArm）の接続ポート maxarm_port が config.yaml の "
                    f"robots セクションに設定されていません（--robot{robot_id}-maxarm / "
                    f"ROBOT{robot_id}_MAXARM_PORT でも指定できます）"
                )
            if use_picus2:
                logger.warning(f"Robot {robot_id} は MaxArm のためピペット（Picus2）は使えません。"
                               "use_picus2 を無視します")
                use_picus2 = False
        elif not self.mock and (not ports or not ports.get("dobot_port")):
            raise RuntimeError(
                f"Robot {robot_id} の接続ポートが config.yaml の robots セクションに"
                f"定義されていません（--robot{robot_id}-dobot / "
                f"ROBOT{robot_id}_DOBOT_PORT でも指定できます）"
            )
        # 可動域の設定エラー（ConfigError）はデバイスを開く前に出す
        validator = self.validator_for(robot_id)
        robot = self.robot_factory(
            robot_id,
            use_picus2=use_picus2,
            ports=ports or {},
            workspace_validator=validator,
        )
        # initialize() は失敗時に例外を送出する（戻り値の握りつぶし防止）
        try:
            ok = await robot.initialize()
        except BaseException as e:
            # 初期化中の中断（Stop / Ctrl+C）: 未登録なので session.cleanup() では
            # 切断されない。ここで切断してから再送出する。
            if not isinstance(e, Exception):
                await self._cleanup_one(f"Robot {robot_id}", robot)
            raise
        if not ok:
            await self._cleanup_one(f"Robot {robot_id}", robot)
            raise RuntimeError(f"Robot {robot_id} の初期化に失敗しました")
        try:
            self._set_home(robot_id, robot, rtype, ports or {})
        except BaseException:
            # 未登録なので session.cleanup() では切断されない（接続・ハートビートが
            # 残る）。ここで切断してから再送出する（中断・キャンセルも同じ）
            await self._cleanup_one(f"Robot {robot_id}", robot)
            raise
        self.robots[robot_id] = robot
        self._robot_types[robot_id] = rtype
        if rtype == "maxarm":
            logger.info(
                f"Robot {robot_id} 初期化完了 "
                f"(MaxArm={(ports or {}).get('maxarm_port') or '-'})"
            )
        else:
            logger.info(
                f"Robot {robot_id} 初期化完了 "
                f"(dobot={(ports or {}).get('dobot_port') or '-'}, "
                f"picus2={'有' if use_picus2 else '無'})"
            )
        return robot

    @staticmethod
    def _set_home(robot_id, robot, rtype: str, ports: dict) -> None:
        """ホームを設定する（add_robot から。失敗時は例外）

        MaxArm で ``home_position`` が設定されていればそれを保ち（現在位置で
        上書きしない）、``home_is_within_limits()`` で検査する。それ以外は
        現在位置をホームにする。
        """
        if rtype == "maxarm" and ports.get("home_position") is not None:
            if getattr(robot, "home_position", None) is None:
                raise RuntimeError(
                    f"Robot {robot_id}（MaxArm）: 設定の home_position がロボットに渡っていません")
            if not robot.home_is_within_limits():
                raise RuntimeError(
                    f"Robot {robot_id}（MaxArm）: 設定の home_position {list(robot.home_position)} が"
                    f"可動域（robots.{robot_id}.workspace）の外です。home_position か可動域を"
                    "確認してください")
            logger.info(f"Robot {robot_id}（MaxArm）: 設定の home_position "
                        f"{list(robot.home_position)} をホームにします")
            return
        robot.set_current_position_as_home()

    async def add_shared(self, *, use_scale: bool = False, use_camera: bool = False,
                         use_microscope: bool = False, use_microscope_serial: bool = False,
                         use_dmm: bool = False, use_lockin: bool = False):
        """設定の値で共有デバイスを生成・初期化して登録する

        初期化が途中で失敗した場合（天秤は開いたが DMM が開けない等）は、開いた分を
        切断してから送出する。``self.shared`` に登録されるのは成功時だけなので、
        ここで閉じないと cleanup() の対象から漏れる。

        初期化中の中断（Ctrl+C / キャンセル）では切断を待たない: DMM の初期化
        ワーカーが GPIB の応答を待っている間などに切断を待つと、run() の緊急停止が
        遅れる。登録だけして再送出し、run() が緊急停止した後の cleanup() で閉じる。
        切断中の中断も通常のエラー（= ホーム復帰）にはせず、中断として再送出する。
        """
        kwargs = dict(use_scale=use_scale, use_camera=use_camera, use_microscope=use_microscope,
                      use_microscope_serial=use_microscope_serial)
        if use_dmm:
            # use_dmm を知らない差し替えファクトリ（テスト・GUI）も動くよう、要るときだけ渡す
            kwargs["use_dmm"] = True
        if use_lockin:
            kwargs["use_lockin"] = True
        shared = self.shared_factory(config=self.shared_config(), **kwargs)
        try:
            ok = await shared.initialize()
        except Exception:
            await self._discard_shared("共有デバイス（初期化エラー）", shared)
            raise
        except BaseException:
            self.shared = shared        # 緊急停止の後で cleanup() が閉じる
            raise
        if not ok:
            await self._discard_shared("共有デバイス（初期化失敗）", shared)
            raise RuntimeError("共有デバイスの初期化に失敗しました")
        self.shared = shared
        logger.info("共有デバイス初期化完了")
        return shared

    # ------------------------------------------------------------------
    # 安全枠
    # ------------------------------------------------------------------
    def emergency_stop_all(self, *, skip_already_stopped: bool = False):
        """登録済み全ロボットを緊急停止する（例外を送出しない）

        同期関数で await を含まないため、イベントループがデバイス呼び出しで
        ブロックされていても別スレッド（GUI の Stop）から直接呼び出せる。

        Args:
            skip_already_stopped: True なら、すでに緊急停止済みのロボットは
                再送しない（GUI の Stop で停止済みのロボットに、キャンセル処理で
                二重に停止コマンドを送らないため）。
        """
        # 実行スレッドが add_robot で辞書を更新中でも安全にスナップショットを取る
        for rid, robot in list(self.robots.items()):
            with self._estop_lock:
                if skip_already_stopped and rid in self._estopped_ids:
                    continue
            try:
                robot.emergency_stop()
            except Exception as e:  # noqa: BLE001 - 1台の失敗で他を止め損ねない
                logger.error(f"Robot {rid} の緊急停止に失敗しました: {e}")
                continue  # 停止済みとして記録しない（次の呼び出しで再送する）
            with self._estop_lock:
                self._estopped_ids.add(rid)

    async def go_home_all(self):
        """登録済み全ロボットをベストエフォートでホームに戻す

        中断（CancelledError / KeyboardInterrupt）は捕捉せずに伝播させる。
        呼び出し側（run）が緊急停止してから後始末する。
        緊急停止済みのロボット（GUI の Stop など）は動かさない。
        """
        for rid, robot in list(self.robots.items()):
            with self._estop_lock:
                stopped = rid in self._estopped_ids
            if stopped:
                logger.warning(f"Robot {rid} は緊急停止済みのためホーム復帰しません")
                continue
            rtype = self._robot_types.get(rid) or getattr(robot, "ROBOT_TYPE", None)
            if rtype in NO_RECOVERY_HOME_TYPES:
                # 検証済みの安全な復帰経路が無い: エラー後に自動では動かさない
                logger.warning(f"Robot {rid}（{rtype}）はエラー後の自動ホーム復帰を行いません"
                               "（その場で停止しています）")
                continue
            try:
                logger.info(f"Robot {rid} をホームに戻しています...")
                await robot.go_home()
            except Exception as e:
                logger.error(f"Robot {rid} のホーム復帰に失敗しました: {e}")

    async def _discard_shared(self, name, shared):
        """初期化に失敗した共有デバイスを切断する。切断中の中断は中断として送出する"""
        err = await self._cleanup_one(name, shared)
        if err is not None and not isinstance(err, Exception):
            # Ctrl+C / キャンセル: 呼び出し側の RuntimeError（= ホーム復帰）に化けさせない。
            # 切断の残りは run() の cleanup() に任せる
            self.shared = shared
            raise err

    @staticmethod
    async def _cleanup_one(name, device):
        """1台を切断する。中断を含むすべての例外を捕捉して返す（送出しない）"""
        try:
            await device.cleanup()
        except BaseException as e:  # noqa: BLE001 - 後始末は最後まで続ける
            logger.error(f"{name} の切断中にエラー／中断: {e!r}")
            return e
        return None

    async def cleanup(self):
        """全ロボット・共有デバイスをベストエフォートで切断する

        1台の失敗や切断中の中断（2回目のキャンセル、Ctrl+C）で残りの
        デバイスの切断が飛ばされないよう、デバイスごとに BaseException を
        捕捉して続行する。中断はすべての切断を試みた後に再送出する。
        """
        interrupted = None
        devices = [(f"Robot {rid}", r) for rid, r in list(self.robots.items())]
        if self.shared is not None:
            devices.append(("共有デバイス", self.shared))
        for name, device in devices:
            err = await self._cleanup_one(name, device)
            if err is not None and not isinstance(err, Exception) and interrupted is None:
                interrupted = err
        if interrupted is not None:
            raise interrupted

    async def run(self, body: Callable[[], Awaitable]):
        """実験本体を安全枠の中で実行する

        - 正常終了: status="completed" とし body の戻り値を返す
        - Ctrl+C / キャンセル: 全ロボットを緊急停止して再送出
          （ホーム復帰の追加動作はさせず、その場で止める）
        - 可動域違反（ValidationError）: アームは動いていないので
          ホーム復帰はせず、そのまま再送出
        - 安全ゲートによる停止（SafetyAbort）: 全ロボットを緊急停止し、
          ホーム復帰はせずに status="aborted" で再送出
        - その他の例外: 全ロボットをホームに戻して再送出（MaxArm など
          NO_RECOVERY_HOME_TYPES の種別は自動では動かさない）。ホーム復帰中に
          中断（Ctrl+C / キャンセル）された場合は緊急停止してから再送出
        - いずれの場合もクリーンアップ（切断）は必ず実行する

        exp_logger の確定は行わない。呼び出し側が finally で
        ``exp_logger.finalize(session.status, session.error)`` を呼ぶこと。
        """
        try:
            result = await body()
            self.status, self.error = "completed", None
            return result
        except (KeyboardInterrupt, asyncio.CancelledError):
            logger.warning("中断要求を受信しました。緊急停止します...")
            self.status, self.error = "aborted", "ユーザーによる中断（Ctrl+C）"
            # GUI の Stop で UI スレッドから停止済みのロボットには再送しない
            self.emergency_stop_all(skip_already_stopped=True)
            raise
        except Exception as e:
            logger.error(f"実行エラー: {e}")
            self.status, self.error = "failed", str(e)
            if isinstance(e, SafetyAbort):
                # 続行が危険: その場で止める。ホーム復帰の動作自体が危険な場合がある
                logger.error("安全ゲートにより中止しました。緊急停止します（ホーム復帰は行いません）")
                self.status = "aborted"
                self.emergency_stop_all()
                raise
            if isinstance(e, ValidationError):
                # 移動は検証で拒否され、コマンドは送信されていない。
                # 未検証の復帰動作をさせる理由がないのでホーム復帰しない
                logger.error("可動域違反のため移動は実行されていません。ホーム復帰は行いません")
                raise
            try:
                await self.go_home_all()
            except BaseException as recovery_interrupt:
                # ホーム復帰中の Ctrl+C / キャンセル: 復帰動作をその場で止める
                logger.warning("ホーム復帰中に中断されました。緊急停止します...")
                self.status = "aborted"
                self.error = f"{e}（ホーム復帰中に中断: 緊急停止済み）"
                self.emergency_stop_all()
                raise recovery_interrupt from e
            raise
        finally:
            await self.cleanup()
