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
_ROBOT_CONFIG_KEYS = {"type", "dobot_port", "picus2_address", "maxarm_port", "mg400_address", "workspace"}

#: エラー時の自動ホーム復帰（go_home_all）をしないロボット種別。MaxArm は
#: 検証済みの安全な復帰経路を持たないため、エラー後に自動では動かさない。
#: MG400 も同じ（エラー後はその場で DisableRobot; flow plan F8）。
NO_RECOVERY_HOME_TYPES = frozenset({"maxarm", "mg400"})

#: 停止リストに initialize() の前から載せる種別（flow plan F6）。MG400 の停止は
#: ラッチ（request_stop）で、接続前に立てても initialize() が最初の段階で見る。
#: Dobot / MaxArm の登録は従来どおり initialize() の後。
PRE_REGISTER_TYPES = frozenset({"mg400"})


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
    - MG400（``type: mg400``）: 同じく自分の ``workspace`` だけから作る（箱と半径。
      到達性はコントローラの InverseSolution で実行時に検査する）。必須キーが
      欠けていれば ``lab_config.ConfigError``。
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
    if robot_type(ports) == "mg400":
        own = lab_config.mg400_config(robot_id, ports)["workspace"]
        try:
            return workspace_validator_from_config(None, own)
        except ValueError as e:
            raise lab_config.ConfigError(
                f"robots.{robot_id}.workspace（MG400）が不正です: {e}") from e
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


def default_robot_factory(robot_id, *, use_picus2, ports, workspace_validator, **extra):
    """実機ロボットを生成する既定ファクトリ（種別で LabRobot / MaxArmRobot / MG400Robot を切り替え）

    ``extra`` は MG400 にだけ渡る（``record_folder``, ``confirmed``; add_robot 参照）。
    """
    if robot_type(ports) == "mg400":
        from src.devices.safety.mg400_robot import MG400Robot  # 遅延 import
        return MG400Robot(
            mg400_address=ports["mg400_address"],
            workspace_validator=workspace_validator,
            record_folder=extra.get("record_folder"),
            confirmed=bool(extra.get("confirmed", False)),
            **robot_options(ports),
        )
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


def mg400_mock_start_pose(robot_id, shared_config: Optional[dict] = None):
    """MG400 の Mock 開始姿勢 [x, y, z(, r)]。未設定なら None（= fake の既定姿勢）。
    MaxArm と同じく、Dobot 用の単一値 MOCK_START_POSE は使わない"""
    if shared_config is None:
        try:
            shared_config = lab_config.get_shared_devices()
        except Exception as e:  # noqa: BLE001 - 設定が読めなければ既定の姿勢
            logger.warning(f"mock_start_pose の設定を読めませんでした: {e}")
            return None
    value = (shared_config or {}).get("mock_start_pose")
    if not isinstance(value, dict):
        return None
    pose = value.get(robot_id, value.get(str(robot_id)))
    if pose is None:
        return None
    from src.devices.safety.mock_maxarm import parse_maxarm_start_pose  # 同じ書式（x,y,z[,r]）
    if isinstance(pose, str):
        parts = [float(p) for p in pose.replace(";", ",").split(",")]
    else:
        parts = [float(p) for p in list(pose)]
    parse_maxarm_start_pose(parts)          # 3〜4 個の有限な数値であることの検査
    return parts


def mg400_mock_kinematics(robot_id, shared_config: Optional[dict] = None) -> str:
    """MG400 の Mock の関節モデル（``shared_devices.mock_kinematics`` の robot_id の値）。
    キーが無ければ "standin"。null・辞書でない値・文字列でない名前・未知の名前は
    ConfigError（書かれた値を黙って既定に戻さない）。Mock だけが読む設定で、実機の経路には渡らない"""
    from src.devices.mg400.fake import KINEMATICS
    if shared_config is None:
        shared_config = lab_config.get_shared_devices()
    shared_config = shared_config or {}
    if "mock_kinematics" not in shared_config:
        return "standin"
    value = shared_config["mock_kinematics"]
    if not isinstance(value, dict):
        raise lab_config.ConfigError(
            f"shared_devices.mock_kinematics はロボット ID をキーにした辞書で書いてください（例 {{5: fitted_planar_v1}}）: {value!r}")
    if robot_id in value:
        name = value[robot_id]
    elif str(robot_id) in value:
        name = value[str(robot_id)]
    else:
        return "standin"
    if not isinstance(name, str) or name not in KINEMATICS:
        raise lab_config.ConfigError(
            f"shared_devices.mock_kinematics.{robot_id} = {name!r} は使えません（{', '.join(KINEMATICS)} のどれか）")
    return name


def mock_robot_factory(robot_id, *, use_picus2, ports, workspace_validator, **extra):
    """MockLabRobot を生成するファクトリ（実機なしでの動作確認用）

    可動域バリデータは実機と同じものを渡す。Mock でも可動域違反が
    実行前に検出されるため、フロー JSON の安全確認に使える。
    開始姿勢は mock_start_pose()（MOCK_START_POSE / config.yaml）から取る。
    MG400 の mock は実機と同じ MG400Robot をプロセス内の fake に接続する
    （``extra`` の ``record_folder`` に記録を書く）。
    """
    if robot_type(ports) == "mg400":
        from src.devices.safety.mg400_robot import MockMG400Robot  # 遅延 import
        start_pose = mg400_mock_start_pose(robot_id)
        kinematics = mg400_mock_kinematics(robot_id)
        if start_pose is not None:
            logger.info(f"[MOCK] Robot {robot_id}（MG400）の開始姿勢: {start_pose}")
        logger.info(f"[MOCK] Robot {robot_id}（MG400）の fake の関節モデル: {kinematics}")
        return MockMG400Robot(
            workspace_validator=workspace_validator,
            start_pose=start_pose,
            kinematics=kinematics,
            record_folder=extra.get("record_folder"),
            **robot_options(ports),
        )
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


#: 内部発振器の上限（config.yaml の shared_devices）。発振器を書くランのデバイスへ渡す
OSC_LIMIT_KEYS = ("lockin_osc_max_vrms", "lockin_osc_min_hz", "lockin_osc_max_hz")


def _osc_kwargs(config: dict, lockin_osc_plan) -> dict:
    if lockin_osc_plan is None:
        return {}
    return {"lockin_osc_plan": lockin_osc_plan,
            **{k: config[k] for k in OSC_LIMIT_KEYS if k in config}}


def default_shared_factory(*, use_scale, use_camera, config, use_microscope=False,
                           use_microscope_serial=False, use_dmm=False, use_lockin=False,
                           use_scope=False, lockin_osc_plan=None):
    """実機 SharedDevices を生成する既定ファクトリ"""
    return SharedDevices(
        use_scale=use_scale,
        use_camera=use_camera,
        use_microscope=use_microscope,
        use_microscope_serial=use_microscope_serial,
        use_dmm=use_dmm,
        use_lockin=use_lockin,
        use_scope=use_scope,
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
        scope_resource=config.get("scope_resource", ""),
        scope_timeout_ms=config.get("scope_timeout_ms", 5000),
        wait_after_scope=config.get("wait_after_scope", 0.0),
        **_osc_kwargs(config, lockin_osc_plan),
    )


def mock_shared_factory(*, use_scale, use_camera, config, use_microscope=False,
                        use_microscope_serial=False, use_dmm=False, use_lockin=False,
                        use_scope=False, lockin_osc_plan=None):
    """MockSharedDevices を生成するファクトリ"""
    return MockSharedDevices(use_scale=use_scale, use_camera=use_camera,
                             use_microscope=use_microscope,
                             use_microscope_serial=use_microscope_serial,
                             use_dmm=use_dmm, use_lockin=use_lockin, use_scope=use_scope,
                             **_osc_kwargs(config, lockin_osc_plan))


class OutputRestoreError(RuntimeError):
    """本体は終わったが、機器の出力（LI5640 の内部発振器）を戻せなかった・確かめられなかった"""


class RobotStopError(RuntimeError):
    """本体は終わったが、ロボット（MG400）が自分で停止した、または停止シーケンスが停止を
    確認できなかった（デバイス報告の failed_reason）"""


#: 復元できなかったときにエラー文へ足す注意
OUTPUT_CHECK_NOTE = "LI5640 の OSC OUT（発振器の周波数・振幅）を前面パネルで確認してください"


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
        record_root: Optional[str] = None,
    ):
        self.mock = mock
        #: ロボットの記録（MG400 の report.json など）を置く実行フォルダ（run_flow / GUI が渡す）
        self.record_root = record_root
        #: 人の確認（Enter / 確認ダイアログ）を経たロボット（confirm_robot）。MG400 の実機ランに要る
        self._confirmed_robots = set()
        #: initialize() 中の MG400（停止リストに先に載せる; flow plan F6）
        self._initializing: Dict[int, object] = {}
        #: 登録に至らなかったロボット（初期化・ホーム設定の失敗、停止後）のデバイス報告
        self._failed_reports: Dict[int, object] = {}
        #: emergency_stop_all() が一度でも呼ばれたら True（永続）。以後 add_robot は初期化を始めない
        self._stop_requested = False
        #: 切断後に集めたロボットのデバイス報告（robot_id → dict; run_report() を持つ種別だけ）
        self.robot_reports: Dict[int, dict] = {}
        self._raise_robot_error = False
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

    def confirm_robot(self, robot_id: int) -> None:
        """人の確認（ターミナルの Enter / GUI の確認ダイアログ）を経たロボットとして記録する。
        MG400 の実機は、これが無いと initialize() が制御コマンドの前で拒否する（flow plan F7）"""
        self._confirmed_robots.add(int(robot_id))

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
        extra = {}
        if rtype == "mg400":
            if not self.mock and not (ports or {}).get("mg400_address"):
                raise RuntimeError(
                    f"Robot {robot_id}（MG400）のアドレス mg400_address が config.yaml の robots "
                    f"セクションに設定されていません（--robot{robot_id}-mg400 / "
                    f"ROBOT{robot_id}_MG400_ADDRESS でも指定できます）"
                )
            if use_picus2:
                logger.warning(f"Robot {robot_id} は MG400 のためピペット（Picus2）は使えません。"
                               "use_picus2 を無視します")
                use_picus2 = False
            extra["confirmed"] = self.mock or (robot_id in self._confirmed_robots)
            if self.record_root:
                extra["record_folder"] = os.path.join(self.record_root, f"mg400_{robot_id}")
        elif rtype == "maxarm":
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
        with self._estop_lock:
            if self._stop_requested:
                raise RuntimeError(f"Robot {robot_id}: 停止要求の後のため初期化を始めません")
        # extra（record_folder, confirmed）は MG400 にだけ渡す: 他の種別のファクトリ（テストの
        # 差し替えを含む）の呼び出しは従来どおり
        robot = self.robot_factory(
            robot_id,
            use_picus2=use_picus2,
            ports=ports or {},
            workspace_validator=validator,
            **extra,
        )
        if rtype in PRE_REGISTER_TYPES:
            # 停止リストに initialize() の前から載せる: 接続前の Stop はラッチとして
            # initialize() の最初の段階で見える（flow plan F6）
            with self._estop_lock:
                if self._stop_requested:
                    raise RuntimeError(f"Robot {robot_id}: 停止要求の後のため初期化を始めません")
                self._initializing[robot_id] = robot
        registered = False
        try:
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
            # 事前登録の種別（MG400）: 登録は停止ロックの下で、停止要求がなければ。初期化中の
            # 停止（_initializing 経由で届いている）の後に、動けるロボットが登録されることは
            # ない（flow plan F6）。拒否するときは停止を（冪等に）もう一度届けてから切断する。
            # それ以外の種別（Dobot / MaxArm）は従来どおり無条件に登録する: 停止は run() の
            # 中断処理が emergency_stop_all() で届ける（切断だけにすると MaxArm の吸着が
            # 通常終了の扱いで切れる）
            with self._estop_lock:
                if rtype not in PRE_REGISTER_TYPES or not self._stop_requested:
                    self.robots[robot_id] = robot
                    self._robot_types[robot_id] = rtype
                    registered = True
            if not registered:
                try:
                    robot.emergency_stop()
                except Exception as e:  # noqa: BLE001 - the latch was already set through _initializing
                    logger.error(f"Robot {robot_id} の緊急停止に失敗しました: {e}")
                await self._cleanup_one(f"Robot {robot_id}", robot)
                raise RuntimeError(f"Robot {robot_id}: 初期化中に停止要求があったため登録しません（切断しました）")
        finally:
            with self._estop_lock:
                self._initializing.pop(robot_id, None)     # after the registration: no gap for a stop (F6)
            if not registered:
                self._keep_failed_report(robot_id, robot)
        if rtype == "mg400":
            logger.info(f"Robot {robot_id} 初期化完了 (MG400: 記録 {getattr(robot, 'record_folder', '-')})")
        elif rtype == "maxarm":
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
        if rtype in ("maxarm", "mg400") and ports.get("home_position") is not None:
            label = "MaxArm" if rtype == "maxarm" else "MG400"
            if getattr(robot, "home_position", None) is None:
                raise RuntimeError(
                    f"Robot {robot_id}（{label}）: 設定の home_position がロボットに渡っていません")
            if not robot.home_is_within_limits():
                raise RuntimeError(
                    f"Robot {robot_id}（{label}）: 設定の home_position {list(robot.home_position)} が"
                    f"可動域（robots.{robot_id}.workspace）の外です。home_position か可動域を"
                    "確認してください")
            logger.info(f"Robot {robot_id}（{label}）: 設定の home_position "
                        f"{list(robot.home_position)} をホームにします")
            return
        robot.set_current_position_as_home()

    async def add_shared(self, *, use_scale: bool = False, use_camera: bool = False,
                         use_microscope: bool = False, use_microscope_serial: bool = False,
                         use_dmm: bool = False, use_lockin: bool = False,
                         use_scope: bool = False, lockin_osc_plan=None):
        """設定の値で共有デバイスを生成・初期化して登録する

        ``lockin_osc_plan`` はフローが LI5640 の内部発振器を書くときだけ渡す
        （渡さなければファクトリにも渡さない: 引数を知らない差し替えファクトリも動く）。

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
        if use_scope:
            kwargs["use_scope"] = True
        if lockin_osc_plan is not None:
            kwargs["lockin_osc_plan"] = lockin_osc_plan
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
    def latch_stops_first(self) -> None:
        """ラッチだけで止まる種別（MG400: ``STOP_NEVER_BLOCKS``）の停止ラッチを、ログも I/O も
        なしで全台に立てる（例外を送出しない）。run_flow の Ctrl+C 中継が、ログのロックや他の
        ロボットの同期の停止を待たずに使う。通常の ``emergency_stop_all`` は後で続ける"""
        with self._estop_lock:
            self._stop_requested = True
            targets = list(self.robots.values()) + [r for rid, r in self._initializing.items()
                                                    if rid not in self.robots]
        for robot in targets:
            if getattr(robot, "STOP_NEVER_BLOCKS", False) and callable(getattr(robot, "latch_stop", None)):
                try:
                    robot.latch_stop()
                except Exception:  # noqa: BLE001 - emergency_stop_all() below tries again and logs
                    pass

    def emergency_stop_all(self, *, skip_already_stopped: bool = False):
        """登録済み全ロボットを緊急停止する（例外を送出しない）

        同期関数で await を含まないため、イベントループがデバイス呼び出しで
        ブロックされていても別スレッド（GUI の Stop）から直接呼び出せる。

        Args:
            skip_already_stopped: True なら、すでに緊急停止済みのロボットは
                再送しない（GUI の Stop で停止済みのロボットに、キャンセル処理で
                二重に停止コマンドを送らないため）。
        """
        # 実行スレッドが add_robot で辞書を更新中でも安全にスナップショットを取る。
        # 初期化中の MG400 も含める（接続前のラッチ; flow plan F6）。以後 add_robot は
        # 新しい初期化を始めない
        with self._estop_lock:
            self._stop_requested = True
            targets = list(self.robots.items()) + [(rid, r) for rid, r in self._initializing.items()
                                                   if rid not in self.robots]
        # 停止がラッチだけの種別（MG400）を先に: 他の種別の同期の停止（シリアル送信）で遅れない
        targets.sort(key=lambda t: not getattr(t[1], "STOP_NEVER_BLOCKS", False))
        for rid, robot in targets:
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

        - 切断（cleanup）の後に共有デバイスの出力の復元結果（LI5640 の内部発振器）を
          読む: 戻せなかった・確かめられなかったら、状態を failed にし（中断・失敗は
          そのまま）エラー文に確認の注意を足す。本体が正常に終わっていた場合だけ
          :class:`OutputRestoreError` を送出する（進行中の例外・中断は置き換えない）。
          切断中に初めて中断された場合は aborted にする

        - 切断の後にロボットのデバイス報告（MG400 の ``run_report()``: 自分で止まった、
          停止シーケンスが停止を確認できなかった）を集める（``robot_reports``）。
          ``failed_reason`` のある報告は、本体が正常に終わっていても状態を failed にし
          :class:`RobotStopError` を送出する（中断・失敗はそのまま、エラー文に足す）

        exp_logger の確定は行わない。呼び出し側が finally で
        ``exp_logger.record_output_restore(session.output_restore_report())``、
        ``exp_logger.record_robot_reports(session.robot_reports)`` と
        ``exp_logger.finalize(session.status, session.error)`` を呼ぶこと。
        """
        body_completed = False
        self._raise_output_error = False
        self._raise_robot_error = False
        try:
            result = await body()
            body_completed = True
            self.status, self.error = "completed", None
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
            cleanup_interrupt = None
            try:
                await self.cleanup()
            except BaseException as e:      # cleanup() は最後に中断を再送出する
                cleanup_interrupt = e
                raise
            finally:
                self._apply_output_report(body_completed, cleanup_interrupt)
                self._apply_robot_reports(body_completed, cleanup_interrupt)
        if self._raise_output_error:
            raise OutputRestoreError(self.error)
        if self._raise_robot_error:
            raise RobotStopError(self.error)
        return result

    def _keep_failed_report(self, robot_id, robot) -> None:
        """登録に至らなかったロボットを、デバイス報告（停止シーケンスの結果など）のために残す。
        報告は _apply_robot_reports の時点で読む（切断が続いていれば、その後の結果になる）"""
        if callable(getattr(robot, "run_report", None)):
            with self._estop_lock:
                self._failed_reports[robot_id] = robot

    @staticmethod
    def _read_report(robot) -> Optional[dict]:
        try:
            return robot.run_report()
        except Exception as e:  # noqa: BLE001 - 報告の失敗で後始末を止めない
            return {"failed_reason": f"デバイス報告を読めません: {e!r}"}

    def _apply_robot_reports(self, body_completed: bool, cleanup_interrupt):
        """切断後のロボットのデバイス報告（I/O も await もしない）。登録に至らなかった
        ロボットの報告（_keep_failed_report）も含める"""
        with self._estop_lock:
            pending = list(self._failed_reports.items())
        self.robot_reports = {}
        for rid, robot in pending + list(self.robots.items()):
            if not callable(getattr(robot, "run_report", None)):
                continue
            report = self._read_report(robot)
            if report:
                self.robot_reports[rid] = report
        failed = [(rid, r["failed_reason"]) for rid, r in self.robot_reports.items()
                  if r.get("failed_reason")]
        if not failed:
            return
        note = "; ".join(f"Robot {rid}: {reason}" for rid, reason in failed)
        logger.error(f"ロボットのデバイス報告: {note}")
        if self.status == "completed":
            self.status, self.error = "failed", note
            self._raise_robot_error = cleanup_interrupt is None and body_completed
        else:
            self.error = f"{self.error}。{note}" if self.error else note

    def output_restore_report(self):
        """共有デバイスの出力の復元結果（LI5640 の発振器）。無ければ None。I/O はしない"""
        fn = getattr(self.shared, "output_restore_report", None)
        if not callable(fn):
            return None
        try:
            return fn()
        except Exception as e:  # noqa: BLE001 - 報告の失敗で後始末を止めない
            logger.error(f"出力の復元結果を読めません: {e}")
            return {"oscillator": {"state": "unverified", "errors": [repr(e)]}, "closed": None}

    def _apply_output_report(self, body_completed: bool, cleanup_interrupt):
        """切断後の状態の確定（I/O も await もしない）"""
        if cleanup_interrupt is not None and body_completed and self.status == "completed":
            self.status, self.error = "aborted", "中断（切断処理中）"
        report = self.output_restore_report()
        if not report:
            return
        state = (report.get("oscillator") or {}).get("state")
        closed = report.get("closed")
        # 書いた後は「閉じたと確かめられた」ことを求める。書いていなくても、閉じられなかった
        # （ハンドルが残った）ことは失敗として扱う
        bad = state in ("failed", "unverified") or (
            closed is not True if state not in (None, "not_needed") else closed is False)
        if not bad:
            return
        note = f"{OUTPUT_CHECK_NOTE}（発振器の復元: {state}、切断: {report.get('closed')}）"
        logger.error(note)
        if self.status == "completed":
            self.status, self.error = "failed", note
            self._raise_output_error = cleanup_interrupt is None and body_completed
        else:
            self.error = f"{self.error}。{note}" if self.error else note
