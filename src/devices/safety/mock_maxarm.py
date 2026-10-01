"""
MaxArmRobot のモック（--mock 実行・GUI の Mock モード・テスト用）

MaxArmRobot と同じ公開 API・同じ SUPPORTED_ACTIONS を持ち、同じ可動域検証を
同じ順序・同じ例外型で行う（--mock のクリーンな実行が実機の実行を予測できるように）。
実機には接続しない。
"""

import asyncio
import logging
import math
from typing import List, Optional, Sequence

from src.devices.safety.maxarm_robot import (
    MAXARM_SUPPORTED_ACTIONS,
    SUCTION_KEPT_WARNING,
    parse_home_position,
    plan_home_legs,
    validate_move_ms,
    validate_move_tolerance,
)
from src.devices.safety.validators import ValidationError, WorkspaceValidator

logger = logging.getLogger(__name__)

#: MockMaxArmRobot の既定の開始姿勢 (X, Y, Z) = MaxArm の ORIGIN（espmax.ORIGIN）
DEFAULT_MAXARM_START_POSE = (0.0, -162.8, 212.0)


def parse_maxarm_start_pose(value) -> Optional[tuple]:
    """"x,y,z" / "x,y,z,r" 文字列または 3〜4 要素のリストを (x, y, z) に変換する

    mock_robot.parse_start_pose（Dobot 用、4 要素必須）と同じ書式を受け付けるため、
    4 要素目（R）は無視する。None / 空文字は None（既定の姿勢）。

    Raises:
        ValueError: 3 つまたは 4 つの有限の数値として解釈できない場合
    """
    if value is None:
        return None
    if isinstance(value, str):
        text = value.strip()
        if not text:
            return None
        parts = text.replace(";", ",").split(",")
    else:
        parts = list(value)
    message = f"MaxArm の mock_start_pose は 'x,y,z'（または 'x,y,z,r'）で指定してください（指定値: {value!r}）"
    try:
        pose = tuple(float(p) for p in parts)
    except (TypeError, ValueError):
        raise ValueError(message)
    if len(pose) not in (3, 4) or not all(math.isfinite(v) for v in pose):
        raise ValueError(message)
    return pose[:3]


class MockMaxArmRobot:
    """MaxArmRobot のモック

    姿勢 ``_pos`` は [X, Y, Z]。開始姿勢の既定は MaxArm の ORIGIN (0, -162.8, 212)。

    緊急停止: MockLabRobot と異なり、emergency_stop() は姿勢と吸着状態を「不明」にする
    （実機の MaxArmRobot と同じ）。次の移動の前に「読み直し」（モックでは何もせず不明を
    解除するだけ）を行うので、実機の読み直し経路と同じ呼び出し順・同じ結果になる。
    移動中（模擬の待機中）に緊急停止されたら、実機と同じく姿勢不明のまま
    RuntimeError（「緊急停止」）を送出する。

    ライフサイクル: 実機と同じく、initialize() の前と cleanup() の後は移動・吸着・
    位置の取得が RuntimeError になる。可動域バリデータの無いモックは initialize() を
    拒否する。cleanup() の吸着ポリシーも実機と同じ（MaxArmRobot.cleanup を参照）。
    """

    ROBOT_TYPE = "maxarm"
    SUPPORTED_ACTIONS = MAXARM_SUPPORTED_ACTIONS

    def __init__(self,
                 workspace_validator: Optional[WorkspaceValidator] = None,
                 start_pose: Optional[Sequence[float]] = None,
                 **device_configs):
        self.workspace_validator = workspace_validator
        self.device_configs = device_configs
        pose = parse_maxarm_start_pose(start_pose) or DEFAULT_MAXARM_START_POSE
        self._pos: List[float] = [float(v) for v in pose]
        # 実機と同じ設定値の検証（範囲外なら同じく ValueError）
        self.move_ms = validate_move_ms(device_configs.get("move_ms", 1500))
        self.move_tolerance_mm = validate_move_tolerance(device_configs.get("move_tolerance_mm", 10.0))
        self.home_position = parse_home_position(device_configs.get("home_position"))
        self.home_configured = self.home_position is not None
        self.pump_off_on_cleanup = bool(device_configs.get("pump_off_on_cleanup", True))
        self.pump_off_after_estop = bool(device_configs.get("pump_off_after_estop", False))
        self.firmware_info = {"fw": "mock"}
        self._pose_unknown = False
        self._estop_generation = 0
        self._connect_generation = 0
        self._suction: Optional[bool] = False
        self._initialized = False

    async def initialize(self) -> bool:
        if self._initialized:
            # 実機の MaxArmRobot.initialize() と同じ: 二重初期化は拒否する
            raise RuntimeError("MaxArm: すでに初期化されています（cleanup() の後に再初期化してください）")
        if self.workspace_validator is None:
            raise RuntimeError(
                "MaxArm: 可動域バリデータ（workspace）がありません。検証なしでは接続しません"
                "（config.yaml の robots.<id>.workspace を設定してください）"
            )
        logger.info("=== [MOCK MaxArm] 仮想デバイス初期化 (成功) ===")
        await asyncio.sleep(0.1)
        self._connect_generation = self._estop_generation
        self._initialized = True
        return True

    def _require_initialized(self):
        """実機の _require_client と同じ（未初期化・切断後は RuntimeError）"""
        if not self._initialized:
            raise RuntimeError("MaxArmが初期化されていません")

    def _check_estop(self, gen):
        if self._estop_generation != gen:
            self._pose_unknown = True
            raise RuntimeError("MaxArm: 移動の途中で緊急停止されました")

    # ===== 姿勢 =====

    def _reread_if_unknown(self, gen):
        """実機の _read_pose（静止確認つき読み直し）に相当。モックでは不明を解除するだけ"""
        if self._pose_unknown:
            logger.warning("[MOCK MaxArm] 前回の移動が中断されたため、現在位置を読み直します（模擬）")
            self._check_estop(gen)
            self._pose_unknown = False

    def get_current_position(self) -> List[float]:
        self._require_initialized()
        return list(self._pos)

    def set_current_position_as_home(self) -> List[float]:
        self._require_initialized()
        self.home_position = list(self._pos)
        logger.info(f"✓ [MOCK MaxArm] 現在位置をホームに設定: {self.home_position}")
        self.home_is_within_limits()
        return list(self.home_position)

    def home_is_within_limits(self) -> bool:
        """MaxArmRobot.home_is_within_limits と同じ検証（可動域外・検証不能なら警告して False）"""
        if self.home_position is None:
            return True
        if self.workspace_validator is None:
            logger.warning("警告: [MOCK MaxArm] 可動域バリデータが無いため、ホームポジションを検証できません")
            return False
        home = self.home_position
        try:
            self.workspace_validator.validate_xyz(home[0], home[1], home[2])
        except ValidationError as e:
            logger.warning(
                "警告: [MOCK MaxArm] ホームポジションが workspace 可動域外です。"
                f"go_home() はこのホームへの移動を拒否します。\n{e}"
            )
            return False
        return True

    # ===== 移動 =====

    async def _move_to(self, x, y, z, gen):
        self._check_estop(gen)
        logger.info(f"[MOCK MaxArm] XYZ移動: ({x}, {y}, {z})")
        self._pose_unknown = True       # 実機と同じく移動中は姿勢不明
        await asyncio.sleep(0.2)
        # 実機と同じく、移動中の緊急停止は完了扱いにしない（姿勢不明のまま送出）
        self._check_estop(gen)
        self._pos[:] = [float(x), float(y), float(z)]
        self._pose_unknown = False
        self._check_estop(gen)

    def _validate_xyz(self, x, y, z):
        if self.workspace_validator is None:
            raise RuntimeError("MaxArm: 可動域バリデータがありません。移動しません")
        self.workspace_validator.validate_xyz(x, y, z)

    def _validate_box(self, x, y, z):
        """現在姿勢の検査: 箱・半径のみ（MaxArmRobot._validate_box と同じ）"""
        if self.workspace_validator is None:
            raise RuntimeError("MaxArm: 可動域バリデータがありません。移動しません")
        WorkspaceValidator.validate_xyz(self.workspace_validator, x, y, z)

    async def move_xyz(self, x: float, y: float, z: float):
        self._require_initialized()
        self._validate_xyz(x, y, z)
        gen = self._estop_generation
        self._reread_if_unknown(gen)
        await self._move_to(x, y, z, gen)
        logger.info("✓ [MOCK MaxArm] XYZ移動完了")

    async def move_z(self, distance: float):
        self._require_initialized()
        gen = self._estop_generation
        self._reread_if_unknown(gen)
        x, y, z = self._pos
        if self.workspace_validator is None:
            raise RuntimeError("MaxArm: 可動域バリデータがありません。移動しません")
        self.workspace_validator.validate_z_relative(z, distance)
        self._validate_xyz(x, y, z + distance)
        logger.info(f"[MOCK MaxArm] Z軸移動: {z} -> {z + distance}")
        await self._move_to(x, y, z + distance, gen)
        logger.info("✓ [MOCK MaxArm] Z軸移動完了")

    async def go_home(self):
        """MaxArmRobot.go_home と同じく、全区間を事前検証してから復帰する"""
        self._require_initialized()
        if self.home_position is None:
            raise RuntimeError(
                "MaxArm: ホームポジションが設定されていません。"
                "set_current_position_as_home() または home_position を設定してください"
            )
        gen = self._estop_generation
        self._reread_if_unknown(gen)
        legs = plan_home_legs(self._pos, self.home_position, self._validate_xyz, self._validate_box)
        logger.info("[MOCK MaxArm] ホームポジションへ復帰中...")
        for leg in legs:
            await self._move_to(*leg, gen)
        logger.info("✓ [MOCK MaxArm] ホーム復帰完了")

    async def suction(self, on: bool = True):
        self._require_initialized()
        on = bool(on)
        logger.info(f"[MOCK MaxArm] 吸着 {'ON' if on else 'OFF'}")
        try:
            await asyncio.sleep(0.1)
        except BaseException:
            self._suction = None
            raise
        self._suction = on
        logger.info(f"✓ [MOCK MaxArm] 吸着 {'ON' if on else 'OFF'} 完了")

    @property
    def suction_state(self) -> Optional[bool]:
        return self._suction

    # ===== 緊急停止・切断 =====

    def emergency_stop(self):
        """[MOCK] 緊急停止: 実機と同じく姿勢・吸着状態を不明にする（MockLabRobot とは異なる）"""
        logger.warning("[MOCK MaxArm] 緊急停止を実行しました")
        if not self._initialized:
            logger.warning("[MOCK MaxArm] は接続されていません")
            return
        self._estop_generation += 1
        self._pose_unknown = True
        self._suction = None

    @property
    def ended_abnormally(self) -> bool:
        """MaxArmRobot.ended_abnormally と同じ"""
        return self._estop_generation != self._connect_generation or self._pose_unknown

    async def cleanup(self):
        """MaxArmRobot.cleanup と同じ吸着ポリシーで仮想接続を切断する"""
        if not self._initialized:
            return
        abnormal = self.ended_abnormally
        want_pump_off = self.pump_off_after_estop if abnormal else self.pump_off_on_cleanup
        if want_pump_off:
            self._suction = False
        elif abnormal and self._suction is not False:
            logger.warning(f"!!! [MOCK MaxArm] {SUCTION_KEPT_WARNING} !!!")
        self._initialized = False
        logger.info("✓ [MOCK MaxArm] 仮想接続を切断")

    async def __aenter__(self):
        await self.initialize()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await self.cleanup()
