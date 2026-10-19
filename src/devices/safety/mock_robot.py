import asyncio
import logging
import math
from typing import Optional, Sequence

from src.devices.safety.lab_robot import LabRobot
from src.devices.safety.validators import ValidationError, WorkspaceValidator

logger = logging.getLogger(__name__)

#: MockLabRobot の既定の開始姿勢 (X, Y, Z, R)
DEFAULT_MOCK_START_POSE = (250.0, 0.0, 150.0, 0.0)


def parse_start_pose(value) -> Optional[tuple]:
    """"x,y,z,r" 文字列または 4 要素のリストを (x, y, z, r) に変換する

    None / 空文字は None を返す（既定の姿勢を使う）。

    Raises:
        ValueError: 4 つの数値として解釈できない場合
    """
    if value is None:
        return None
    if isinstance(value, str):
        text = value.strip()
        if not text:
            return None
        parts = [p for p in text.replace(";", ",").split(",")]
    else:
        parts = list(value)
    try:
        pose = tuple(float(p) for p in parts)
    except (TypeError, ValueError):
        raise ValueError(f"mock_start_pose は 'x,y,z,r' の 4 つの数値で指定してください（指定値: {value!r}）")
    if len(pose) != 4 or not all(math.isfinite(v) for v in pose):
        raise ValueError(f"mock_start_pose は 'x,y,z,r' の 4 つの数値で指定してください（指定値: {value!r}）")
    return pose


class MockLabRobot:
    """
    GUIテスト・--mock 実行用モッククラス
    LabRobotと同じメソッドを持つが、実機には接続しない。

    --mock のクリーンな実行が実機の実行を予測できるよう、LabRobot と同じ
    制約を同じ例外型で再現する:

    - ピペット保持量のトラッキング（最大容量・最小操作量・保持量を超える分注の拒否、
      blow_out で 0 に戻る、中断後は保持量不明として吸引・分注を拒否）
    - 一貫した模擬姿勢: 回転は半径と Z を保ったまま X/Y も動かし、XYZ 移動は
      Joint1 (= atan2(Y, X)) も更新する。半径方向移動は現在の半径に沿って動き、
      基部に近すぎる場合はバリデータの有無にかかわらず ValueError を送出する
    - 開始姿勢 ``start_pose``: 実機は起動時のアームの現在位置をホームとし、
      相対移動をそこから検証する。Mock でも同じ姿勢から始めると検証結果が一致する

    内部の姿勢 ``_pos`` は [X, Y, Z, Joint1]（LabRobot.home_position と同じ並び）。
    """

    def __init__(
        self,
        use_dobot: bool = False,
        use_picus2: bool = False,
        use_scale: bool = False,
        use_camera: bool = False,
        workspace_validator: Optional[WorkspaceValidator] = None,
        start_pose: Optional[Sequence[float]] = None,
        **device_configs
    ):
        self.device_configs = device_configs
        pose = parse_start_pose(start_pose) or DEFAULT_MOCK_START_POSE
        x, y, z, r = pose
        # Dobot Magician の Joint1 はアームの向きそのもの (atan2(Y, X))。
        # R は J1 + J4（回転サーボ無しなら J1 と同じ）
        if math.hypot(x, y) >= 1.0:
            j1 = math.degrees(math.atan2(y, x))
            if abs(j1 - r) > 1.0:
                logger.warning(
                    f"[MOCK] start_pose の R={r:.1f}° が X/Y から決まる Joint1={j1:.1f}° と"
                    "一致しません。Joint1 の検証には X/Y から求めた値を使います"
                )
        else:
            j1 = r
        # [X, Y, Z, Joint1]
        self._pos = [x, y, z, j1]
        self._r = r
        self._slider_pos = 0.0
        self.home_position = list(self._pos)

        # 可動域バリデータ（Noneの場合はバリデーションなし）
        self.workspace_validator = workspace_validator

        # 使用フラグ
        self.use_dobot = use_dobot
        self.use_picus2 = use_picus2
        self.use_scale = use_scale
        self.use_camera = use_camera

        # ピペット（LabRobot と同じ制約）
        self.max_pipette_volume = LabRobot.DEFAULT_MAX_PIPETTE_VOLUME
        self.min_pipette_volume = LabRobot.DEFAULT_MIN_PIPETTE_VOLUME
        self._current_pipette_volume = 0.0
        self._pipette_volume_unknown = False

        self.slider_max_position = device_configs.get(
            'slider_max_position', LabRobot.DEFAULT_SLIDER_MAX_POSITION)

    async def initialize(self) -> bool:
        logger.info("=== [MOCK] 仮想デバイス初期化 (成功) ===")
        await asyncio.sleep(0.5)
        return True

    # ===== 模擬姿勢 =====

    def get_current_position(self):
        """[X, Y, Z, R, J1, J2, J3, J4]（LabRobot / PyDobot と同じ並び。J2-J4 は 0）"""
        x, y, z, j1 = self._pos
        return [x, y, z, j1, j1, 0.0, 0.0, 0.0]

    def _set_joint1(self, angle: float):
        """半径と Z を保ったまま Joint1 を回す（MOVJ_ANGLE と同じく J2/J3 は不変）"""
        radius = math.hypot(self._pos[0], self._pos[1])
        rad = math.radians(angle)
        self._pos[0] = radius * math.cos(rad)
        self._pos[1] = radius * math.sin(rad)
        self._pos[3] = angle

    async def rotate(self, angle: float):
        """絶対角度で回転"""
        # 可動域チェック
        if self.workspace_validator:
            self.workspace_validator.validate_joint1(angle)

        logger.info(f"[MOCK Dobot] 絶対回転: {self._pos[3]:.1f}° -> {angle:.1f}°")
        await asyncio.sleep(1.0)
        self._set_joint1(angle)
        logger.info(f"✓ [MOCK Dobot] 絶対回転完了")

    async def rotate_relative(self, delta_angle: float, speed: str = None):
        """現在位置から相対的に回転"""
        current = self._pos[3]
        target = current + delta_angle

        # 可動域チェック
        if self.workspace_validator:
            self.workspace_validator.validate_joint1_relative(current, delta_angle)

        speed_info = f", 速度: {speed}" if speed else ""
        logger.info(f"[MOCK Dobot] 相対回転: {current:.1f}°から{delta_angle:+.1f}°（目標: {target:.1f}°{speed_info}）")
        await asyncio.sleep(1.0)
        self._set_joint1(target)
        logger.info(f"✓ [MOCK Dobot] 相対回転完了（現在: {target:.1f}°）")

    async def move_z(self, distance: float):
        current_z = self._pos[2]

        # 可動域チェック
        if self.workspace_validator:
            self.workspace_validator.validate_z_relative(current_z, distance)

        target_z = current_z + distance
        logger.info(f"[MOCK Dobot] Z軸移動: {current_z} -> {target_z}")
        await asyncio.sleep(0.5)
        self._pos[2] = target_z
        logger.info(f"✓ [MOCK Dobot] Z軸移動完了")

    async def move_xyz(self, x: float, y: float, z: float):
        # 可動域チェック
        if self.workspace_validator:
            self.workspace_validator.validate_xyz(x, y, z)

        logger.info(f"[MOCK Dobot] XYZ移動: ({x}, {y}, {z})")
        await asyncio.sleep(1.0)
        self._pos[0], self._pos[1], self._pos[2] = x, y, z
        if math.hypot(x, y) >= 1.0:
            self._pos[3] = math.degrees(math.atan2(y, x))
        logger.info(f"✓ [MOCK Dobot] XYZ移動完了")

    async def move_radial(self, distance: float):
        """[MOCK] 半径方向移動のシミュレーション（LabRobot.move_radial と同じ検証）"""
        x, y = self._pos[0], self._pos[1]
        r = math.sqrt(x**2 + y**2)

        if r < 1.0:
            raise ValueError(
                f"現在の半径距離が小さすぎます（r={r:.1f}mm）。"
                "基部直上では半径方向が定義できません"
            )

        scale = (r + distance) / r
        new_x, new_y = x * scale, y * scale
        if self.workspace_validator:
            self.workspace_validator.validate_xyz(new_x, new_y, self._pos[2])

        direction = "外向き" if distance > 0 else "内向き"
        logger.info(f"[MOCK Dobot] 半径方向移動: {abs(distance):.1f}mm {direction}（r: {r:.1f} -> {r + distance:.1f}mm）")
        await asyncio.sleep(0.5)
        self._pos[0], self._pos[1] = new_x, new_y
        logger.info(f"✓ [MOCK Dobot] 半径方向移動完了")

    async def move_slider(self, position: float):
        """[MOCK] スライダー移動のシミュレーション（LabRobot と同じ範囲検証）"""
        if position < 0:
            raise ValueError(f"スライダー位置は0以上で指定してください（指定値: {position}）")
        if position > self.slider_max_position:
            raise ValueError(
                f"スライダー位置が最大値を超えています"
                f"（指定値: {position}, 最大: {self.slider_max_position}）"
            )
        logger.info(f"[MOCK Dobot] スライダー移動: {self._slider_pos:.1f} -> {position:.1f}")
        await asyncio.sleep(1.0)
        self._slider_pos = position
        logger.info(f"✓ [MOCK Dobot] スライダー移動完了（位置: {position:.1f}）")

    async def move_conveyer(self, index: int, speed: float, duration: float):
        """[MOCK] コンベアベルト動作のシミュレーション（LabRobot と同じ範囲検証）"""
        if not 0 <= index <= 1:
            raise ValueError(f"コンベアインデックスは0-1の範囲で指定してください（指定値: {index}）")
        if speed <= 0 or speed > 200:
            raise ValueError(f"コンベア速度は0より大きく200以下で指定してください（指定値: {speed}）")
        if duration <= 0 or duration > 300:
            raise ValueError(f"コンベア動作時間は0より大きく300秒以下で指定してください（指定値: {duration}）")
        logger.info(
            f"[MOCK Dobot] コンベアベルト動作"
            f"（インデックス: {index}, 速度: {speed}, 時間: {duration}秒）"
        )
        await asyncio.sleep(0.5)
        logger.info(f"✓ [MOCK Dobot] コンベアベルト動作完了")

    async def go_home(self):
        """LabRobot.go_home と同じく、全区間を事前検証してから復帰する"""
        home = self.home_position
        if self.workspace_validator:
            if self._pos[2] < home[2] - 0.1:
                self.workspace_validator.validate_z_relative(self._pos[2], home[2] - self._pos[2])
            if len(home) >= 4:
                self.workspace_validator.validate_joint1(home[3])
            self.workspace_validator.validate_xyz(home[0], home[1], home[2])
        logger.info("[MOCK Dobot] ホームポジションへ復帰中...")
        await asyncio.sleep(1.0)
        self._pos[:] = list(self.home_position)
        logger.info("✓ [MOCK Dobot] ホーム復帰完了")

    def set_current_position_as_home(self):
        self.home_position = list(self._pos)
        logger.info(f"✓ [MOCK Dobot] 現在位置をホームに設定: {self.home_position}")
        self.home_is_within_limits()
        return self.home_position

    def home_is_within_limits(self) -> bool:
        """LabRobot.home_is_within_limits と同じ検証（可動域外なら警告）"""
        if self.workspace_validator is None or self.home_position is None:
            return True
        home = self.home_position
        try:
            self.workspace_validator.validate_xyz(home[0], home[1], home[2])
            if len(home) >= 4:
                self.workspace_validator.validate_joint1(home[3])
        except ValidationError as e:
            logger.warning(
                "警告: [MOCK] ホームとして取得した現在位置が workspace 可動域外です。"
                f"go_home() はこのホームへの移動を拒否します。\n{e}"
            )
            return False
        return True

    # ===== 電動ピペット操作 =====

    @staticmethod
    def _nominal_pipette_time(volume: float, speed: int) -> float:
        """Picus2 マニュアル p.63 の表に基づく公称所要時間(秒)"""
        from src.devices.picus2.picus2_controller import nominal_operation_time

        return nominal_operation_time(volume, speed)

    # 保持量トラッキングは LabRobot と同じ実装・同じメッセージを使う
    pipette_volume = LabRobot.pipette_volume
    pipette_volume_known = LabRobot.pipette_volume_known
    pipette_remaining_capacity = LabRobot.pipette_remaining_capacity
    reset_pipette_volume = LabRobot.reset_pipette_volume
    _mark_pipette_volume_unknown = LabRobot._mark_pipette_volume_unknown
    _require_known_pipette_volume = LabRobot._require_known_pipette_volume

    def _check_pipette(self, volume: float, speed: int, label: str):
        """LabRobot.aspirate / dispense と同じ順序・同じ例外型で検証する"""
        if not self.use_picus2:
            raise RuntimeError("Picus2が初期化されていません")
        self._require_known_pipette_volume()
        if volume < self.min_pipette_volume:
            raise ValueError(f"{label}量が最小値未満です（指定: {volume}mL, 最小: {self.min_pipette_volume}mL）")
        if not 1 <= speed <= 9:
            raise ValueError(f"speedは1-9の範囲で指定してください（指定値: {speed}）")

    async def aspirate(self, volume: float, speed: int = 5):
        """[MOCK] 液体吸引のシミュレーション（LabRobot と同じ保持量制約）"""
        self._check_pipette(volume, speed, "吸引")
        new_volume = self._current_pipette_volume + volume
        if new_volume > self.max_pipette_volume:
            raise ValueError(
                f"容量オーバー: 現在 {self._current_pipette_volume:.2f}mL + "
                f"吸引 {volume:.2f}mL = {new_volume:.2f}mL > "
                f"最大容量 {self.max_pipette_volume}mL"
            )
        nominal_time = self._nominal_pipette_time(volume, speed)
        logger.info(f"[MOCK] 吸引中: {volume:.2f}mL（速度: {speed}）"
                    f" [現在: {self._current_pipette_volume:.2f}mL → {new_volume:.2f}mL]")
        logger.info(f"  nominal ≈{nominal_time:.2f} s for {volume:.1f} mL at speed {speed}")
        try:
            await asyncio.sleep(0.5)
        except BaseException as e:
            self._mark_pipette_volume_unknown(f"[MOCK] 吸引が中断されました: {e!r}")
            raise
        self._current_pipette_volume = new_volume
        logger.info(f"✓ [MOCK] 吸引完了: {volume:.2f}mL（保持量: {new_volume:.2f}mL）")
        return {"nominal_time_s": nominal_time}

    async def dispense(self, volume: float, speed: int = 5):
        """[MOCK] 液体分注のシミュレーション（LabRobot と同じ保持量制約）"""
        self._check_pipette(volume, speed, "分注")
        if self._current_pipette_volume <= 0:
            logger.warning("警告: [MOCK] 吸引せずに分注しようとしています（保持量: 0mL）")
        if volume > self._current_pipette_volume:
            raise ValueError(
                f"分注量が保持量を超えています: "
                f"分注 {volume:.2f}mL > 保持量 {self._current_pipette_volume:.2f}mL"
            )
        new_volume = self._current_pipette_volume - volume
        nominal_time = self._nominal_pipette_time(volume, speed)
        logger.info(f"[MOCK] 分注中: {volume:.2f}mL（速度: {speed}）"
                    f" [現在: {self._current_pipette_volume:.2f}mL → {new_volume:.2f}mL]")
        logger.info(f"  nominal ≈{nominal_time:.2f} s for {volume:.1f} mL at speed {speed}")
        try:
            await asyncio.sleep(0.5)
        except BaseException as e:
            self._mark_pipette_volume_unknown(f"[MOCK] 分注が中断されました: {e!r}")
            raise
        self._current_pipette_volume = new_volume
        logger.info(f"✓ [MOCK] 分注完了: {volume:.2f}mL（残量: {new_volume:.2f}mL）")
        return {"nominal_time_s": nominal_time}

    async def blow_out(self, go_home: bool = True, speed: int = 1, delay_ms: int = 3000):
        """[MOCK] ブローアウトのシミュレーション（保持量を 0 に戻す）"""
        if not self.use_picus2:
            raise RuntimeError("Picus2が初期化されていません")
        if not 1 <= speed <= 9:
            raise ValueError(f"speedは1-9の範囲で指定してください（指定値: {speed}）")
        if delay_ms < 0:
            raise ValueError(f"delay_msは0以上で指定してください（指定値: {delay_ms}）")
        previous = self._current_pipette_volume
        logger.info(f"[MOCK] ブローアウト中（速度: {speed}, 待機: {delay_ms}ms）[排出量: {previous:.2f}mL]")
        try:
            await asyncio.sleep(0.5)
        except BaseException as e:
            self._mark_pipette_volume_unknown(f"[MOCK] ブローアウトが中断されました: {e!r}")
            raise
        if go_home:
            logger.info("[MOCK] ピストンをホームに戻す")
        self._current_pipette_volume = 0.0
        self._pipette_volume_unknown = False
        logger.info("✓ [MOCK] ブローアウト完了（保持量: 0.00mL）")

    # ===== 電子天秤操作（BCE8221） =====

    async def measure_weight(self, stabilization_count: int = 3) -> float:
        """
        [MOCK] BCE8221で安定した重量測定を行うシミュレーション

        Args:
            stabilization_count: 測定回数（中央値を返す）

        Returns:
            float: 測定重量（g）- モックでは固定値を返す
        """
        logger.info(f"[MOCK] BCE8221重量測定中（{stabilization_count}回測定）...")
        await asyncio.sleep(0.5)
        mock_weight = 12.345  # モック値
        logger.info(f"✓ [MOCK] 重量測定完了: {mock_weight:.3f}g")
        return mock_weight

    async def tare_scale(self, delay: float = 1.0):
        """
        [MOCK] BCE8221電子天秤を風袋引き（ゼロ点リセット）するシミュレーション

        Args:
            delay: 風袋引き後の待機時間（秒）
        """
        logger.info("[MOCK] BCE8221電子天秤を風袋引き中...")
        await asyncio.sleep(delay)
        logger.info("✓ [MOCK] 風袋引き完了")

    # ===== カメラ操作 =====

    async def capture_and_save(self, file_path: Optional[str] = None) -> Optional[str]:
        """
        [MOCK] Webcamで画像をキャプチャして保存するシミュレーション

        Args:
            file_path: 保存先のファイル名（省略時は自動生成）

        Returns:
            Optional[str]: 保存したファイルパス
        """
        mock_path = file_path or "captured_images/mock_capture.jpg"
        logger.info(f"[MOCK] Webcamで画像キャプチャ・保存中...")
        logger.info(f"  保存先: {mock_path}")
        await asyncio.sleep(0.5)
        logger.info(f"✓ [MOCK] 画像キャプチャ・保存完了: {mock_path}")
        return mock_path

    def emergency_stop(self):
        """[MOCK] 緊急停止のシミュレーション（LabRobotとのインターフェース互換用）"""
        logger.warning("[MOCK] 緊急停止を実行しました")

    async def cleanup(self):
        logger.info("✓ [MOCK] 仮想接続を切断")

    async def __aenter__(self):
        await self.initialize()
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        await self.cleanup()


class MockSharedDevices:
    """SharedDevices のモック（天秤・カメラの共有デバイス）

    MockLabRobot と対になる共有デバイス側のモック。実機を持たない環境でも
    measurements.csv / summary.md が実行時と同じ形で埋まるように、
    ``measure_weight`` は float を、``capture_and_save`` / ``capture_microscope``
    は保存先パス文字列を返す（None を返すと記録が空になり、記録経路のバグに
    気付けない）。
    """

    #: measure_weight が返すモック測定値 (g)
    MOCK_WEIGHT = 4.98
    #: measure_dmm が返すモック測定値（単位は測定機能による）
    MOCK_DMM_VALUE = 1.234
    #: measure_lockin が返すモック値（DATA1 = X [V], DATA2 = Y [V], 参照周波数 [Hz]）
    MOCK_LOCKIN_X = 1.0e-3
    MOCK_LOCKIN_Y = 2.0e-5
    MOCK_LOCKIN_FREQ = 1000.0
    #: measure_scope が返すモック値（項目 -> 値。単位は実機と同じ規則: 電圧項目は V）
    MOCK_SCOPE_VALUES = {
        "vpp": 2.0, "vmax": 1.0, "vmin": -1.0, "vamplitude": 2.0, "vtop": 1.0,
        "vbase": -1.0, "vaverage": 0.0, "vrms": 0.707, "frequency": 1000.0,
        "period": 1.0e-3, "risetime": 2.0e-6, "falltime": 2.0e-6, "pwidth": 5.0e-4,
        "nwidth": 5.0e-4, "pdutycycle": 50.0, "ndutycycle": 50.0, "overshoot": 1.0,
        "preshoot": 1.0,
    }

    def __init__(self, use_scale: bool = False, use_camera: bool = False,
                 use_microscope: bool = False, use_microscope_serial: bool = False,
                 use_dmm: bool = False, use_lockin: bool = False, use_scope: bool = False,
                 **_ignored):
        self.use_scale = use_scale
        self.use_camera = use_camera
        self.use_microscope = use_microscope
        self.use_microscope_serial = use_microscope_serial
        self.use_dmm = use_dmm
        self._dmm_open = False          # 実機と同じく initialize() 〜 cleanup() の間だけ測れる
        self.use_lockin = use_lockin
        self._lockin_open = False
        self.use_scope = use_scope
        self._scope_open = False
        self._captures = 0
        self._microscope_captures = 0
        self.microscope_led_on = True
        self.microscope_led_level = 12
        self.microscope_focus_position = 1568   # 実機の出荷時レンズ位置

    async def initialize(self) -> bool:
        logger.info("=== [MOCK] 共有デバイス初期化 (成功) ===")
        self._dmm_open = self.use_dmm
        self._lockin_open = self.use_lockin
        self._scope_open = self.use_scope
        return True

    async def measure_weight(self, stabilization_count: int = 3) -> float:
        logger.info(f"[MOCK] 重量測定中（{stabilization_count}回測定）...")
        await asyncio.sleep(0.1)
        logger.info(f"✓ [MOCK] 重量測定完了: {self.MOCK_WEIGHT:.3f}g")
        return self.MOCK_WEIGHT

    async def measure_dmm(self, function: str = "dc_voltage", samples: int = 3,
                          nplc: float = 1.0, range_: Optional[float] = None) -> dict:
        # 実機と同じ契約: 未初期化は RuntimeError、有効値なし（samples < 1）は ValueError、
        # 引数はドライバの configure と同じ表で検査する
        from src.devices.keithley2002 import FUNCTIONS, NPLC_MAX, NPLC_MIN
        if not self.use_dmm or not self._dmm_open:
            raise RuntimeError("Keithley 2002 が初期化されていません")
        if int(samples) < 1:
            raise ValueError(f"DMM から有効な測定値を取得できませんでした（{samples} 回）")
        if function not in FUNCTIONS:
            raise ValueError(f"未知の測定機能: {function!r}")
        _, unit, max_range = FUNCTIONS[function]
        if not (NPLC_MIN <= float(nplc) <= NPLC_MAX):
            raise ValueError(f"nplc は {NPLC_MIN}〜{NPLC_MAX}: {nplc}")
        if range_ is not None and not (0 < float(range_) <= max_range):
            raise ValueError(f"{function} のレンジは 0 より大きく {max_range:g} {unit} 以下: {range_}")
        logger.info(f"[MOCK] DMM 測定中（{function}, {samples}回, NPLC {nplc}）...")
        await asyncio.sleep(0.05)
        logger.info(f"✓ [MOCK] DMM 測定完了: {self.MOCK_DMM_VALUE} {unit}")
        return {
            "dmm_value": self.MOCK_DMM_VALUE,
            "dmm_unit": unit,
            "dmm_function": function,
            "dmm_range": float(range_) if range_ is not None else max_range,
            "dmm_nplc": float(nplc),
            "dmm_valid_samples": int(samples),
        }

    async def measure_lockin(self, samples: int = 5, interval: float = 0.0) -> dict:
        # 実機と同じ契約: 未初期化は RuntimeError、samples < 1 は ValueError。
        # 表示は X / Y（電圧入力、正規化・レシオなし）として返す
        if not self.use_lockin or not self._lockin_open:
            raise RuntimeError("LI5640 が初期化されていません")
        if int(samples) < 1:
            raise ValueError(f"samples は 1 以上: {samples}")
        logger.info(f"[MOCK] LI5640 読み取り中（{samples}回, 間隔 {interval}s）...")
        await asyncio.sleep(0.05)
        logger.info(f"✓ [MOCK] LI5640: X = {self.MOCK_LOCKIN_X} V, Y = {self.MOCK_LOCKIN_Y} V")
        return {
            "lockin_data1": self.MOCK_LOCKIN_X,
            "lockin_data1_kind": "X",
            "lockin_data1_unit": "V",
            "lockin_data2": self.MOCK_LOCKIN_Y,
            "lockin_data2_kind": "Y",
            "lockin_data2_unit": "V",
            "lockin_freq": self.MOCK_LOCKIN_FREQ,
            "lockin_sensitivity": "100 mV",
            "lockin_time_constant": "100 ms",
            "lockin_valid_samples": int(samples),
            "lockin_invalid": {},
            "lockin_settings": {"mock": True},
        }

    async def measure_scope(self, channel: int = 1, items=("vpp", "frequency"),
                            samples: int = 3, interval: float = 0.5) -> dict:
        # 実機と同じ契約: 未初期化は RuntimeError、引数は同じ規則で検査する。
        # 4 チャネル機（DSO1014A）として振る舞う
        from src.devices.dso1000 import ITEMS
        from src.devices.dso1000.dso1000 import item_unit
        if not self.use_scope or not self._scope_open:
            raise RuntimeError("オシロスコープが初期化されていません")
        items = list(items)
        if not items or len(set(items)) != len(items) or any(i not in ITEMS for i in items):
            raise ValueError(f"items は重複のない既知の項目 1 つ以上: {items}")
        if int(samples) < 1:
            raise ValueError(f"samples は 1 以上: {samples}")
        if not 1 <= int(channel) <= 4:
            raise ValueError(f"channel は 1〜4: {channel}")
        logger.info(f"[MOCK] オシロスコープ CH{channel} 読み取り中（{items}, {samples}回）...")
        await asyncio.sleep(0.05)
        values = {i: self.MOCK_SCOPE_VALUES[i] for i in items}
        logger.info(f"✓ [MOCK] オシロスコープ: {values}")
        return {
            "scope_channel": int(channel),
            "scope_values": values,
            "scope_units": {i: item_unit(i, "V") for i in items},
            "scope_valid": {i: int(samples) for i in items},
            "scope_invalid": {},
            "scope_settings": {"mock": True},
            "scope_settings_after": {"mock": True},
            "scope_raw": {i: [repr(values[i])] * int(samples) for i in items},
        }

    async def tare_scale(self, delay: float = 1.0):
        logger.info("[MOCK] 電子天秤を風袋引き中...")
        await asyncio.sleep(min(delay, 0.1))
        logger.info("✓ [MOCK] 風袋引き完了")

    async def capture_and_save(self, file_path: Optional[str] = None) -> str:
        self._captures += 1
        path = file_path or f"captured_images/mock_capture_{self._captures:03d}.jpg"
        logger.info(f"[MOCK] 画像キャプチャ・保存中: {path}")
        await asyncio.sleep(0.1)
        logger.info(f"✓ [MOCK] 画像キャプチャ・保存完了: {path}")
        return path

    async def capture_microscope(self, file_path: Optional[str] = None) -> str:
        self._microscope_captures += 1
        path = file_path or f"microscope_images/mock_microscope_{self._microscope_captures:03d}.jpg"
        logger.info(f"[MOCK] 顕微鏡画像キャプチャ・保存中: {path}")
        await asyncio.sleep(0.1)
        logger.info(f"✓ [MOCK] 顕微鏡画像キャプチャ・保存完了: {path}")
        return path

    async def set_microscope_led(self, on: bool = True, level: Optional[int] = None) -> bool:
        if level is not None:
            self.microscope_led_level = int(level)
        self.microscope_led_on = bool(on)
        await asyncio.sleep(0.05)
        logger.info(f"✓ [MOCK] 顕微鏡 LED {'ON' if on else 'OFF'}"
                    + (f" (level {level})" if level is not None else ""))
        return self.microscope_led_on

    async def focus_microscope(self, mode: str = "auto", position: Optional[int] = None,
                               direction: str = "in", steps: int = 1,
                               timeout: float = 60.0) -> dict:
        if mode == "auto":
            self.microscope_focus_position = 1400
        elif mode == "position":
            if position is None:
                raise ValueError("mode='position' には position が必要です")
            self.microscope_focus_position = int(position)
        elif mode == "step":
            # 実機観測: "out" で位置の値が増え、"in" で減る（1 押し ≈ 14）
            self.microscope_focus_position += (-14 if direction == "in" else 14) * int(steps)
        else:
            raise ValueError(f"unknown focus mode: {mode!r}")
        await asyncio.sleep(0.05)
        logger.info(f"✓ [MOCK] 顕微鏡フォーカス {mode}: レンズ位置 {self.microscope_focus_position}")
        return {"focus_position": self.microscope_focus_position, "focus_converged": True}

    async def cleanup(self):
        self._dmm_open = False
        self._lockin_open = False
        self._scope_open = False
        logger.info("✓ [MOCK] 共有デバイスの仮想接続を切断")
