#jsonの型検証ファイル（validation有）

from pydantic import BaseModel, ConfigDict, Field, model_validator
from typing import Literal, Union, List, Optional

from src.devices.keithley2002.keithley2002 import FUNCTIONS as DMM_FUNCTIONS, NPLC_MAX, NPLC_MIN

# ==========================================
# 0. 共通の基底モデル
# ==========================================

#: ピペット 1 回の最小操作量 (mL)。実機ラッパー LabRobot.min_pipette_volume と
#: 同じ値にしておくこと（Mock は最小値を検査しないため、スキーマで揃えないと
#: Mock で通ったフローが実機で拒否される）。tests/test_phase2_schema_volume.py が一致を確認する。
PIPETTE_MIN_VOLUME_ML = 0.5
#: ピペットの最大容量 (mL)。LabRobot.max_pipette_volume と同じ。
PIPETTE_MAX_VOLUME_ML = 10.0
#: robot_id の上限。1〜3 は Dobot Magician、4 は任意の Hiwonder MaxArm
#: （config.yaml の robots.<id>.type で種別を決める）。ロボットが実際に
#: そのアクションを実行できるかは run_flow.preflight_capabilities が検査する。
MAX_ROBOT_ID = 4


class _StrictModel(BaseModel):
    """全アクションとワークフローの基底。

    - ``allow_inf_nan=False``: JSON の NaN / Infinity を拒否する。NaN は大小比較が
      常に False になり、可動域チェックをすり抜けるため。
    - ``extra="forbid"``: 未知のキーを拒否する。``"robotid": 2`` のような綴り
      間違いが黙って捨てられ、既定の robot_id=1 に送られるのを防ぐ。
      ``_iteration`` は expand_loops() が検証後に付けるキーなので影響しない。
    """
    model_config = ConfigDict(allow_inf_nan=False, extra="forbid")


# ==========================================
# 1. 個別のアクション定義 (Dobot用)
# ==========================================

#...は必須項目を示す
class ActionMoveXYZ(_StrictModel):
    """
    XYZ絶対座標への移動
    LabRobot.move_xyz(x, y, z) に対応
    """
    action: Literal["move_xyz"] = Field(
        ...,
        description="アクション識別子: 絶対座標移動"
    )
    x: float = Field(..., description="目標X座標 (mm)")
    y: float = Field(..., description="目標Y座標 (mm)")
    z: float = Field(..., description="目標Z座標 (mm)")
    robot_id: int = Field(
        default=1,
        ge=1,
        le=MAX_ROBOT_ID,
        description="操作対象のロボットID (1..4)"
    )

class ActionMoveZ(_StrictModel):
    """
    Z軸方向への相対移動
    LabRobot.move_z(distance) に対応
    """
    action: Literal["move_z"] = Field(
        ...,
        description="アクション識別子: Z軸相対移動"
    )
    distance: float = Field(
        ...,
        description="移動距離 (mm)。正の値で上昇、負の値で下降"
    )
    robot_id: int = Field(
        default=1,
        ge=1,
        le=MAX_ROBOT_ID,
        description="操作対象のロボットID (1..4)"
    )

class ActionRotate(_StrictModel):
    """
    ベース回転（Joint1）
    LabRobot.rotate(angle) に対応
    """
    action: Literal["rotate"] = Field(
        ...,
        description="アクション識別子: ベース回転"
    )
    angle: float = Field(
        ...,
        description="目標角度 (度)。Joint1の絶対角度"
    )
    robot_id: int = Field(
        default=1,
        ge=1,
        le=MAX_ROBOT_ID,
        description="操作対象のロボットID (1..4)"
    )

class ActionRotateRelative(_StrictModel):
    """
    ベース相対回転（Joint1）
    LabRobot.rotate_relative(delta_angle, speed) に対応
    """
    action: Literal["rotate_relative"] = Field(
        ...,
        description="アクション識別子: ベース相対回転"
    )
    angle: float = Field(
        ...,
        description="回転角度 (度)。現在位置からの相対角度。正=反時計回り（上から見て、+Y方向）、負=時計回り"
    )
    speed: Literal["low", "normal", "high"] = Field(
        default="low",
        description="回転速度。low=低速, normal=普通, high=高速"
    )
    robot_id: int = Field(
        default=1,
        ge=1,
        le=MAX_ROBOT_ID,
        description="操作対象のロボットID (1..4)"
    )

class ActionMoveRadial(_StrictModel):
    """
    半径方向への相対移動（円柱座標系）
    LabRobot.move_radial(distance) に対応

    アームの向きを維持したまま、基部からの距離を変更します。
    """
    action: Literal["move_radial"] = Field(
        ...,
        description="アクション識別子: 半径方向相対移動"
    )
    distance: float = Field(
        ...,
        description="半径方向の移動距離 (mm)。正の値で外向き（基部から離れる）、負の値で内向き（基部に近づく）"
    )
    robot_id: int = Field(
        default=1,
        ge=1,
        le=MAX_ROBOT_ID,
        description="操作対象のロボットID (1..4)"
    )

class ActionGoHome(_StrictModel):
    """
    ホームポジションへの復帰
    LabRobot.go_home() に対応
    """
    action: Literal["go_home"] = Field(
        ...,
        description="アクション識別子: ホーム復帰"
    )
    robot_id: int = Field(
        default=1,
        ge=1,
        le=MAX_ROBOT_ID,
        description="操作対象のロボットID (1..4)"
    )

# ==========================================
# 1.1 個別のアクション定義 (Dobot周辺機器用)
# ==========================================

class ActionMoveSlider(_StrictModel):
    """
    スライダー（リニアレール）を絶対位置に移動
    LabRobot.move_slider(position) に対応
    """
    action: Literal["move_slider"] = Field(
        ...,
        description="アクション識別子: スライダー移動"
    )
    position: float = Field(
        ...,
        ge=0,
        le=1000.0,
        description="スライダー目標位置 (mm)。0以上1000以下"
    )
    robot_id: int = Field(
        default=1,
        ge=1,
        le=MAX_ROBOT_ID,
        description="操作対象のロボットID (1..4)"
    )

class ActionMoveConveyer(_StrictModel):
    """
    コンベアベルトを動作させる
    LabRobot.move_conveyer(index, speed, duration) に対応
    """
    action: Literal["move_conveyer"] = Field(
        ...,
        description="アクション識別子: コンベアベルト動作"
    )
    index: int = Field(
        default=0,
        ge=0,
        le=1,
        description="コンベアベルトのインデックス (0 or 1)"
    )
    speed: float = Field(
        ...,
        gt=0,
        le=200,
        description="コンベアベルト速度 (mm/s)"
    )
    duration: float = Field(
        ...,
        gt=0,
        le=300,
        description="動作時間 (秒)。最大300秒"
    )
    robot_id: int = Field(
        default=1,
        ge=1,
        le=MAX_ROBOT_ID,
        description="操作対象のロボットID (1..4)"
    )

class ActionWait(_StrictModel):
    """
    待機（ユーティリティ）
    asyncio.sleep(seconds) に対応
    """
    action: Literal["wait"] = Field(
        ...,
        description="アクション識別子: 待機"
    )
    seconds: float = Field(
        ...,
        ge=0,
        description="待機時間 (秒)"
    )
    robot_id: int = Field(
        default=1,
        ge=1,
        le=MAX_ROBOT_ID,
        description="操作対象のロボットID (1..4)"
    )

# ==========================================
# 1.5 個別のアクション定義 (Picus2電動ピペット用)
# ==========================================

class ActionAspirate(_StrictModel):
    """
    液体の吸引
    LabRobot.aspirate(volume, speed) に対応

    安全機能:
    - 最大容量（10mL）を超える吸引を防止
    - 連続吸引時の容量オーバーチェック
    """
    action: Literal["aspirate"] = Field(
        ...,
        description="アクション識別子: 液体吸引"
    )
    volume: float = Field(
        ...,
        ge=PIPETTE_MIN_VOLUME_ML,
        le=PIPETTE_MAX_VOLUME_ML,
        description="吸引量 (mL)。0.5 mL 以上 10 mL 以下（実機ピペットの最小操作量が 0.5 mL）"
    )
    speed: int = Field(
        default=5,
        ge=1,
        le=9,
        description="吸引速度 (1-9)。1が最も遅く、9が最も速い"
    )
    robot_id: int = Field(
        default=1,
        ge=1,
        le=MAX_ROBOT_ID,
        description="操作対象のロボットID (1..4)"
    )

class ActionDispense(_StrictModel):
    """
    液体の分注
    LabRobot.dispense(volume, speed) に対応

    安全機能:
    - 保持量を超える分注を防止
    - 吸引なしでの分注時に警告
    """
    action: Literal["dispense"] = Field(
        ...,
        description="アクション識別子: 液体分注"
    )
    volume: float = Field(
        ...,
        ge=PIPETTE_MIN_VOLUME_ML,
        le=PIPETTE_MAX_VOLUME_ML,
        description="分注量 (mL)。0.5 mL 以上で保持量以下（実機ピペットの最小操作量が 0.5 mL）"
    )
    speed: int = Field(
        default=5,
        ge=1,
        le=9,
        description="分注速度 (1-9)。1が最も遅く、9が最も速い"
    )
    robot_id: int = Field(
        default=1,
        ge=1,
        le=MAX_ROBOT_ID,
        description="操作対象のロボットID (1..4)"
    )

class ActionBlowOut(_StrictModel):
    """
    液体の完全排出（ブローアウト）
    LabRobot.blow_out(go_home, speed, delay_ms) に対応

    チップ内に残った液体を完全に排出します。
    分注後の残液除去や、粘性の高い液体の完全吐出に使用。
    """
    action: Literal["blow_out"] = Field(
        ...,
        description="アクション識別子: 液体完全排出"
    )
    go_home: bool = Field(
        default=True,
        description="終了後にピストンをホームポジションに戻すか"
    )
    speed: int = Field(
        default=1,
        ge=1,
        le=9,
        description="排出速度 (1-9)。1が最も遅く安全"
    )
    delay_ms: int = Field(
        default=3000,
        ge=0,
        description="排出所要時間 (ミリ秒)"
    )
    robot_id: int = Field(
        default=1,
        ge=1,
        le=MAX_ROBOT_ID,
        description="操作対象のロボットID (1..4)"
    )

# ==========================================
# 1.55 個別のアクション定義 (MaxArm 吸着ノズル用)
# ==========================================

class ActionSuction(_StrictModel):
    """
    吸着ノズル（ポンプ）の ON / OFF
    MaxArmRobot.suction(on) に対応

    MaxArm（config.yaml の type: maxarm）専用。Dobot に指定すると
    プリフライトで拒否される。
    """
    action: Literal["suction"] = Field(
        ...,
        description="アクション識別子: 吸着 ON/OFF"
    )
    on: bool = Field(
        default=True,
        description="True で吸着（ポンプ ON）、False で解放（バルブを開いて離す）"
    )
    robot_id: int = Field(
        default=1,
        ge=1,
        le=MAX_ROBOT_ID,
        description="操作対象のロボットID (1..4)。MaxArm の robot_id を指定する"
    )

# ==========================================
# 1.6 個別のアクション定義 (共有デバイス: Webcam用)
# ==========================================

class ActionCaptureAndSave(_StrictModel):
    """
    Webcamで画像をキャプチャしてファイルに保存
    SharedDevices.capture_and_save(file_path) に対応

    Note:
        共有デバイスのため、robot_idは不要です。
    """
    action: Literal["capture_and_save"] = Field(
        ...,
        description="アクション識別子: 画像キャプチャ・保存"
    )
    file_path: str = Field(
        default="",
        description="保存先のファイル名（空欄で自動生成）"
    )


class ActionCaptureMicroscope(_StrictModel):
    """
    USB デジタル顕微鏡（UVC、例: サンワサプライ 400-CAM106）で画像をキャプチャして保存
    SharedDevices.capture_microscope(file_path) に対応

    Note:
        共有デバイスのため、robot_idは不要です。
    """
    action: Literal["capture_microscope"] = Field(
        ...,
        description="アクション識別子: 顕微鏡画像キャプチャ・保存"
    )
    file_path: str = Field(
        default="",
        description="保存先のファイル名（空欄で自動生成）"
    )


class ActionMicroscopeLed(_StrictModel):
    """
    USB デジタル顕微鏡（UM22 系）の LED 照明の点灯/消灯と明るさ
    SharedDevices.set_microscope_led(on, level) に対応

    制御用シリアルポート (config.yaml の shared_devices.microscope_port) が必要。

    Note:
        共有デバイスのため、robot_idは不要です。
    """
    action: Literal["microscope_led"] = Field(
        ...,
        description="アクション識別子: 顕微鏡 LED 制御"
    )
    on: bool = Field(
        default=True,
        description="True で点灯、False で消灯"
    )
    level: Optional[int] = Field(
        default=None,
        ge=0,
        le=255,
        description="明るさ (0-255、出荷時 12)。省略時は変更しない"
    )


class ActionMicroscopeFocus(_StrictModel):
    """
    USB デジタル顕微鏡（UM22 系）の焦点合わせ
    SharedDevices.focus_microscope(mode, position, direction, steps, timeout) に対応

    実行後のレンズ位置は measurements.csv の focus_position 列に記録される。
    再現性の点では、一度合焦した位置を読み取って mode="position" で指定するのが確実。

    Note:
        共有デバイスのため、robot_idは不要です。
    """
    action: Literal["microscope_focus"] = Field(
        ...,
        description="アクション識別子: 顕微鏡フォーカス"
    )
    mode: Literal["auto", "position", "step"] = Field(
        default="auto",
        description="auto = ワンショット AF / position = レンズ位置指定 / step = ステップ移動"
    )
    position: Optional[int] = Field(
        default=None,
        ge=0,
        le=65535,
        description="mode=position の目標レンズ位置（実機で 346-1766 を観測、出荷時 1568）"
    )
    direction: Literal["in", "out"] = Field(
        default="in",
        description="mode=step の方向"
    )
    steps: int = Field(
        default=1,
        ge=1,
        le=100,
        description="mode=step の回数"
    )
    timeout: float = Field(
        default=60.0,
        ge=1.0,
        le=300.0,
        description="モーター停止を待つ上限秒。AF が収束しない場合はこの時間で手動モードに戻す"
    )

    @model_validator(mode="after")
    def _position_required_for_position_mode(self):
        if self.mode == "position" and self.position is None:
            raise ValueError("microscope_focus: mode='position' には position (0-65535) が必要です")
        return self

# ==========================================
# 1.65 個別のアクション定義 (共有デバイス: Keithley 2002 DMM)
# ==========================================

class ActionMeasureDMM(_StrictModel):
    """
    Keithley 2002 デジタルマルチメータ（GPIB）で測定
    SharedDevices.measure_dmm(function, samples, nplc, range_) に対応

    結果は measurements.csv の dmm_value / dmm_unit / dmm_function / dmm_range /
    dmm_nplc / dmm_valid_samples 列に記録される。
    config.yaml の shared_devices.dmm_resource（例 GPIB0::16::INSTR）が必要。

    Note:
        共有デバイスのため、robot_idは不要です。
    """
    action: Literal["measure_dmm"] = Field(
        ...,
        description="アクション識別子: DMM 測定"
    )
    function: Literal["dc_voltage", "ac_voltage", "dc_current", "ac_current",
                      "resistance_2w", "resistance_4w"] = Field(
        default="dc_voltage",
        description="測定機能: 直流/交流電圧 (V)、直流/交流電流 (A)、2線式/4線式抵抗 (Ohm)"
    )
    samples: int = Field(
        default=3,
        ge=1,
        le=10,
        description="測定回数（有効値の中央値を記録）"
    )
    nplc: float = Field(
        default=1.0,
        ge=NPLC_MIN,
        le=NPLC_MAX,
        description="積分時間（電源周期の倍数）。大きいほど低ノイズで遅い"
    )
    range: Optional[float] = Field(
        default=None,
        gt=0,
        description="固定レンジ（その機能の単位）。省略でオートレンジ"
    )

    @model_validator(mode="after")
    def _range_within_function_limit(self):
        _, unit, max_range = DMM_FUNCTIONS[self.function]
        if self.range is not None and self.range > max_range:
            raise ValueError(f"measure_dmm: {self.function} のレンジは {max_range:g} {unit} 以下です"
                             f"（指定 {self.range:g}）")
        return self

# ==========================================
# 1.66 個別のアクション定義 (共有デバイス: NF LI5640 ロックインアンプ)
# ==========================================

class ActionMeasureLockin(_StrictModel):
    """
    NF LI5640 ロックインアンプ（GPIB）のパネル表示値を読む（読み取り専用）
    SharedDevices.measure_lockin(samples, interval) に対応

    感度・時定数・表示（X/Y、R/θ など）は前面パネルで設定する。結果は
    measurements.csv の lockin_* 列（DATA1 / DATA2 の値・種類・単位、参照周波数、
    感度、時定数、有効サンプル数、パネル設定）に記録される。
    config.yaml の shared_devices.lockin_resource が必要。

    Note:
        共有デバイスのため、robot_idは不要です。
    """
    action: Literal["measure_lockin"] = Field(
        ...,
        description="アクション識別子: ロックインアンプの読み取り"
    )
    samples: int = Field(
        default=5,
        ge=1,
        le=100,
        description="読み取り回数（有効サンプルの中央値を記録）"
    )
    interval: float = Field(
        default=0.0,
        ge=0.0,
        le=60.0,
        description="読み取りの間隔（秒）。整定待ちではない"
    )

# ==========================================
# 1.67 個別のアクション定義 (共有デバイス: Agilent 1000 シリーズ オシロスコープ)
# ==========================================

ScopeItem = Literal["vpp", "vmax", "vmin", "vamplitude", "vtop", "vbase", "vaverage", "vrms",
                    "frequency", "period", "risetime", "falltime", "pwidth", "nwidth",
                    "pdutycycle", "ndutycycle", "overshoot", "preshoot"]


class ActionMeasureScope(_StrictModel):
    """
    Agilent 1000 シリーズ（DSO1014A など）オシロスコープの自動測定値を読む（読み取り専用）
    SharedDevices.measure_scope(channel, items, samples, interval) に対応

    垂直・水平・トリガ・取り込みの設定は前面パネルで行う。結果は measurements.csv の
    scope_* 列（項目ごとの値・単位・有効回数の JSON、パネル設定）に記録される。
    config.yaml の shared_devices.scope_resource が必要。

    Note:
        共有デバイスのため、robot_idは不要です。
    """
    action: Literal["measure_scope"] = Field(
        ...,
        description="アクション識別子: オシロスコープの自動測定値の読み取り"
    )
    channel: int = Field(
        default=1,
        ge=1,
        le=4,
        description="測るチャネル（2 チャネル機では 1〜2）"
    )
    items: List[ScopeItem] = Field(
        default_factory=lambda: ["vpp", "frequency"],
        min_length=1,
        max_length=10,
        description="読む測定項目（重複なし）"
    )
    samples: int = Field(
        default=3,
        ge=1,
        le=20,
        description="読み取り回数（項目ごとに有効値の中央値を記録）。取り込みの回数ではない"
    )
    interval: float = Field(
        default=0.5,
        ge=0.0,
        le=60.0,
        description="読み取りの間隔（秒）"
    )

    @model_validator(mode="after")
    def _items_unique(self):
        if len(set(self.items)) != len(self.items):
            raise ValueError(f"measure_scope: items が重複しています: {self.items}")
        return self

# ==========================================
# 1.7 個別のアクション定義 (共有デバイス: BCE8221電子天秤用)
# ==========================================

class ActionMeasureWeight(_StrictModel):
    """
    BCE8221電子天秤で重量測定
    SharedDevices.measure_weight(stabilization_count) に対応

    Note:
        共有デバイスのため、robot_idは不要です。
    """
    action: Literal["measure_weight"] = Field(
        ...,
        description="アクション識別子: 重量測定"
    )
    stabilization_count: int = Field(
        default=3,
        ge=1,
        le=10,
        description="測定回数（中央値を返す）"
    )

class ActionTareScale(_StrictModel):
    """
    BCE8221電子天秤の風袋引き（ゼロ点リセット）
    SharedDevices.tare_scale(delay) に対応

    Note:
        共有デバイスのため、robot_idは不要です。
    """
    action: Literal["tare_scale"] = Field(
        ...,
        description="アクション識別子: 風袋引き"
    )
    delay: float = Field(
        default=1.0,
        ge=0.1,
        le=10.0,
        description="風袋引き後の待機時間 (秒)"
    )

# ==========================================
# 1.8 個別のアクション定義 (ループ制御)
# ==========================================

class ActionLoopStart(_StrictModel):
    """
    ループ開始マーカー
    loop_idで対応するloop_endと紐づける

    Note:
        これは制御フローアクションであり、デバイス操作ではありません。
        実行時にexpand_loops()によってフラットなリストに展開されます。
    """
    action: Literal["loop_start"] = Field(
        ...,
        description="アクション識別子: ループ開始"
    )
    loop_id: str = Field(
        ...,
        description="ループ識別子（対応するloop_endと一致させる）"
    )
    count: int = Field(
        ...,
        ge=1,
        le=1000,
        description="繰り返し回数（1-1000）"
    )

class ActionLoopEnd(_StrictModel):
    """
    ループ終了マーカー
    loop_idで対応するloop_startと紐づける

    Note:
        これは制御フローアクションであり、デバイス操作ではありません。
        実行時にexpand_loops()によってフラットなリストに展開されます。
    """
    action: Literal["loop_end"] = Field(
        ...,
        description="アクション識別子: ループ終了"
    )
    loop_id: str = Field(
        ...,
        description="ループ識別子（対応するloop_startと一致させる）"
    )

# ==========================================
# 2. アクションの統合型 (Union)
# ==========================================

# ここにリストされた型だけが「有効なステップ」として認められます
LabRobotAction = Union[
    # Dobot（ロボットアーム）操作
    ActionMoveXYZ,
    ActionMoveZ,
    ActionMoveRadial,
    ActionRotate,
    ActionRotateRelative,
    ActionGoHome,
    # Dobot（周辺機器）操作
    ActionMoveSlider,
    ActionMoveConveyer,
    # Picus2（電動ピペット）操作
    ActionAspirate,
    ActionDispense,
    ActionBlowOut,
    # MaxArm（吸着ノズル）操作
    ActionSuction,
    # カメラ操作
    ActionCaptureAndSave,
    # デジタル顕微鏡操作
    ActionCaptureMicroscope,
    ActionMicroscopeLed,
    ActionMicroscopeFocus,
    # 電子天秤操作（BCE8221）
    ActionMeasureWeight,
    ActionTareScale,
    ActionMeasureDMM,
    ActionMeasureLockin,
    ActionMeasureScope,
    # ユーティリティ
    ActionWait,
    # ループ制御
    ActionLoopStart,
    ActionLoopEnd,
]

# 後方互換性のためのエイリアス
DobotAction = LabRobotAction

# ==========================================
# 3. ワークフロー全体の定義
# ==========================================

class ExperimentWorkflow(_StrictModel):
    """
    実験ワークフロー全体の定義
    """
    name: str = Field(..., description="実験の名前")
    description: str = Field("", description="実験の説明")
    steps: List[LabRobotAction] = Field(..., description="実行する手順のリスト")

# ==========================================
# 4. デバッグ用: JSONスキーマの出力機能
# ==========================================
if __name__ == "__main__":
    import json
    # このスクリプトを実行すると、LLMに渡すべきJSONスキーマが表示されます
    print(json.dumps(ExperimentWorkflow.model_json_schema(), indent=2, ensure_ascii=False))