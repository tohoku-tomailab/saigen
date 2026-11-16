#jsonの型検証ファイル（validation有）

from pydantic import AfterValidator, BaseModel, ConfigDict, Field, model_validator
from typing import Annotated, Literal, Union, List, Optional

# DAQ のチャンネル名の検査（ハードウェアにも nidaqmx にも依存しない）
from src.devices.nidaq.policy import check_flow_name, physical_kind

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
# 1.75 個別のアクション定義 (共有デバイス: NI DAQ USB-6351 用)
# ==========================================
#
# チャンネルは、config.yaml の shared_devices.daq_signals で付けた信号名
# （"photodiode" など）か、デバイス名を付けない物理名（"ai0" / "ao0" /
# "port0/line0" / "ctr0" / "pfi0"）で書く。配線（どの端子か、差動か、換算係数）は
# config.yaml、実験の手順はフロー、と分けるためで、信号名で書いたフローは配線の
# 違う別の装置でもそのまま使える。デバイス名（Dev1 など）は daq_device。
#
# 出力（daq_write_* / daq_pulse_digital / daq_generate_*）は config.yaml の
# daq_ao_channels / daq_do_lines / daq_co_counters に載せた物理チャンネルだけが
# 実行時（--mock を含む）に許可される。駆動した出力は実行終了時（中断・エラーを
# 含む）に 0 V / Low / 停止 に戻る。

def _daq_name(*kinds):
    """チャンネル欄の検査: その種類の物理名か、信号名の形をしていること。"""
    def check(value: str) -> str:
        check_flow_name(value, kinds)
        return value
    return AfterValidator(check)


DaqAiName = Annotated[str, _daq_name("ai")]
DaqAoName = Annotated[str, _daq_name("ao")]
DaqLineName = Annotated[str, _daq_name("line")]
DaqCounterName = Annotated[str, _daq_name("counter")]
DaqTerminalName = Annotated[str, _daq_name("terminal")]
DaqOutputName = Annotated[str, _daq_name("ao", "line", "counter")]

DaqVRange = Optional[Literal[0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0]]
DaqTerminalConfig = Optional[Literal["default", "rse", "nrse", "diff"]]
_V_RANGE_DOC = ("入力レンジ ±v_range (V)。信号が収まる最小のレンジが最も分解能が高い。"
                "省略時は信号の設定、無ければ 10")
_TERMINAL_DOC = ("配線方式: rse = GND 基準シングルエンド / nrse / diff = 差動 / default = デバイス既定。"
                 "省略時は信号の設定、無ければ default")


class ActionDaqReadAnalog(_StrictModel):
    """
    DAQ のアナログ入力 1 チャンネルの電圧を読む（samples 回の平均）

    測定値は measurements.csv の voltage_v 列に記録される。信号に換算係数
    （scale / offset / unit）があれば、換算値が daq_value / daq_unit 列にも入る。

    Note:
        共有デバイスのため、robot_idは不要です。
    """
    action: Literal["daq_read_analog"] = Field(
        ...,
        description="アクション識別子: DAQ アナログ入力"
    )
    channel: DaqAiName = Field(
        ...,
        description="アナログ入力（信号名、または ai0-ai15）"
    )
    samples: int = Field(
        default=100,
        ge=1,
        le=100000,
        description="平均するサンプル数（1 = 単発読み取り）"
    )
    rate_hz: float = Field(
        default=1000.0,
        ge=1.0,
        le=100000.0,
        description="サンプリング周波数 (Hz)。samples / rate_hz 秒かけて取り込む"
    )
    v_range: DaqVRange = Field(default=None, description=_V_RANGE_DOC)
    terminal: DaqTerminalConfig = Field(default=None, description=_TERMINAL_DOC)

    @model_validator(mode="after")
    def _acquisition_time_bounded(self):
        if self.samples / self.rate_hz > 60.0:
            raise ValueError("daq_read_analog: samples / rate_hz（取り込み時間）は 60 秒以下にしてください")
        return self


class ActionDaqAcquire(_StrictModel):
    """
    DAQ のアナログ入力（複数可）の波形をハードウェアタイミングで取り込み、CSV に保存する

    rate_hz × duration_s 点を全チャンネル同時刻で取り込む。trigger を指定すると、
    その端子のデジタルエッジで取り込みを開始する。trigger_level も指定すると
    アナログトリガになり、trigger（channels の先頭のアナログ入力）がその電圧を
    trigger_edge の向きに横切ったときに始まる。pretrigger_s を指定すると、トリガの
    前の波形も含めて取り込む（CSV の時刻はトリガが 0、それより前は負）。clock を
    指定すると、その端子のエッジごとに 1 点取り込む（外部サンプルクロック。rate_hz は
    そのクロックの最高周波数）。保存先は実行フォルダの data/
    （measurements.csv の data_path 列）。1 チャンネルのときは平均が voltage_v 列にも入る。

    Note:
        共有デバイスのため、robot_idは不要です。
    """
    action: Literal["daq_acquire"] = Field(
        ...,
        description="アクション識別子: DAQ 波形取り込み"
    )
    channels: List[DaqAiName] = Field(
        ...,
        min_length=1,
        max_length=16,
        description="アナログ入力のリスト（信号名、または ai0-ai15）"
    )
    rate_hz: float = Field(
        default=1000.0,
        gt=0.0,
        le=1250000.0,
        description="チャンネルあたりのサンプリング周波数 (Hz)。1 ch は 1.25 MHz まで、"
                    "複数 ch は合計 1 MHz まで"
    )
    duration_s: float = Field(
        default=1.0,
        gt=0.0,
        le=3600.0,
        description="取り込み時間 (秒)"
    )
    v_range: DaqVRange = Field(default=None, description=_V_RANGE_DOC)
    terminal: DaqTerminalConfig = Field(default=None, description=_TERMINAL_DOC)
    trigger: Optional[Annotated[str, _daq_name("terminal", "ai")]] = Field(
        default=None,
        description="開始トリガの端子（信号名、または pfi0-pfi15）。省略時はすぐ開始。"
                    "trigger_level を指定するときは channels の先頭のアナログ入力"
    )
    trigger_level: Optional[float] = Field(
        default=None,
        ge=-10.0,
        le=10.0,
        description="アナログトリガのレベル (V)。指定すると trigger はアナログ入力になる"
    )
    pretrigger_s: Optional[float] = Field(
        default=None,
        gt=0.0,
        le=3600.0,
        description="duration_s のうちトリガより前に取り込む時間 (秒)。trigger が必要"
    )
    clock: Optional[DaqTerminalName] = Field(
        default=None,
        description="外部サンプルクロックの端子（信号名、または pfi0-pfi15）。省略時は内部クロック"
    )
    trigger_edge: Literal["rising", "falling"] = Field(
        default="rising",
        description="トリガのエッジ"
    )
    timeout_s: float = Field(
        default=10.0,
        gt=0.0,
        le=3600.0,
        description="トリガを待つ上限 (秒)。来なければステップは失敗する"
    )
    file_path: Optional[str] = Field(
        default=None,
        description="保存先 CSV（省略時は実行フォルダの data/ に自動で付ける）"
    )

    @model_validator(mode="after")
    def _within_device_limits(self):
        n = len(self.channels)
        if len({c.strip().lower() for c in self.channels}) != n:
            raise ValueError("daq_acquire: channels に同じチャンネルが重複しています")
        if n > 1 and self.rate_hz * n > 1000000.0:
            raise ValueError("daq_acquire: 複数チャンネルでは rate_hz × チャンネル数を 1 MHz 以下に"
                             "してください")
        points = self.rate_hz * self.duration_s
        if points < 2:
            raise ValueError("daq_acquire: rate_hz × duration_s は 2 点以上にしてください")
        if points * n > 2000000:
            raise ValueError("daq_acquire: 1 ステップは合計 200 万点まで（短くするか rate_hz を下げる。"
                             "長時間の記録は daq_start_logging）")
        if self.trigger_level is not None:
            if self.trigger is None:
                raise ValueError("daq_acquire: trigger_level には trigger（channels の先頭の"
                                 "アナログ入力）が必要です")
            if self.trigger.strip().lower() != self.channels[0].strip().lower():
                raise ValueError("daq_acquire: アナログトリガの trigger は channels の先頭と同じに"
                                 "してください")
        elif self.trigger is not None and physical_kind(self.trigger) == "ai":
            raise ValueError("daq_acquire: アナログ入力をトリガにするには trigger_level を指定して"
                             "ください（デジタルトリガは pfi0 など）")
        if self.pretrigger_s is not None:
            if self.trigger is None:
                raise ValueError("daq_acquire: pretrigger_s には trigger が必要です")
            before = self.rate_hz * self.pretrigger_s
            if before < 2 or points - before < 2:
                raise ValueError("daq_acquire: pretrigger_s は 0 より大きく duration_s 未満で、"
                                 "前後に 2 点以上残るようにしてください")
        return self


class ActionDaqStartLogging(_StrictModel):
    """
    DAQ のアナログ入力（複数可）の連続記録を開始する（他のステップの実行中も記録が続く）

    daq_stop_logging か実行終了まで、CSV に書き続ける。保存先は実行フォルダの data/。
    記録中は他の daq_read_analog / daq_acquire / daq_wait_analog は使えない
    （アナログ入力のタイミング回路は 1 つ。読みたいチャンネルは channels に入れる）。

    Note:
        共有デバイスのため、robot_idは不要です。
    """
    action: Literal["daq_start_logging"] = Field(
        ...,
        description="アクション識別子: DAQ 連続記録の開始"
    )
    channels: List[DaqAiName] = Field(
        ...,
        min_length=1,
        max_length=16,
        description="アナログ入力のリスト（信号名、または ai0-ai15）"
    )
    rate_hz: float = Field(
        default=100.0,
        gt=0.0,
        le=10000.0,
        description="チャンネルあたりのサンプリング周波数 (Hz)。CSV に全点を書くので 10 kHz まで"
    )
    v_range: DaqVRange = Field(default=None, description=_V_RANGE_DOC)
    terminal: DaqTerminalConfig = Field(default=None, description=_TERMINAL_DOC)
    file_path: Optional[str] = Field(
        default=None,
        description="保存先 CSV（省略時は実行フォルダの data/ に自動で付ける）"
    )

    @model_validator(mode="after")
    def _no_duplicates(self):
        if len({c.strip().lower() for c in self.channels}) != len(self.channels):
            raise ValueError("daq_start_logging: channels に同じチャンネルが重複しています")
        return self


class ActionDaqStopLogging(_StrictModel):
    """
    daq_start_logging で始めた連続記録を止める（記録した点数が daq_value 列に入る）

    Note:
        共有デバイスのため、robot_idは不要です。
    """
    action: Literal["daq_stop_logging"] = Field(
        ...,
        description="アクション識別子: DAQ 連続記録の終了"
    )


class ActionDaqWaitAnalog(_StrictModel):
    """
    DAQ のアナログ入力がしきい値を超える（下回る）まで待つ

    timeout_s 以内に条件が成立しなければステップは失敗し、フローは止まる。
    しきい値は、信号に換算係数があればその単位、無ければ V。Mock では成立したものとして進む。

    Note:
        共有デバイスのため、robot_idは不要です。
    """
    action: Literal["daq_wait_analog"] = Field(
        ...,
        description="アクション識別子: DAQ アナログ入力の条件待ち"
    )
    channel: DaqAiName = Field(
        ...,
        description="アナログ入力（信号名、または ai0-ai15）"
    )
    threshold: float = Field(
        ...,
        description="しきい値（信号の単位。換算の無い信号・物理名では V）"
    )
    condition: Literal["above", "below"] = Field(
        default="above",
        description="above = しきい値より大きくなるまで / below = 小さくなるまで"
    )
    timeout_s: float = Field(
        default=60.0,
        gt=0.0,
        le=86400.0,
        description="待つ上限 (秒)"
    )
    samples: int = Field(
        default=10,
        ge=1,
        le=1000,
        description="1 回の判定で平均するサンプル数（1 kHz）"
    )
    v_range: DaqVRange = Field(default=None, description=_V_RANGE_DOC)
    terminal: DaqTerminalConfig = Field(default=None, description=_TERMINAL_DOC)


class ActionDaqWriteAnalog(_StrictModel):
    """
    DAQ のアナログ出力 1 チャンネルを DC 電圧に設定する（次の設定か実行終了まで保持）

    実行終了時（中断・エラーを含む）に 0 V に戻る。config.yaml の daq_ao_channels に
    無いチャンネル、daq_ao_min_v..daq_ao_max_v の外の電圧は実行時に拒否される。

    Note:
        共有デバイスのため、robot_idは不要です。
    """
    action: Literal["daq_write_analog"] = Field(
        ...,
        description="アクション識別子: DAQ アナログ出力"
    )
    channel: DaqAoName = Field(
        ...,
        description="アナログ出力（信号名、または ao0-ao1）"
    )
    voltage: float = Field(
        ...,
        ge=-10.0,
        le=10.0,
        description="出力電圧 (V)。許容範囲は config.yaml の daq_ao_min_v / daq_ao_max_v でさらに絞られる"
    )


class ActionDaqGenerateWaveform(_StrictModel):
    """
    DAQ のアナログ出力から周期波形を出す（ハードウェアタイミング）

    offset_v ± amplitude_v の範囲で振れる。両端とも config.yaml の
    daq_ao_min_v..daq_ao_max_v に入っていなければ拒否される。duration_s を指定すると
    その時間だけ出して 0 V に戻す（ステップはその間ブロックする）。省略すると
    daq_stop_output か実行終了まで出し続け、次のステップへ進む。

    Note:
        共有デバイスのため、robot_idは不要です。
    """
    action: Literal["daq_generate_waveform"] = Field(
        ...,
        description="アクション識別子: DAQ 波形出力"
    )
    channel: DaqAoName = Field(
        ...,
        description="アナログ出力（信号名、または ao0-ao1）"
    )
    waveform: Literal["sine", "square", "triangle", "sawtooth"] = Field(
        default="sine",
        description="波形"
    )
    frequency_hz: float = Field(
        ...,
        gt=0.0,
        le=100000.0,
        description="周波数 (Hz)"
    )
    amplitude_v: float = Field(
        ...,
        ge=0.0,
        le=10.0,
        description="振幅 (V, 片側)。出力は offset_v ± amplitude_v"
    )
    offset_v: float = Field(
        default=0.0,
        ge=-10.0,
        le=10.0,
        description="オフセット (V)"
    )
    duration_s: Optional[float] = Field(
        default=None,
        gt=0.0,
        le=86400.0,
        description="出力時間 (秒)。省略時は daq_stop_output か実行終了まで"
    )


class ActionDaqStopOutput(_StrictModel):
    """
    DAQ の出力 1 つを安全な状態に戻す（アナログ出力 = 0 V、デジタル = Low、パルス列 = 停止）

    このランで駆動していない出力には何も送らない。

    Note:
        共有デバイスのため、robot_idは不要です。
    """
    action: Literal["daq_stop_output"] = Field(
        ...,
        description="アクション識別子: DAQ 出力の停止"
    )
    channel: DaqOutputName = Field(
        ...,
        description="出力（信号名、または ao0 / port0/line0 / ctr0 など）"
    )


class ActionDaqReadDigital(_StrictModel):
    """
    DAQ のデジタル 1 ラインを入力として読む

    結果は measurements.csv の digital_state 列に 1 (High) / 0 (Low) で記録される。

    Note:
        共有デバイスのため、robot_idは不要です。
    """
    action: Literal["daq_read_digital"] = Field(
        ...,
        description="アクション識別子: DAQ デジタル入力"
    )
    line: DaqLineName = Field(
        ...,
        description="デジタルライン（信号名、または port0/line0 など）"
    )


class ActionDaqWriteDigital(_StrictModel):
    """
    DAQ のデジタル 1 ラインを High / Low に駆動する（次の設定か実行終了まで保持）

    実行終了時（中断・エラーを含む）に Low に戻る。config.yaml の daq_do_lines に
    無いラインは実行時に拒否される。

    Note:
        共有デバイスのため、robot_idは不要です。
    """
    action: Literal["daq_write_digital"] = Field(
        ...,
        description="アクション識別子: DAQ デジタル出力"
    )
    line: DaqLineName = Field(
        ...,
        description="デジタルライン（信号名、または port0/line0 など）"
    )
    state: bool = Field(
        ...,
        description="True = High (5 V), False = Low (0 V)"
    )


class ActionDaqPulseDigital(_StrictModel):
    """
    DAQ のデジタル 1 ラインを duration_s のあいだ state にして、元に戻す（ソフトウェアタイミング）

    時間の精度は数 ms 程度。正確なパルスや繰り返しは daq_generate_pulses。

    Note:
        共有デバイスのため、robot_idは不要です。
    """
    action: Literal["daq_pulse_digital"] = Field(
        ...,
        description="アクション識別子: DAQ デジタルパルス"
    )
    line: DaqLineName = Field(
        ...,
        description="デジタルライン（信号名、または port0/line0 など）"
    )
    duration_s: float = Field(
        ...,
        gt=0.0,
        le=3600.0,
        description="パルスの長さ (秒)"
    )
    state: bool = Field(
        default=True,
        description="パルス中のレベル。True = High にして Low に戻す"
    )


class ActionDaqWaitDigital(_StrictModel):
    """
    DAQ のデジタル 1 ラインが指定のレベルになるまで待つ（外部機器の完了信号・スイッチなど）

    timeout_s 以内にならなければステップは失敗し、フローは止まる。
    Mock では成立したものとして進む。

    Note:
        共有デバイスのため、robot_idは不要です。
    """
    action: Literal["daq_wait_digital"] = Field(
        ...,
        description="アクション識別子: DAQ デジタル入力の条件待ち"
    )
    line: DaqLineName = Field(
        ...,
        description="デジタルライン（信号名、または port0/line0 など）"
    )
    state: bool = Field(
        default=True,
        description="待つレベル。True = High"
    )
    timeout_s: float = Field(
        default=60.0,
        gt=0.0,
        le=86400.0,
        description="待つ上限 (秒)"
    )


class ActionDaqCountEdges(_StrictModel):
    """
    DAQ のカウンタで、端子に来たエッジを duration_s のあいだ数える

    結果は daq_value 列（daq_unit = count）。流量計・回転計・フォトンカウンタなどのパルス出力用。

    Note:
        共有デバイスのため、robot_idは不要です。
    """
    action: Literal["daq_count_edges"] = Field(
        ...,
        description="アクション識別子: DAQ エッジカウント"
    )
    terminal: DaqTerminalName = Field(
        ...,
        description="入力端子（信号名、または pfi0-pfi15。100khztimebase は配線不要の内部基準）"
    )
    duration_s: float = Field(
        default=1.0,
        gt=0.0,
        le=3600.0,
        description="数える時間 (秒, ソフトウェアタイミング)"
    )
    edge: Literal["rising", "falling"] = Field(
        default="rising",
        description="数えるエッジ"
    )
    counter: Optional[DaqCounterName] = Field(
        default=None,
        description="使うカウンタ（信号名、または ctr0-ctr3）。省略時は空いているものを使う"
    )


class ActionDaqMeasureFrequency(_StrictModel):
    """
    DAQ のカウンタで、端子の信号の周波数を測る（1 周期から求める）

    結果は daq_value 列（daq_unit = Hz）。timeout_s 以内に信号が来なければ失敗する。

    Note:
        共有デバイスのため、robot_idは不要です。
    """
    action: Literal["daq_measure_frequency"] = Field(
        ...,
        description="アクション識別子: DAQ 周波数測定"
    )
    terminal: DaqTerminalName = Field(
        ...,
        description="入力端子（信号名、または pfi0-pfi15）"
    )
    min_hz: float = Field(
        default=1.0,
        gt=0.0,
        le=10000000.0,
        description="予想される周波数の下限 (Hz)"
    )
    max_hz: float = Field(
        default=100000.0,
        gt=0.0,
        le=10000000.0,
        description="予想される周波数の上限 (Hz)"
    )
    timeout_s: float = Field(
        default=5.0,
        gt=0.0,
        le=600.0,
        description="信号を待つ上限 (秒)"
    )
    edge: Literal["rising", "falling"] = Field(
        default="rising",
        description="周期の基準にするエッジ"
    )
    counter: Optional[DaqCounterName] = Field(
        default=None,
        description="使うカウンタ（信号名、または ctr0-ctr3）。省略時は空いているものを使う"
    )

    @model_validator(mode="after")
    def _range_ordered(self):
        if not self.min_hz < self.max_hz:
            raise ValueError("daq_measure_frequency: min_hz < max_hz にしてください")
        return self


class ActionDaqGeneratePulses(_StrictModel):
    """
    DAQ のカウンタからパルス列（PWM）を出す（ハードウェアタイミング）

    出力端子はカウンタごとの既定（USB-6351: ctr0 = PFI12, ctr1 = PFI13, ctr2 = PFI14,
    ctr3 = PFI15）。config.yaml の daq_co_counters に無いカウンタ、daq_co_max_hz を
    超える周波数は実行時に拒否される。count（パルス数）か duration_s（秒）を指定すると
    その分だけ出して止める（ステップはその間ブロックする）。どちらも省略すると
    daq_stop_output か実行終了まで出し続け、次のステップへ進む。

    Note:
        共有デバイスのため、robot_idは不要です。
    """
    action: Literal["daq_generate_pulses"] = Field(
        ...,
        description="アクション識別子: DAQ パルス列出力"
    )
    counter: DaqCounterName = Field(
        ...,
        description="カウンタ（信号名、または ctr0-ctr3）"
    )
    frequency_hz: float = Field(
        ...,
        gt=0.0,
        le=1000000.0,
        description="周波数 (Hz)。config.yaml の daq_co_max_hz でさらに絞られる"
    )
    duty_cycle: float = Field(
        default=0.5,
        gt=0.0,
        lt=1.0,
        description="デューティ比（High の割合、0-1）"
    )
    count: Optional[int] = Field(
        default=None,
        ge=1,
        le=100000000,
        description="パルス数。出し終えるまで待つ"
    )
    duration_s: Optional[float] = Field(
        default=None,
        gt=0.0,
        le=86400.0,
        description="出力時間 (秒)"
    )

    @model_validator(mode="after")
    def _count_or_duration(self):
        if self.count is not None and self.duration_s is not None:
            raise ValueError("daq_generate_pulses: count と duration_s はどちらか一方にしてください")
        if self.count is not None and self.count / self.frequency_hz > 86400.0:
            raise ValueError("daq_generate_pulses: count / frequency_hz は 86400 秒以下にしてください")
        return self


class ActionDaqStimulateAcquire(_StrictModel):
    """
    DAQ のアナログ出力から波形を出し、同じ瞬間からアナログ入力を取り込む（刺激と応答）

    取り込みは出力の開始トリガで始まる（CSV の 0 秒が波形の開始。出力と入力は別の
    サンプルクロックで動くため、対応は約 1 サンプル以内で、実機では未測定）。
    取り込みが終わると出力は 0 V に戻る。CSV には応答のほか、指令した刺激の値
    （<出力>_set_V 列）も入る。出力の許可と電圧範囲は daq_generate_waveform と同じ。

    Note:
        共有デバイスのため、robot_idは不要です。
    """
    action: Literal["daq_stimulate_acquire"] = Field(
        ...,
        description="アクション識別子: DAQ 刺激と応答の同時測定"
    )
    channel: DaqAoName = Field(
        ...,
        description="刺激を出すアナログ出力（信号名、または ao0-ao1）"
    )
    waveform: Literal["sine", "square", "triangle", "sawtooth"] = Field(
        default="sine",
        description="刺激の波形"
    )
    frequency_hz: float = Field(
        ...,
        gt=0.0,
        le=100000.0,
        description="刺激の周波数 (Hz)"
    )
    amplitude_v: float = Field(
        ...,
        ge=0.0,
        le=10.0,
        description="刺激の振幅 (V、片側)"
    )
    offset_v: float = Field(
        default=0.0,
        ge=-10.0,
        le=10.0,
        description="刺激のオフセット (V)"
    )
    channels: List[DaqAiName] = Field(
        ...,
        min_length=1,
        max_length=16,
        description="応答を取り込むアナログ入力のリスト（信号名、または ai0-ai15）"
    )
    rate_hz: float = Field(
        default=1000.0,
        gt=0.0,
        le=1250000.0,
        description="取り込みのサンプリング周波数 (Hz/ch)"
    )
    duration_s: float = Field(
        default=1.0,
        gt=0.0,
        le=3600.0,
        description="刺激と取り込みの時間 (秒)"
    )
    v_range: DaqVRange = Field(default=None, description=_V_RANGE_DOC)
    terminal: DaqTerminalConfig = Field(default=None, description=_TERMINAL_DOC)
    file_path: Optional[str] = Field(
        default=None,
        description="保存先 CSV（省略時は実行フォルダの data/ に自動で付ける）"
    )

    @model_validator(mode="after")
    def _within_device_limits(self):
        n = len(self.channels)
        if len({c.strip().lower() for c in self.channels}) != n:
            raise ValueError("daq_stimulate_acquire: channels に同じチャンネルが重複しています")
        if n > 1 and self.rate_hz * n > 1000000.0:
            raise ValueError("daq_stimulate_acquire: 複数チャンネルでは rate_hz × チャンネル数を "
                             "1 MHz 以下にしてください")
        points = self.rate_hz * self.duration_s
        if points < 2 or points * n > 2000000:
            raise ValueError("daq_stimulate_acquire: rate_hz × duration_s は 2 点以上、"
                             "全チャンネル合計 200 万点以下にしてください")
        return self


DaqPort0Line = Annotated[str, _daq_name("line")]
_PORT0_DOC = "（信号名、または port0/line0-port0/line7。ハードウェアタイミングは port0 のみ）"


class ActionDaqAcquireDigital(_StrictModel):
    """
    DAQ のデジタルライン（port0）の状態を一定周期で取り込み、CSV に保存する

    rate_hz × duration_s 点。CSV は time_s と、ラインごとの 0 / 1。clock を指定すると
    その端子のエッジごとに 1 点取り込む（外部クロック）。

    Note:
        共有デバイスのため、robot_idは不要です。
    """
    action: Literal["daq_acquire_digital"] = Field(
        ...,
        description="アクション識別子: DAQ デジタル波形取り込み"
    )
    lines: List[DaqPort0Line] = Field(
        ...,
        min_length=1,
        max_length=8,
        description="デジタルラインのリスト" + _PORT0_DOC
    )
    rate_hz: float = Field(
        default=1000.0,
        gt=0.0,
        le=1000000.0,
        description="サンプリング周波数 (Hz)"
    )
    duration_s: float = Field(
        default=1.0,
        gt=0.0,
        le=3600.0,
        description="取り込み時間 (秒)"
    )
    clock: Optional[DaqTerminalName] = Field(
        default=None,
        description="外部サンプルクロックの端子（信号名、または pfi0-pfi15）。省略時は内部クロック"
    )
    file_path: Optional[str] = Field(
        default=None,
        description="保存先 CSV（省略時は実行フォルダの data/ に自動で付ける）"
    )

    @model_validator(mode="after")
    def _within_limits(self):
        n = len(self.lines)
        if len({c.strip().lower() for c in self.lines}) != n:
            raise ValueError("daq_acquire_digital: lines に同じラインが重複しています")
        points = self.rate_hz * self.duration_s
        if points < 2 or points * n > 2000000:
            raise ValueError("daq_acquire_digital: rate_hz × duration_s は 2 点以上、"
                             "全ライン合計 200 万点以下にしてください")
        return self


class ActionDaqWriteDigitalPattern(_StrictModel):
    """
    DAQ のデジタルライン（port0）に、決めた順序の High / Low をハードウェアタイミングで出す

    pattern は 1 ステップ 1 行。各行は lines の順に 1 文字ずつ "1"（High）/ "0"（Low）
    （2 ライン・3 ステップなら ["10", "01", "00"]）。1 / rate_hz 秒ごとに次の行へ進み、
    出し終えるまでステップはブロックする。終わると全ラインが Low に戻る。
    record を指定すると、同じクロックでそのラインを取り込んで CSV に保存する（応答）。
    config.yaml の daq_do_lines に無いラインは実行時に拒否される。

    Note:
        共有デバイスのため、robot_idは不要です。
    """
    action: Literal["daq_write_digital_pattern"] = Field(
        ...,
        description="アクション識別子: DAQ デジタルパターン出力"
    )
    lines: List[DaqPort0Line] = Field(
        ...,
        min_length=1,
        max_length=8,
        description="出力するデジタルラインのリスト" + _PORT0_DOC
    )
    pattern: List[str] = Field(
        ...,
        min_length=2,
        max_length=100000,
        description="1 ステップ 1 行。各行は lines の順の '0' / '1'（例: [\"10\", \"01\", \"00\"]）"
    )
    rate_hz: float = Field(
        ...,
        gt=0.0,
        le=1000000.0,
        description="1 秒あたりのステップ数 (Hz)"
    )
    record: Optional[List[DaqPort0Line]] = Field(
        default=None,
        max_length=8,
        description="パターンと同じクロックで取り込むラインのリスト" + _PORT0_DOC
    )
    file_path: Optional[str] = Field(
        default=None,
        description="record の保存先 CSV（省略時は実行フォルダの data/ に自動で付ける）"
    )

    @model_validator(mode="after")
    def _pattern_matches_lines(self):
        n = len(self.lines)
        played = {c.strip().lower() for c in self.lines}
        if len(played) != n:
            raise ValueError("daq_write_digital_pattern: lines に同じラインが重複しています")
        for row in self.pattern:
            if len(row) != n or set(row) - {"0", "1"}:
                raise ValueError(f"daq_write_digital_pattern: pattern の行 {row!r} は "
                                 f"'0' / '1' を {n} 文字（lines の数）にしてください")
        if len(self.pattern) / self.rate_hz > 60.0:
            raise ValueError("daq_write_digital_pattern: パターンの長さ（行数 / rate_hz）は "
                             "60 秒以下にしてください")
        if self.record is not None:
            recorded = {c.strip().lower() for c in self.record}
            if not recorded or len(recorded) != len(self.record) or recorded & played:
                raise ValueError("daq_write_digital_pattern: record は重複なし・1 本以上で、"
                                 "lines と別のラインにしてください")
        return self


class ActionDaqWaitDigitalChange(_StrictModel):
    """
    DAQ のデジタルライン（port0）のどれかが変化するまで待つ（ハードウェアの変化検出）

    ポーリング（daq_wait_digital）では取りこぼす短いパルスも検出する。timeout_s 以内に
    変化が無ければステップは失敗する。--mock では変化があったものとして進む。

    Note:
        共有デバイスのため、robot_idは不要です。
    """
    action: Literal["daq_wait_digital_change"] = Field(
        ...,
        description="アクション識別子: DAQ デジタル変化待ち"
    )
    lines: List[DaqPort0Line] = Field(
        ...,
        min_length=1,
        max_length=8,
        description="見張るデジタルラインのリスト" + _PORT0_DOC
    )
    edge: Literal["rising", "falling", "both"] = Field(
        default="both",
        description="検出する変化: rising = Low→High / falling = High→Low / both"
    )
    timeout_s: float = Field(
        default=60.0,
        gt=0.0,
        le=86400.0,
        description="待つ上限 (秒)"
    )


_TIME_MIN_DOC = "予想される時間の下限 (秒)"
_TIME_MAX_DOC = "予想される時間の上限 (秒)"
_COUNTER_DOC = "使うカウンタ（信号名、または ctr0-ctr3）。省略時は空いているものを使う"


class _DaqTimeMeasurement(_StrictModel):
    min_s: float = Field(default=1e-6, gt=0.0, le=1000.0, description=_TIME_MIN_DOC)
    max_s: float = Field(default=1.0, gt=0.0, le=1000.0, description=_TIME_MAX_DOC)
    timeout_s: float = Field(default=5.0, gt=0.0, le=3600.0, description="信号を待つ上限 (秒)")
    counter: Optional[DaqCounterName] = Field(default=None, description=_COUNTER_DOC)

    @model_validator(mode="after")
    def _range_ordered(self):
        if not self.min_s < self.max_s:
            raise ValueError("min_s < max_s にしてください")
        return self


class ActionDaqMeasurePulseWidth(_DaqTimeMeasurement):
    """
    DAQ のカウンタで、端子に来るパルス 1 個の幅を測る

    結果は daq_value 列（daq_unit = s）。timeout_s 以内にパルスが来なければ失敗する。

    Note:
        共有デバイスのため、robot_idは不要です。
    """
    action: Literal["daq_measure_pulse_width"] = Field(
        ...,
        description="アクション識別子: DAQ パルス幅測定"
    )
    terminal: DaqTerminalName = Field(
        ...,
        description="入力端子（信号名、または pfi0-pfi15）"
    )
    level: Literal["high", "low"] = Field(
        default="high",
        description="測る部分: high = High の間 / low = Low の間"
    )


class ActionDaqMeasurePeriod(_DaqTimeMeasurement):
    """
    DAQ のカウンタで、端子の信号の周期を測る（1 周期）

    結果は daq_value 列（daq_unit = s）。timeout_s 以内に信号が来なければ失敗する。

    Note:
        共有デバイスのため、robot_idは不要です。
    """
    action: Literal["daq_measure_period"] = Field(
        ...,
        description="アクション識別子: DAQ 周期測定"
    )
    terminal: DaqTerminalName = Field(
        ...,
        description="入力端子（信号名、または pfi0-pfi15）"
    )
    edge: Literal["rising", "falling"] = Field(
        default="rising",
        description="周期の基準にするエッジ"
    )


class ActionDaqMeasureEdgeSeparation(_DaqTimeMeasurement):
    """
    DAQ のカウンタで、first のエッジから second の次のエッジまでの時間を測る

    2 つの信号の遅れ（応答時間、通過時間）を測るのに使う。結果は daq_value 列
    （daq_unit = s）。timeout_s 以内にエッジの組が来なければ失敗する。

    Note:
        共有デバイスのため、robot_idは不要です。
    """
    action: Literal["daq_measure_edge_separation"] = Field(
        ...,
        description="アクション識別子: DAQ 2 エッジ間隔測定"
    )
    first: DaqTerminalName = Field(
        ...,
        description="最初のエッジの端子（信号名、または pfi0-pfi15）"
    )
    second: DaqTerminalName = Field(
        ...,
        description="次のエッジの端子（信号名、または pfi0-pfi15。first と同じでもよい）"
    )
    first_edge: Literal["rising", "falling"] = Field(default="rising", description="最初のエッジ")
    second_edge: Literal["rising", "falling"] = Field(default="rising", description="次のエッジ")


class ActionDaqStartEncoder(_StrictModel):
    """
    DAQ のカウンタで、ロータリーエンコーダ（A / B の 2 相）の追従を始める（0° から）

    以後、daq_read_encoder で回った角度を読む。同時に追従できるのは 1 個。
    daq_stop_encoder か実行終了まで続く。

    Note:
        共有デバイスのため、robot_idは不要です。
    """
    action: Literal["daq_start_encoder"] = Field(
        ...,
        description="アクション識別子: DAQ エンコーダ追従開始"
    )
    a: DaqTerminalName = Field(..., description="A 相の端子（信号名、または pfi0-pfi15）")
    b: DaqTerminalName = Field(..., description="B 相の端子（信号名、または pfi0-pfi15）")
    pulses_per_rev: int = Field(
        ...,
        ge=1,
        le=10000000,
        description="1 回転あたりのパルス数（エンコーダの仕様値）"
    )
    decoding: Literal["x1", "x2", "x4"] = Field(
        default="x4",
        description="逓倍（x4 が最も分解能が高い）"
    )
    counter: Optional[DaqCounterName] = Field(default=None, description=_COUNTER_DOC)

    @model_validator(mode="after")
    def _two_terminals(self):
        if self.a.strip().lower() == self.b.strip().lower():
            raise ValueError("daq_start_encoder: a と b は別の端子にしてください")
        return self


class ActionDaqReadEncoder(_StrictModel):
    """
    追従中のエンコーダの角度を読む（daq_start_encoder からの回転、度。逆回転は負）

    結果は daq_value 列（daq_unit = deg）。

    Note:
        共有デバイスのため、robot_idは不要です。
    """
    action: Literal["daq_read_encoder"] = Field(
        ...,
        description="アクション識別子: DAQ エンコーダ読み取り"
    )


class ActionDaqStopEncoder(_StrictModel):
    """
    エンコーダの追従を終える（最後の角度を daq_value 列に残す）

    Note:
        共有デバイスのため、robot_idは不要です。
    """
    action: Literal["daq_stop_encoder"] = Field(
        ...,
        description="アクション識別子: DAQ エンコーダ追従終了"
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
    # DAQ（NI USB-6351）操作
    ActionDaqReadAnalog,
    ActionDaqAcquire,
    ActionDaqStartLogging,
    ActionDaqStopLogging,
    ActionDaqWaitAnalog,
    ActionDaqWriteAnalog,
    ActionDaqGenerateWaveform,
    ActionDaqStopOutput,
    ActionDaqReadDigital,
    ActionDaqWriteDigital,
    ActionDaqPulseDigital,
    ActionDaqWaitDigital,
    ActionDaqCountEdges,
    ActionDaqMeasureFrequency,
    ActionDaqGeneratePulses,
    ActionDaqStimulateAcquire,
    ActionDaqAcquireDigital,
    ActionDaqWriteDigitalPattern,
    ActionDaqWaitDigitalChange,
    ActionDaqMeasurePulseWidth,
    ActionDaqMeasurePeriod,
    ActionDaqMeasureEdgeSeparation,
    ActionDaqStartEncoder,
    ActionDaqReadEncoder,
    ActionDaqStopEncoder,
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