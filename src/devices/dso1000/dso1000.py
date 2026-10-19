"""
Agilent 1000 シリーズ（DSO1014A など）デジタルオシロスコープの読み取り専用ドライバ

背面の USB DEVICE ポート（USBTMC、USB 2.0 full speed）で、VISA ライブラリ
（NI-VISA など）を通して PyVISA で話す（プログラマーズガイド p.21）。
リソース文字列は ``USB0::0x0957::<製品 ID>::<シリアル>::INSTR`` の形で、
シリアル番号を含むので config.yaml にだけ書く。

コマンドは "Agilent 1000 Series Oscilloscopes Programmer's Guide"
（版 011.04.00.04.06、2010 年 4 月）で確認した。以下の p.n はその頁。

方針: 垂直・水平・トリガ・取り込みの設定は前面パネルのもので、SAIGEN は
問い合わせしか送らない。設定を書き込む形のコマンド（``*RST``、``:AUToscale``、
``:RUN`` / ``:STOP`` / ``:SINGLE``、``:CHANnel`` / ``:TIMebase`` / ``:TRIGger`` /
``:ACQuire`` / ``:MEASure`` などの設定形）は送らない。副作用は接続・切断時の
デバイスクリア（入出力バッファを消す）と接続時の ``*CLS``（状態とエラーキューを消す、
p.60）だけ。

自動測定（p.157-191）は ``:MEASure:<項目>? CHANnel<n>`` で読む（チャネルは常に
明示する）。応答は文字列として扱う:

- デューティ比・オーバーシュート・プリシュートはガイドでは ``%`` を含む（p.168, 172,
  174, 178）が、DSO1014A（ファームウェア 00.04.04）は ``5.000e001`` のように ``%`` なしで
  返した。末尾の ``%`` があれば 1 つ外して読み、単位を % とする（100 倍しない）
- 立ち上がり・立ち下がり時間などは ``<`` を含むことがある（p.165, p.181）。
  境界値で測定値ではないので捨てる
- 測れないとき（チャネル非表示など）は 9.9E+37 が返る。ガイドではカウンタの頁
  （p.103）にしか書かれていないが、DSO1014A（ファームウェア 00.04.04）で
  :MEASure でも確かめた

ドライバは実験・フロー・ロボットについて何も知らない。非同期化・中断・
排他は SharedDevices（src/devices/safety/shared_devices.py）が受け持つ。
"""
import logging
import math
import re
from typing import Optional

logger = logging.getLogger(__name__)

#: 対応機種とチャネル数（ガイド p.3）
MODELS = {"DSO1002A": 2, "DSO1004A": 4, "DSO1012A": 2, "DSO1014A": 4,
          "DSO1022A": 2, "DSO1024A": 4}

#: フローの項目名 -> (:MEASure のヘッダー, 単位の種類)。"volt" はチャネルの UNITs に従う
ITEMS = {
    "vpp":        ("VPP",        "volt"),
    "vmax":       ("VMAX",       "volt"),
    "vmin":       ("VMIN",       "volt"),
    "vamplitude": ("VAMPlitude", "volt"),
    "vtop":       ("VTOP",       "volt"),
    "vbase":      ("VBASe",      "volt"),
    "vaverage":   ("VAVerage",   "volt"),
    "vrms":       ("VRMS",       "volt"),
    "frequency":  ("FREQuency",  "Hz"),
    "period":     ("PERiod",     "s"),
    "risetime":   ("RISetime",   "s"),
    "falltime":   ("FALLtime",   "s"),
    "pwidth":     ("PWIDth",     "s"),
    "nwidth":     ("NWIDth",     "s"),
    "pdutycycle": ("PDUTycycle", "%"),
    "ndutycycle": ("NDUTycycle", "%"),
    "overshoot":  ("OVERshoot",  "%"),
    "preshoot":   ("PREShoot",   "%"),
}

#: 測れない値として扱う大きさ（p.103。:MEASure でも実機で確認）
NO_MEASUREMENT = 9.9e37

#: :CHANnel<n>:UNITs? -> 単位（p.98）
#: :CHANnel<n>:UNITs? の応答（長形・短形、大文字で照合）-> 単位。実機は "VOLTs" を返した
_UNITS = {"VOLTS": "V", "VOLT": "V", "AMPERES": "A", "AMP": "A", "WATTS": "W", "WATT": "W",
          "UNKNOWN": "", "UNKN": ""}
_ON_OFF = {"1": 1, "0": 0, "ON": 1, "OFF": 0}
_PROBES = ("0.001X", "0.01X", "0.1X", "1X", "10X", "100X", "1000X")
_MAX_ERROR_READS = 31   # キューの深さは p.208 で 30、p.301 で 10 と食い違う。30 + 終端の 0


class DSO1000Error(RuntimeError):
    """機器がエラーを報告した、または応答を解釈できなかった"""


class DSO1000Aborted(DSO1000Error):
    """打ち切り要求（``abort_check``）により、次の I/O を送らずに止めた"""


class ScopeStepError(DSO1000Error):
    """測定ステップの失敗。``diagnostics`` に記録用の構造化データを持つ"""

    def __init__(self, message: str, diagnostics: Optional[dict] = None):
        super().__init__(message)
        self.diagnostics = diagnostics or {}


# ----------------------------------------------------------------------
# 応答の解釈（純粋関数）
# ----------------------------------------------------------------------
def parse_measurement(reply, item: str) -> tuple:
    """自動測定の応答を (値, 捨てる理由) にする。使える値なら理由は None

    捨てる理由: ``bound``（``<`` / ``>`` で始まる境界値）、``unparsable``、
    ``non_finite``（NaN / Inf）、``no_measurement``（|値| >= 9.9e37）。
    ``%`` は % の 4 項目だけ末尾の 1 つを外す（ほかの位置・ほかの項目は unparsable）。
    """
    text = str(reply).strip()
    if text.startswith(("<", ">")):
        return None, "bound"
    if ITEMS[item][1] == "%" and text.endswith("%"):
        text = text[:-1].strip()
    try:
        value = float(text)
    except ValueError:
        return None, "unparsable"
    if not math.isfinite(value):
        return None, "non_finite"
    if abs(value) >= NO_MEASUREMENT:
        return None, "no_measurement"
    return value, None


def parse_error(reply: str):
    """``:SYSTem:ERRor?`` の応答 ``63,"Undefined header"`` を (番号, 全文) にする"""
    head = str(reply).split(",", 1)[0].strip()
    try:
        return int(head), str(reply).strip()
    except ValueError:
        raise DSO1000Error(f":SYSTem:ERRor? の応答を解釈できません: {reply!r}")


def parse_float(reply, what: str, positive: bool = False, low: float = None,
                high: float = None) -> float:
    """実数の応答。非有限・範囲外（low / high は両端を含み、相対 1e-6 の余裕を持つ）は
    DSO1000Error"""
    try:
        value = float(str(reply).strip())
    except ValueError:
        raise DSO1000Error(f"{what} の応答を数値として読めません: {reply!r}")
    slack = 1e-6
    if (not math.isfinite(value) or (positive and value <= 0)
            or (low is not None and value < low * (1 - slack))
            or (high is not None and value > high * (1 + slack))):
        raise DSO1000Error(f"{what} の応答が範囲外です: {reply!r}")
    return value


def parse_choice(reply, choices, what: str) -> str:
    """列挙値の応答を大文字で照合する。知らない値は DSO1000Error"""
    text = str(reply).strip().upper()
    if text not in choices:
        raise DSO1000Error(f"{what} の応答が想定外です（{', '.join(choices)}）: {reply!r}")
    return text


def parse_on_off(reply, what: str) -> int:
    """ガイドの 1 / 0（ON / OFF も受け付ける）"""
    text = str(reply).strip().upper()
    if text not in _ON_OFF:
        raise DSO1000Error(f"{what} の応答が想定外です（1 / 0）: {reply!r}")
    return _ON_OFF[text]


def parse_units(reply, what: str) -> str:
    text = str(reply).strip().upper()
    if text in _UNITS:
        return _UNITS[text]
    raise DSO1000Error(f"{what} の応答が想定外です（VOLTs / AMPeres / WATTs / UNKNown）: {reply!r}")


def parse_timebase_format(reply) -> str:
    """``:TIMebase:FORMat?`` は ``Y-T | X-Y | ROLL`` を返す（設定の引数は YT | XY | ROLL、
    p.214）。どちらの形も受け付けて YT / XY / ROLL にする"""
    text = str(reply).strip().upper().replace("-", "")
    if text not in ("YT", "XY", "ROLL"):
        raise DSO1000Error(f":TIMebase:FORMat? の応答が想定外です: {reply!r}")
    return text


def parse_model(idn: str) -> tuple:
    """``*IDN?`` から (機種, チャネル数)。対応表にない機種は DSO1000Error"""
    fields = [f.strip() for f in str(idn).split(",")]
    model = fields[1].upper() if len(fields) > 1 else ""
    if model not in MODELS:
        raise DSO1000Error(f"対応していない機種です（{', '.join(MODELS)}）: *IDN? = {idn!r}")
    return model, MODELS[model]


#: 測定の前後で比べない項目（自然に変わる、または今回は記録だけするもの）
VOLATILE_SETTINGS = ("status", "srate", "meas_source")


def item_unit(item: str, channel_unit: str) -> str:
    kind = ITEMS[item][1]
    return channel_unit if kind == "volt" else kind


class DSO1000:
    """Agilent 1000 シリーズのオシロスコープを PyVISA で読む薄いドライバ

    Args:
        resource: VISA リソース文字列（``USB0::0x0957::...::INSTR``）。既定値は持たない
        timeout_ms: 1 回の I/O の上限（ミリ秒）
        resource_manager: テスト用の差し替え。省略時は ``pyvisa.ResourceManager()``
    """

    def __init__(self, resource: str, timeout_ms: int = 5000, resource_manager=None):
        if not resource:
            raise ValueError("VISA リソース文字列が空です（例: USB0::0x0957::...::INSTR）")
        if resource_manager is None:
            import pyvisa            # 実機を使うときだけ必要（Mock / テストでは不要）
            resource_manager = pyvisa.ResourceManager()
        self.resource = resource
        #: 通常の write / query の直前に見る打ち切り判定。終了処理（:meth:`release`）は見ない
        self.abort_check = None
        self.model = None
        self.channels = None
        self._inst = resource_manager.open_resource(resource)
        try:
            self._inst.timeout = int(timeout_ms)
            self._inst.read_termination = "\n"     # <NL> = LF（p.57）
            self._inst.write_termination = "\n"
        except BaseException:
            self._inst.close()
            raise

    # ------------------------------------------------------------------
    # 低レベル I/O
    # ------------------------------------------------------------------
    def _check_abort(self, command: str):
        if self.abort_check is not None and self.abort_check():
            raise DSO1000Aborted(f"打ち切り要求のため送信しません: {command}")

    def write(self, command: str):
        self._check_abort(command)
        logger.debug(f"DSO <- {command}")
        self._inst.write(command)

    def query(self, command: str) -> str:
        self._check_abort(command)
        reply = str(self._inst.query(command)).strip("\r\n")
        logger.debug(f"DSO <- {command} -> {reply!r}")
        return reply

    # ------------------------------------------------------------------
    # 機器レベルの操作
    # ------------------------------------------------------------------
    def clear_status(self):
        """デバイスクリア（入出力バッファを消す）と ``*CLS``（状態とエラーキュー）"""
        self._inst.clear()
        self.write("*CLS")

    def identify(self) -> str:
        """``*IDN?`` を読み、対応機種かを確かめてチャネル数を覚える"""
        idn = self.query("*IDN?").strip()
        self.model, self.channels = parse_model(idn)
        return idn

    def check_errors(self, context: str = "", into: list = None):
        """エラーキューを ``0`` まで読み（最大 31 回）、1 件でもあれば DSO1000Error

        ``into`` を渡すと読んだエラーをそこへ順に足す（途中の I/O エラーでも残る）
        """
        errors = into if into is not None else []
        for _ in range(_MAX_ERROR_READS):
            code, text = parse_error(self.query(":SYSTem:ERRor?"))
            if code == 0:
                break
            errors.append(text)
        else:
            errors.append(f"{_MAX_ERROR_READS} 回読んでもエラーキューが空になりません")
        if errors:
            where = f"{context}: " if context else ""
            raise DSO1000Error(f"{where}機器がエラーを報告しました: {'; '.join(errors)}")

    def timebase_format(self) -> str:
        return parse_timebase_format(self.query(":TIMebase:FORMat?"))

    def snapshot(self, channel: int, into: dict = None) -> dict:
        """値の意味に関わる設定を読む（問い合わせだけ）

        トリガは EDGE のときだけエッジの設定を読む（ほかのモードはステップ側で拒否する）。
        読めない応答は DSO1000Error。I/O エラーはそのまま送出する。``into`` を渡すと
        読めた設定をそこへ順に書く（途中で失敗しても、それまでの値が残る）
        """
        q = self.query
        c = f":CHANnel{int(channel)}"
        s = into if into is not None else {}
        s["tb_format"] = self.timebase_format()
        s["tb_mode"] = parse_choice(q(":TIMebase:MODE?"), ("MAIN", "DELAYED"), ":TIMebase:MODE?")
        # 範囲はガイドのとおり: 時間軸 1 ns〜50 s/div（p.216）、V/div 2 mV（1X）〜10 kV
        # （1000X、p.97）、ホールドオフ 100 ns〜1.5 s（p.224）、感度 0.1〜1 div（p.226）
        s["tb_scale"] = parse_float(q(":TIMebase:SCALe?"), ":TIMebase:SCALe?", low=1e-9, high=50.0)
        s["tb_offset"] = parse_float(q(":TIMebase:OFFSet?"), ":TIMebase:OFFSet?")
        s["ch_display"] = parse_on_off(q(f"{c}:DISPlay?"), f"{c}:DISPlay?")
        s["ch_scale"] = parse_float(q(f"{c}:SCALe?"), f"{c}:SCALe?", low=2e-3 * 0.001, high=1e4)
        s["ch_offset"] = parse_float(q(f"{c}:OFFSet?"), f"{c}:OFFSet?")
        s["ch_probe"] = parse_choice(q(f"{c}:PROBe?"), _PROBES, f"{c}:PROBe?")
        s["ch_coupling"] = parse_choice(q(f"{c}:COUPling?"), ("AC", "DC", "GND"), f"{c}:COUPling?")
        for key, header in (("ch_bwlimit", "BWLimit"), ("ch_filter", "FILTer"),
                            ("ch_invert", "INVert"), ("ch_vernier", "VERNier")):
            s[key] = parse_on_off(q(f"{c}:{header}?"), f"{c}:{header}?")
        s["ch_units"] = parse_units(q(f"{c}:UNITs?"), f"{c}:UNITs?")
        s["acq_type"] = parse_choice(q(":ACQuire:TYPE?"), ("NORMAL", "AVERAGE", "PEAK"),
                                     ":ACQuire:TYPE?")
        s["acq_averages"] = int(parse_choice(q(":ACQuire:AVERages?"),
                                             ("2", "4", "8", "16", "32", "64", "128", "256"),
                                             ":ACQuire:AVERages?"))
        s["trig_mode"] = parse_choice(q(":TRIGger:MODE?"),
                                      ("EDGE", "PULSE", "VIDEO", "PATTERN", "ALTERNATION"),
                                      ":TRIGger:MODE?")
        s["trig_coupling"] = parse_choice(q(":TRIGger:COUPling?"), ("DC", "AC", "LF"),
                                          ":TRIGger:COUPling?")
        s["trig_holdoff"] = parse_float(q(":TRIGger:HOLDoff?"), ":TRIGger:HOLDoff?",
                                        low=100e-9, high=1.5)
        s["trig_sensitivity"] = parse_float(q(":TRIGger:SENSitivity?"), ":TRIGger:SENSitivity?",
                                            low=0.1, high=1.0)
        s["trig_hfreject"] = parse_on_off(q(":TRIGger:HFREject?"), ":TRIGger:HFREject?")
        if s["trig_mode"] == "EDGE":
            s["edge_source"] = parse_choice(
                q(":TRIGger:EDGE:SOURce?"),
                ("CH1", "CH2", "CH3", "CH4", "EXT", "EXT5", "ACLINE"), ":TRIGger:EDGE:SOURce?")
            s["edge_level"] = parse_float(q(":TRIGger:EDGE:LEVel?"), ":TRIGger:EDGE:LEVel?")
            s["edge_slope"] = parse_choice(q(":TRIGger:EDGE:SLOPe?"),
                                           ("NEGATIVE", "POSITIVE", "ALTERNATION"),
                                           ":TRIGger:EDGE:SLOPe?")
            s["edge_sweep"] = parse_choice(q(":TRIGger:EDGE:SWEep?"), ("AUTO", "NORMAL", "SINGLE"),
                                           ":TRIGger:EDGE:SWEep?")
        s["meas_lock"] = parse_choice(q(":MEASure?"), ("UNLOCKED", "LOCKED"), ":MEASure?")
        # ガイド p.135 は ENAB | DIS だが、DSO1014A（ファームウェア 00.04.04）は ENABLE を返した
        s["key_lock"] = parse_choice(q(":KEY:LOCK?"), ("ENAB", "DIS", "ENABLE", "DISABLE"),
                                     ":KEY:LOCK?")[:4].replace("DISA", "DIS")
        # 以下は記録だけ（前後で比べない: VOLATILE_SETTINGS）
        s["meas_source"] = q(":MEASure:SOURce?").strip()
        s["status"] = parse_choice(q(":TRIGger:STATus?"),
                                   ("RUN", "STOP", "T'D", "WAIT", "SCAN", "AUTO"),
                                   ":TRIGger:STATus?")
        s["srate"] = parse_float(q(":ACQuire:SRATe?"), ":ACQuire:SRATe?")
        return s

    def measure(self, item: str, channel: int) -> str:
        """``:MEASure:<項目>? CHANnel<n>`` の応答（文字列のまま）"""
        return self.query(f":MEASure:{ITEMS[item][0]}? CHANnel{int(channel)}")

    # ------------------------------------------------------------------
    # 終了処理（abort_check を見ない）
    # ------------------------------------------------------------------
    def release(self) -> bool:
        """デバイスクリア -> 切断。クリアが失敗しても閉じる。close() の失敗は送出

        GTL は送らない: DSO1014A（ファームウェア 00.04.04）と NI-VISA の USB では
        ``control_ren`` が 120 秒止まった末に VI_ERROR_SYSTEM_ERROR になった。
        問い合わせの接続の合間に前面パネルは操作できた（:KEY:LOCK? = ENABLE は「前面パネル
        有効」）。ただし GTL が止まった後に一度ロックされていた（原因は未確定）。
        """
        try:
            self._inst.clear()
        except Exception as e:  # noqa: BLE001 - 後続を優先
            logger.warning(f"オシロスコープのデバイスクリアに失敗しました: {e}")
        self._inst.close()
        return True


if __name__ == "__main__":
    # 接続確認（人が実行する。接続・切断時のデバイスクリアと接続時の *CLS のほかは問い合わせだけ）:
    #   python -m src.devices.dso1000.dso1000 USB0::0x0957::...::INSTR [channel]
    import json
    import sys

    logging.basicConfig(level=logging.INFO)
    if len(sys.argv) not in (2, 3):
        print("usage: python -m src.devices.dso1000.dso1000 <VISA resource> [channel]")
        sys.exit(2)
    ch = int(sys.argv[2]) if len(sys.argv) == 3 else 1
    dso = DSO1000(sys.argv[1])
    try:
        dso.clear_status()
        print(dso.identify(), f"-> {dso.model}, {dso.channels} channels")
        dso.check_errors("接続確認")
        print(json.dumps(dso.snapshot(ch)))
        for name in ("vpp", "frequency", "pdutycycle"):
            raw = dso.measure(name, ch)
            print(f"{name}: {raw!r} -> {parse_measurement(raw, name)}")
        print(f":MEASure:SOURce? after: {dso.query(':MEASure:SOURce?')!r}")
        dso.check_errors("測定の後")
    finally:
        dso.release()
