"""
Keithley 2002 8.5 桁デジタルマルチメータ（GPIB / SCPI）のドライバ

2002 のインターフェースは IEEE-488（GPIB）だけで、PC 側の GPIB アダプタと
VISA ライブラリ（NI-VISA など）を通して PyVISA で話す。リソース文字列は
``GPIB0::16::INSTR`` の形（16 は工場出荷時のアドレス。前面 MENU > GPIB で確認）。

測定 1 回分の手順（コマンドは 2002 取扱説明書 2002-900-01 Rev. E で確認）:

- ``:ABORt`` / ``:CONFigure:<機能>`` -- 単発測定の既知状態にする。説明書 p.3-42:
  トリガモデル 3 層すべてソース IMMediate・カウント 1・遅延 0、連続イニシエート OFF、
  演算・バッファ・スキャン無効、取得方式 normal、オートゼロとその機能の設定
  （フィルタ・REL・桁数など）は *RST 値。``*RST`` と違い、この一覧の外の設定
  （GPIB アドレス、表示、ライン同期など）は変えない
- ``:SENSe:<機能>:RANGe:AUTO ON`` または ``:RANGe <値>``、``:NPLCycles <n>``
  （AC 電圧・AC 電流も含めて 6 機能すべて 0.01〜50。説明書 Table 3-10）
- AC 電圧のみ ``:UNIT:VOLTage:AC V``（dB / dBm 表示のままだと単位が変わる）
- ``:FORMat:DATA ASCii`` / ``:FORMat:ELEMents READing`` /
  ``:FORMat:EXPonent HPRecision``（応答を数値だけにし、レンジの工学単位に
  丸めず、出力時の桁落ちを抑える。校正精度とは別。説明書 p.3-81, 3-83）
- ``:SYSTem:ERRor?`` を ``0`` が返るまで読み、エラーが 1 件でもあれば例外にする
  （write が通っても機器が設定を受け付けたとは限らない）
- 測定後に ``:SENSe:<機能>:RANGe?`` / ``:NPLCycles?`` で実際の設定を読み返す
  （オートレンジでは最後のサンプルで使われたレンジ）。I/O エラーは握りつぶさない
  （応答の順序がずれたまま次の問い合わせに進むと、設定値を測定値と取り違える）

``:CONFigure`` が変えないもの: GPIB・表示などの機器全体の設定と、ライン同期
（``:SYSTem:LSYNc:STATe``、説明書 p.3-143）。ライン同期は ``line_sync()`` で
読み、初期化時にログへ残す。
- ``:READ?`` = ``:ABORt`` + ``:INITiate`` + ``:FETCh?``。単発状態なので 1 回の
  変換を起こしてその値を返す（説明書 p.3-42）。オーバーフローは ``+9.9E37``

ドライバは実験・フロー・ロボットについて何も知らない。非同期化・中断・
排他は SharedDevices（src/devices/safety/shared_devices.py）が受け持つ。

使用例:
    from src.devices.keithley2002 import Keithley2002

    dmm = Keithley2002("GPIB0::16::INSTR")
    try:
        dmm.clear_status()
        print(dmm.identify())                         # 'KEITHLEY INSTRUMENTS INC.,MODEL 2002,...'
        dmm.configure("dc_voltage", nplc=10)
        print(dmm.read())                             # 例: 1.234567891
        print(dmm.read_settings("dc_voltage"))        # {'range': 2.0, 'nplc': 10.0}
    finally:
        try:
            dmm.local()                               # 前面パネル操作に戻す
        finally:
            dmm.close()                               # GTL が失敗しても閉じる
"""
import logging
import math
import re
from typing import Optional

logger = logging.getLogger(__name__)

#: フローで使う測定機能名 -> (SCPI の機能名, 単位, 最大レンジ)。
#: 最大レンジは説明書 Table 3-10 の ``RANGe[:UPPer]`` の上限。
FUNCTIONS = {
    "dc_voltage":    ("VOLTage:DC",  "V",   1100.0),
    "ac_voltage":    ("VOLTage:AC",  "V",   775.0),
    "dc_current":    ("CURRent:DC",  "A",   2.1),
    "ac_current":    ("CURRent:AC",  "A",   2.1),
    "resistance_2w": ("RESistance",  "Ohm", 1.05e9),
    "resistance_4w": ("FRESistance", "Ohm", 2.1e6),
}
#: NPLC（積分時間、電源周期の何倍か）の範囲。全機能共通（説明書 Table 3-10）
NPLC_MIN = 0.01
NPLC_MAX = 50.0
#: READing 要素のオーバーフロー値（説明書 p.3-81）。これ以上の大きさは無効値として扱う
OVERFLOW_THRESHOLD = 9.9e37
#: 工場出荷時の GPIB アドレス
DEFAULT_GPIB_ADDRESS = 16

_NUMBER = re.compile(r"^\s*([-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?)")
_MODEL = re.compile(r"\b2002\b")
_MAX_ERROR_READS = 11   # エラーキューは 10 件 + "Queue Overflow"


class Keithley2002Error(RuntimeError):
    """機器がエラーを報告した、または応答を解釈できなかった"""


class Keithley2002Aborted(Keithley2002Error):
    """打ち切り要求（``abort_check``）により、次の I/O を送らずに止めた"""


def parse_reading(text) -> Optional[float]:
    """``:READ?`` の応答を float にする。読めない・オーバーフロー・非有限なら None

    ``FORMat:ELEMents READing`` なら応答は ``+1.23456789E+00`` だけだが、前面で
    要素を足された場合（``+1.23E+00VDC,+12.3SECS,...``）も先頭の数値を取る。
    """
    if not text:
        return None
    match = _NUMBER.match(str(text).split(",")[0])
    if not match:
        return None
    value = float(match.group(1))
    if not math.isfinite(value) or abs(value) >= OVERFLOW_THRESHOLD:
        return None
    return value


def parse_error(reply: str):
    """``:SYSTem:ERRor?`` の応答 ``-113,"Undefined header"`` を (コード, 全文) にする"""
    head = str(reply).split(",", 1)[0].strip()
    try:
        return int(head), str(reply).strip()
    except ValueError:
        raise Keithley2002Error(f":SYST:ERR? の応答を解釈できません: {reply!r}")


def _gtl_mode():
    """GPIB の Go To Local（GTL）を指す PyVISA の定数"""
    try:
        from pyvisa.constants import RENLineOperation
        return RENLineOperation.address_gtl
    except ImportError:          # テスト用の偽リソースのみ（実機は PyVISA が必須）
        return 6                 # VI_GPIB_REN_ADDRESS_GTL


class Keithley2002:
    """Keithley 2002 を PyVISA で操作する薄いドライバ

    Args:
        resource: VISA リソース文字列（例 ``GPIB0::16::INSTR``）。既定値は持たない
        timeout_ms: 1 回の I/O の上限（ミリ秒）。NPLC が大きい・オートレンジ・AC は
            1 読み取りに数秒かかることがある
        resource_manager: テスト用の差し替え。省略時は ``pyvisa.ResourceManager()``
    """

    def __init__(self, resource: str, timeout_ms: int = 10000, resource_manager=None):
        if not resource:
            raise ValueError("VISA リソース文字列が空です（例: GPIB0::16::INSTR）")
        if resource_manager is None:
            import pyvisa            # 実機を使うときだけ必要（Mock / テストでは不要）
            resource_manager = pyvisa.ResourceManager()
        self.resource = resource
        #: 呼び出し側が設定する打ち切り判定（引数なし、True で打ち切り）。write / query の
        #: 直前に見るので、進行中の 1 回の I/O が終わった次の境界で止まる
        self.abort_check = None
        self._inst = resource_manager.open_resource(resource)
        try:
            self._inst.timeout = int(timeout_ms)
            self._inst.read_termination = "\n"
            self._inst.write_termination = "\n"
        except BaseException:
            self._inst.close()
            raise

    # ------------------------------------------------------------------
    # 低レベル I/O
    # ------------------------------------------------------------------
    def _check_abort(self, command: str):
        if self.abort_check is not None and self.abort_check():
            raise Keithley2002Aborted(f"打ち切り要求のため送信しません: {command}")

    def write(self, command: str):
        self._check_abort(command)
        logger.debug(f"2002 <- {command}")
        self._inst.write(command)

    def query(self, command: str) -> str:
        self._check_abort(command)
        reply = self._inst.query(command)
        logger.debug(f"2002 <- {command} -> {reply!r}")
        return reply

    # ------------------------------------------------------------------
    # 機器レベルの操作
    # ------------------------------------------------------------------
    def clear_status(self):
        """デバイスクリア（読み残しの応答を捨てる）と ``*CLS``（エラーキューを空にする）"""
        self._inst.clear()
        self.write("*CLS")

    def identify(self) -> str:
        """``*IDN?`` を返す。2 番目の欄が 2002 でなければ例外"""
        idn = self.query("*IDN?").strip()
        fields = idn.split(",")
        if len(fields) < 2 or not _MODEL.search(fields[1]):
            raise Keithley2002Error(f"Keithley 2002 ではありません: *IDN? = {idn!r}")
        return idn

    def check_errors(self, context: str = ""):
        """エラーキューを空になるまで読み、1 件でもあれば Keithley2002Error"""
        errors = []
        for _ in range(_MAX_ERROR_READS):
            code, text = parse_error(self.query(":SYST:ERR?"))
            if code == 0:
                break
            errors.append(text)
        if errors:
            where = f"{context}: " if context else ""
            raise Keithley2002Error(f"{where}機器がエラーを報告しました: {'; '.join(errors)}")

    def configure(self, function: str = "dc_voltage", range_: Optional[float] = None,
                  nplc: float = 1.0) -> dict:
        """単発測定の状態にして機能・レンジ・NPLC を設定する

        Args:
            function: :data:`FUNCTIONS` のキー
            range_: 固定レンジ（その機能の単位）。None ならオートレンジ
            nplc: 積分時間（電源周期の倍数、0.01〜50）

        Returns:
            dict: ``{"function", "unit", "auto_range"}``。機器が実際に使ったレンジと
            NPLC は測定の後で :meth:`read_settings` で読み返す

        Raises:
            ValueError: 引数が範囲外
            Keithley2002Error: 機器が設定を受け付けなかった
        """
        if function not in FUNCTIONS:
            raise ValueError(f"未知の測定機能: {function!r}（{', '.join(FUNCTIONS)}）")
        scpi, unit, max_range = FUNCTIONS[function]
        if not (NPLC_MIN <= float(nplc) <= NPLC_MAX):
            raise ValueError(f"nplc は {NPLC_MIN}〜{NPLC_MAX}: {nplc}")
        if range_ is not None and not (0 < float(range_) <= max_range):
            raise ValueError(f"{function} のレンジは 0 より大きく {max_range:g} {unit} 以下: {range_}")

        self.write(":ABOR")
        self.write(f":CONF:{scpi}")
        if range_ is None:
            self.write(f":SENS:{scpi}:RANG:AUTO ON")
        else:
            self.write(f":SENS:{scpi}:RANG {float(range_):g}")
        self.write(f":SENS:{scpi}:NPLC {float(nplc):g}")
        if function == "ac_voltage":
            self.write(":UNIT:VOLT:AC V")
        self.write(":FORM:DATA ASC")
        self.write(":FORM:ELEM READ")
        self.write(":FORM:EXP HPR")
        self.check_errors(f"{function} の設定")
        return {"function": function, "unit": unit, "auto_range": range_ is None}

    def read_settings(self, function: str) -> dict:
        """実際に使われているレンジと NPLC を読み返す（記録用）

        I/O エラー（タイムアウトなど）は送出する。応答の順序がずれたまま次へ進むと、
        後の問い合わせが前の応答を受け取り、設定値を測定値として採用しかねない。
        応答が数値として読めないだけなら None。
        """
        scpi = FUNCTIONS[function][0]
        return {"range": parse_reading(self.query(f":SENS:{scpi}:RANG?")),
                "nplc": parse_reading(self.query(f":SENS:{scpi}:NPLC?"))}

    def line_sync(self) -> bool:
        """ライン同期（``:SYSTem:LSYNc:STATe``）の状態。:CONFigure では変わらない"""
        reply = self.query(":SYST:LSYN:STAT?").strip()
        if reply not in ("0", "1"):
            raise Keithley2002Error(f":SYST:LSYN:STAT? の応答を解釈できません: {reply!r}")
        return reply == "1"

    def read(self) -> Optional[float]:
        """1 回測定して値を返す。オーバーフロー・解釈不能なら None（警告ログ）"""
        reply = self.query(":READ?")
        value = parse_reading(reply)
        if value is None:
            logger.warning(f"Keithley 2002 の応答を測定値として使えません"
                           f"（オーバーフロー or 解釈不能）: {reply!r}")
        return value

    # ------------------------------------------------------------------
    # 終了処理
    # ------------------------------------------------------------------
    def local(self):
        """前面パネル操作に戻す（GPIB の GTL）。設定は戻さない"""
        self._inst.control_ren(_gtl_mode())

    def close(self):
        self._inst.close()


if __name__ == "__main__":
    # 接続確認（人が実行する）: *IDN? と DC 電圧 1 回。
    #   python -m src.devices.keithley2002.keithley2002 GPIB0::16::INSTR
    import sys

    logging.basicConfig(level=logging.INFO)
    if len(sys.argv) != 2:
        print("usage: python -m src.devices.keithley2002.keithley2002 <VISA resource>")
        sys.exit(2)
    dmm = Keithley2002(sys.argv[1])
    try:
        dmm.clear_status()
        print(dmm.identify())
        print(f"line sync: {'ON' if dmm.line_sync() else 'OFF'}")
        dmm.configure("dc_voltage")
        print(f"DC voltage: {dmm.read()} V")
        print(dmm.read_settings("dc_voltage"))
    finally:
        try:
            dmm.local()
        finally:
            dmm.close()
