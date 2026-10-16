"""
NF LI5640 マルチファンクション・デジタル・ロックインアンプ（GPIB）の読み取り専用ドライバ

LI5640 は GPIB（IEEE 488.1、488.2 一部準拠）か RS-232 のどちらか一方で話す
（前面 SHIFT+SPECIAL の ``intFc`` で選択。説明書 5-3）。SAIGEN は GPIB だけを
扱う。リソース文字列は ``GPIB0::<アドレス>::INSTR``。アドレス（0〜30）と送信
ターミネータは前面 SHIFT+ADDRESS でしか設定できない（説明書 5-4）。同じバスの
機器とアドレスが重ならないことを両方のパネルで確かめること。

コマンドは LI5640 取扱説明書（英語版 DB509993-003）第 5 章で確認した。

測定の設定（感度・時定数・参照信号・入力・フィルタ・表示など）は前面パネルの
もので、SAIGEN は変えない。送る設定コマンドは読み出し項目の選択 ``OTYP`` だけ
（リモート専用の設定、説明書 5-22, 5-32）。接続時に ``OTYP?`` で元の値を覚え、
切断時に戻して読み返す。そのほかの副作用: デバイスクリア（入出力バッファを
消す。設定は変わらない、5-77, 5-78）、``*CLS``（イベントレジスタとエラーキューを
消す。戻さない、5-36）、GTL までの REMOTE 状態（前面パネル操作不可、5-6）。

読み出し（説明書 5-32, 5-70）:

- ``OTYP 1,2,3,5`` -- ``DOUT?`` が DATA1, DATA2, FREQ（参照信号の測定周波数 Hz。
  高調波測定では基本波）, OVERLEVEL（``OVCR?`` と同じ整数）を返すようにする
- ``DOUT?`` -- 最新の値。DATA1 / DATA2 は**パネル表示の値**で、意味は表示選択・
  正規化・レシオ・オフセット・拡大で変わる。そのため値の前後で設定を読み
  （:meth:`LI5640.snapshot`）、意味を :func:`describe` で決める
- ``WRCR?`` -- 警告状態（1 = UNLOCK、2 = SIGNAL 参照で時定数が小さく同期できない）

``FREQ?`` は内部発振器の設定周波数で、測定周波数ではない（説明書 5-23）。使わない。
``DOUT?`` にはヘッダーは付かない（5-21 のヘッダー出力は旧機種互換メッセージ専用）。
オーバーレベル時の ``DOUT?`` の数値は説明書に書かれていないので、値ではなく
OVERLEVEL のビットで判定する。

ドライバは実験・フロー・ロボットについて何も知らない。非同期化・中断・
排他は SharedDevices（src/devices/safety/shared_devices.py）が受け持つ。

使用例:
    from src.devices.li5640 import LI5640

    lia = LI5640("GPIB0::<address>::INSTR")
    try:
        lia.clear_status()
        print(lia.identify())
        saved = lia.output_type()                     # 例 '1,2'
        lia.set_output_type(READ_OUTPUT_TYPE)
        snap = lia.snapshot()
        print(describe(snap), lia.read_sample())
    finally:
        lia.release(saved)                            # クリア・OTYP 復元・GTL・切断
"""
import logging
import math
import re
from typing import Optional

logger = logging.getLogger(__name__)

#: DOUT? の並び: DATA1, DATA2, FREQ, OVERLEVEL（説明書 5-32 の OTYP 番号）
READ_OUTPUT_TYPE = "1,2,3,5"

#: DDEF? 1 / DDEF? 2 の値 -> 表示内容（説明書 5-28）
DATA1_KINDS = {0: "X", 1: "R", 2: "NOISE", 3: "AUX1"}
DATA2_KINDS = {0: "Y", 1: "THETA", 2: "AUX1", 3: "AUX2"}

#: OVCR（OVERLEVEL）のビット（説明書 5-41）
OVER_INPUT = 1
OVER_MIDDLE = 2
OVER_DATA1 = 4
OVER_DATA2 = 8
OVER_RATIO1 = 16
OVER_RATIO2 = 32
OVER_ALL = 63
#: WRCR のビット（説明書 5-40）
WARN_UNLOCK = 1
WARN_SIGNAL_LOCK = 2
WARN_ALL = 3
#: OPCR のビット（説明書 5-39）。自動設定の実行中は測定値の意味が変わりうる
OPCR_AUTO_SETUP = 256
OPCR_AUTO_SENSITIVITY = 512

#: 感度コード 0〜26 の表示（説明書 5-26）。電圧は 2 nV〜1 V、電流は 5 fA〜1 µA
#: （電流のコード 0 は無い）。1-2-5 系列で、電流は同じコードの電圧の 1e-6 倍
_SENS_MANTISSA = (2, 5, 10)


def sensitivity_value(code: int, current: bool) -> Optional[float]:
    """感度コードをフルスケール値（V または A）にする。範囲外なら None"""
    if not isinstance(code, int) or not 0 <= code <= 26 or (current and code == 0):
        return None
    volts = _SENS_MANTISSA[code % 3] * 10.0 ** (code // 3) * 1e-9
    return volts * 1e-6 if current else volts


def time_constant_value(code: int) -> Optional[float]:
    """時定数コード 0〜19 を秒にする（10 µs, 30 µs, ..., 30 ks。説明書 5-27）"""
    if not isinstance(code, int) or not 0 <= code <= 19:
        return None
    return (1 if code % 2 == 0 else 3) * 10.0 ** (code // 2) * 1e-5


#: SLOP? の値 -> dB/oct（説明書 5-27）
SLOPES_DB_OCT = {0: 6, 1: 12, 2: 18, 3: 24}

_PREFIXES = ((1e-15, "f"), (1e-12, "p"), (1e-9, "n"), (1e-6, "µ"), (1e-3, "m"), (1.0, ""),
             (1e3, "k"))


def format_si(value: Optional[float], unit: str) -> str:
    """1e-7, 'V' -> '100 nV'（記録用の表示。値そのものは別に残す）"""
    if value is None:
        return ""
    for scale, prefix in reversed(_PREFIXES):
        if abs(value) >= scale * 0.9999:
            return f"{value / scale:.4g} {prefix}{unit}"
    return f"{value:.4g} {unit}"


class LI5640Error(RuntimeError):
    """機器がエラーを報告した、または応答を解釈できなかった"""


class LI5640Aborted(LI5640Error):
    """打ち切り要求（``abort_check``）により、次の I/O を送らずに止めた"""


class LockinStepError(LI5640Error):
    """測定ステップの失敗。``diagnostics`` に記録用の構造化データを持つ

    実行系（run_flow / GUI / CSV runner）は ``getattr(e, "diagnostics", None)`` を
    ログ（measurements.csv / events / metadata）へ渡す。
    """

    def __init__(self, message: str, diagnostics: Optional[dict] = None):
        super().__init__(message)
        self.diagnostics = diagnostics or {}


# ----------------------------------------------------------------------
# 応答の解釈（純粋関数。テストで実機なしに確かめる）
# ----------------------------------------------------------------------
def parse_error(reply: str):
    """``EROR?`` の応答 ``-113,"Undefined header"`` を (番号, 全文) にする"""
    head = str(reply).split(",", 1)[0].strip()
    try:
        return int(head), str(reply).strip()
    except ValueError:
        raise LI5640Error(f"EROR? の応答を解釈できません: {reply!r}")


def parse_int(reply: str, low: int, high: int, what: str) -> int:
    """整数の応答を範囲つきで読む。読めない・範囲外は LI5640Error"""
    text = str(reply).strip()
    if not re.fullmatch(r"[-+]?\d+", text):
        raise LI5640Error(f"{what} の応答を整数として読めません: {reply!r}")
    value = int(text)
    if not low <= value <= high:
        raise LI5640Error(f"{what} の応答が範囲外です（{low}〜{high}）: {reply!r}")
    return value


def parse_float(reply: str, low: float, high: float, what: str) -> float:
    """実数の応答を範囲つきで読む。読めない・非有限・範囲外は LI5640Error"""
    try:
        value = float(str(reply).strip())
    except ValueError:
        raise LI5640Error(f"{what} の応答を数値として読めません: {reply!r}")
    if not math.isfinite(value) or not low <= value <= high:
        raise LI5640Error(f"{what} の応答が範囲外です（{low:g}〜{high:g}）: {reply!r}")
    return value


def parse_output_type(reply: str) -> str:
    """``OTYP?`` の応答（例 ``1,2``）を正規化する。項目は 0〜5 を 1〜6 個"""
    items = [s.strip() for s in str(reply).strip().split(",")]
    if not 1 <= len(items) <= 6 or not all(re.fullmatch(r"[0-5]", s) for s in items):
        raise LI5640Error(f"OTYP? の応答を解釈できません: {reply!r}")
    return ",".join(items)


def parse_dout(reply: str) -> Optional[dict]:
    """``OTYP 1,2,3,5`` の ``DOUT?`` 応答を読む。形が違えば None

    DATA1 / DATA2 / FREQ は有限な実数、OVERLEVEL は 0〜63 の整数でなければならない。
    """
    fields = [s.strip() for s in str(reply).strip().split(",")]
    if len(fields) != 4:
        return None
    try:
        data1, data2, freq = (float(s) for s in fields[:3])
    except ValueError:
        return None
    if not all(math.isfinite(v) for v in (data1, data2, freq)):
        return None
    if not re.fullmatch(r"[-+]?\d+", fields[3]):
        return None
    overlevel = int(fields[3])
    if not 0 <= overlevel <= OVER_ALL:
        return None
    return {"data1": data1, "data2": data2, "freq": freq, "overlevel": overlevel}


def parse_warning(reply: str) -> Optional[int]:
    """``WRCR?`` の応答。0〜3 の整数でなければ None"""
    text = str(reply).strip()
    if not re.fullmatch(r"[-+]?\d+", text):
        return None
    value = int(text)
    return value if 0 <= value <= WARN_ALL else None


def describe(snap: dict) -> dict:
    """設定スナップショットから DATA1 / DATA2 の意味（種類・表示モード・単位）を決める

    説明書の規則:
    - DATA1 が NOISE なら DATA2 は DDEF に関係なく AUX IN1（4-34）
    - レシオ表示は X / Y / R だけに効き、正規化（dB / %）より優先する（4-33）
    - 正規化は X / R の表示に効く（4-32）。NOISE・AUX には効かない
    - NOISE の単位は電圧入力で V/√Hz、電流入力で A/√Hz（4-35）
    - θ は deg、AUX は V（4-30）。拡大は dB / % とレシオでは効かない（4-32, 4-33）

    Y に正規化が効くかは説明書から読み取れないので、正規化中の Y は単位を空にして
    ``mode = "normalize?"`` とする（意味を決めつけない）。
    """
    kind1 = DATA1_KINDS[snap["ddef1"]]
    kind2 = "AUX1" if kind1 == "NOISE" else DATA2_KINDS[snap["ddef2"]]
    current = snap["isrc"] in (2, 3)
    base = "A" if current else "V"
    ratio = snap["rat"] == 1
    norm = {0: None, 1: "dB", 2: "%"}[snap["norm"]]

    def channel(kind: str, normalizable: bool) -> tuple:
        if kind in ("X", "Y", "R"):
            if ratio:
                return "ratio", ""
            if norm is not None:
                return (norm, norm) if normalizable else ("normalize?", "")
            return "plain", base
        if kind == "THETA":
            return "plain", "deg"
        if kind == "NOISE":
            return "plain", f"{base}/√Hz"
        return "plain", "V"                      # AUX1 / AUX2

    mode1, unit1 = channel(kind1, normalizable=kind1 in ("X", "R"))
    mode2, unit2 = channel(kind2, normalizable=False)
    return {"data1_kind": kind1, "data1_mode": mode1, "data1_unit": unit1,
            "data2_kind": kind2, "data2_mode": mode2, "data2_unit": unit2}


def invalid_reason(sample: Optional[dict], warning: Optional[int], desc: dict) -> Optional[str]:
    """サンプルを捨てる理由。使えるなら None

    保守的な方針: 入力段・中段のオーバーレベル、UNLOCK / SIGNAL 参照の同期不可、
    表示中のチャネル（レシオ表示ならそのレシオ）のオーバーレベルのどれかがあれば、
    AUX のチャネルも含めてサンプル全体を捨てる。
    """
    if sample is None:
        return "unparsable"
    if warning is None:
        return "status_unparsable"
    over = sample["overlevel"]
    if over & (OVER_INPUT | OVER_MIDDLE):
        return "overlevel_input"
    bit1 = OVER_RATIO1 if desc["data1_mode"] == "ratio" else OVER_DATA1
    bit2 = OVER_RATIO2 if desc["data2_mode"] == "ratio" else OVER_DATA2
    if over & (bit1 | bit2):
        return "overlevel_display"
    if warning & WARN_ALL:
        return "unlock"
    return None


def theta_median(values: list) -> Optional[float]:
    """位相（deg）の中央値。全点が幅 180° 未満の円弧に入らなければ None（曖昧）

    円周上で隣り合う点の最大の隙間が 180° を超えるなら、その隙間の反対側の円弧
    （幅 180° 未満）に全点が入る。その円弧の上で並べて中央値を取り、[-180, 180) に
    戻す。点の順序には依存しない。
    """
    if not values:
        return None
    angles = sorted(v % 360.0 for v in values)
    n = len(angles)
    # 隣との隙間。最後の点から最初の点へは一周して戻る（全点が同じ角度なら 360°。
    # ``% 360`` で求めると 0° になり、ばらつきのない測定を曖昧と誤判定する）
    gaps = [angles[i + 1] - angles[i] for i in range(n - 1)] + [angles[0] + 360.0 - angles[-1]]
    widest = max(range(n), key=lambda i: gaps[i])
    if gaps[widest] <= 180.0:
        return None
    start = angles[(widest + 1) % n]
    unwrapped = sorted(start + ((a - start) % 360.0) for a in angles)
    mid = n // 2
    median = unwrapped[mid] if n % 2 else (unwrapped[mid - 1] + unwrapped[mid]) / 2.0
    return (median + 180.0) % 360.0 - 180.0


def snapshot_diff(before: dict, after: dict) -> dict:
    """2 つのスナップショットで値が違う項目 -> [前, 後]"""
    keys = sorted(set(before) | set(after))
    return {k: [before.get(k), after.get(k)] for k in keys if before.get(k) != after.get(k)}


def _gtl_mode():
    """GPIB の Go To Local（GTL）を指す PyVISA の定数（宛先はこの機器だけ）"""
    try:
        from pyvisa.constants import RENLineOperation
        return RENLineOperation.address_gtl
    except ImportError:          # テスト用の偽リソースのみ（実機は PyVISA が必須）
        return 6                 # VI_GPIB_REN_ADDRESS_GTL


_MODEL = re.compile(r"\bLI\s?5640\b", re.IGNORECASE)
_MAX_ERROR_READS = 21            # エラーキューは最大 20 件（5-68）+ 終端の 0


class LI5640:
    """LI5640 を PyVISA で読む薄いドライバ

    Args:
        resource: VISA リソース文字列（例 ``GPIB0::<アドレス>::INSTR``）。既定値は持たない
        timeout_ms: 1 回の I/O の上限（ミリ秒）
        resource_manager: テスト用の差し替え。省略時は ``pyvisa.ResourceManager()``
    """

    def __init__(self, resource: str, timeout_ms: int = 5000, resource_manager=None):
        if not resource:
            raise ValueError("VISA リソース文字列が空です（例: GPIB0::<アドレス>::INSTR）")
        if resource_manager is None:
            import pyvisa            # 実機を使うときだけ必要（Mock / テストでは不要）
            resource_manager = pyvisa.ResourceManager()
        self.resource = resource
        #: 呼び出し側が設定する打ち切り判定（引数なし、True で打ち切り）。通常の
        #: write / query の直前に見る。終了処理（:meth:`release`）は見ない
        self.abort_check = None
        self._inst = resource_manager.open_resource(resource)
        try:
            self._inst.timeout = int(timeout_ms)
            # 受信は END（EOI）で終わらせ、パネルで選んだ CR / LF / CR+LF は削る
            # （説明書 5-4。どの送信ターミネータでも同じに読める）
            self._inst.read_termination = None
            self._inst.write_termination = "\n"
        except BaseException:
            self._inst.close()
            raise

    # ------------------------------------------------------------------
    # 低レベル I/O
    # ------------------------------------------------------------------
    def _check_abort(self, command: str):
        if self.abort_check is not None and self.abort_check():
            raise LI5640Aborted(f"打ち切り要求のため送信しません: {command}")

    def write(self, command: str):
        self._check_abort(command)
        self._raw_write(command)

    def query(self, command: str) -> str:
        self._check_abort(command)
        return self._raw_query(command)

    def _raw_write(self, command: str):
        logger.debug(f"LI5640 <- {command}")
        self._inst.write(command)

    def _raw_query(self, command: str) -> str:
        reply = str(self._inst.query(command)).strip("\r\n")
        logger.debug(f"LI5640 <- {command} -> {reply!r}")
        return reply

    # ------------------------------------------------------------------
    # 機器レベルの操作
    # ------------------------------------------------------------------
    def clear_status(self):
        """デバイスクリア（この機器の入出力バッファを消す）と ``*CLS``"""
        self._inst.clear()
        self.write("*CLS")

    def identify(self) -> str:
        """``*IDN?`` を返す。機種名 LI5640 が含まれなければ例外"""
        idn = self.query("*IDN?").strip()
        if not _MODEL.search(idn):
            raise LI5640Error(f"LI5640 ではありません: *IDN? = {idn!r}")
        return idn

    def check_errors(self, context: str = ""):
        """エラーキューを ``0`` まで読み、1 件でもあれば LI5640Error

        キューは最大 20 件なので 21 回目に 0 が返れば正常。21 回読んでも 0 に
        届かなければ、それ自体をエラーにする。
        """
        errors = []
        for _ in range(_MAX_ERROR_READS):
            code, text = parse_error(self.query("EROR?"))
            if code == 0:
                break
            errors.append(text)
        else:
            errors.append(f"{_MAX_ERROR_READS} 回読んでもエラーキューが空になりません")
        if errors:
            where = f"{context}: " if context else ""
            raise LI5640Error(f"{where}機器がエラーを報告しました: {'; '.join(errors)}")

    def output_type(self) -> str:
        """``OTYP?``（DOUT? が返す項目の並び）"""
        return parse_output_type(self.query("OTYP?"))

    def set_output_type(self, spec: str = READ_OUTPUT_TYPE):
        """``OTYP`` を設定し、受け付けられたことをエラーキューで確かめる"""
        self.write(f"OTYP {parse_output_type(spec)}")
        self.check_errors("OTYP の設定")

    def continuous_output(self) -> bool:
        """連続データ出力（``OSTR?``）が動いているか。動いていると他の問い合わせが
        通らない（説明書 5-68 の -440）ので、SAIGEN は止まっていることを接続条件にする
        """
        return parse_int(self.query("OSTR?"), 0, 1, "OSTR?") == 1

    def snapshot(self) -> dict:
        """値の意味と再現性に関わるパネル設定を読む（問い合わせだけ）

        読めない・範囲外の応答は LI5640Error。I/O エラーはそのまま送出する
        （応答の順序がずれたまま進むと、設定値を測定値と取り違える）。
        """
        q = self.query
        s = {}
        s["ddef1"] = parse_int(q("DDEF? 1"), 0, 3, "DDEF? 1")
        s["ddef2"] = parse_int(q("DDEF? 2"), 0, 3, "DDEF? 2")
        s["isrc"] = parse_int(q("ISRC?"), 0, 3, "ISRC?")
        current = s["isrc"] in (2, 3)
        s["norm"] = parse_int(q("NORM?"), 0, 2, "NORM?")
        # 正規化の基準値と K 定数の範囲は説明書 5-28, 5-30 のとおり
        if current:
            s["istd"] = parse_float(q("ISTD?"), 1e-15, 1e-6, "ISTD?")
        else:
            s["vstd"] = parse_float(q("VSTD?"), 1e-9, 1.0, "VSTD?")
        s["rat"] = parse_int(q("RAT?"), 0, 1, "RAT?")
        s["kfac"] = parse_float(q("KFAC?"), 0.1, 9.999, "KFAC?")
        for i in (1, 2):
            s[f"ofso{i}"] = parse_int(q(f"OFSO? {i}"), 0, 1, f"OFSO? {i}")
            s[f"offs{i}"] = parse_float(q(f"OFFS? {i}"), -100.0, 100.0, f"OFFS? {i}")
            s[f"oexp{i}"] = parse_int(q(f"OEXP? {i}"), 0, 2, f"OEXP? {i}")
        s["phas"] = parse_float(q("PHAS?"), -180.0, 180.0, "PHAS?")
        s["icpl"] = parse_int(q("ICPL?"), 0, 1, "ICPL?")
        s["ignd"] = parse_int(q("IGND?"), 0, 1, "IGND?")
        if current:
            s["isen"] = parse_int(q("ISEN?"), 1, 26, "ISEN?")
        else:
            s["vsen"] = parse_int(q("VSEN?"), 0, 26, "VSEN?")
        s["tcon"] = parse_int(q("TCON?"), 0, 19, "TCON?")
        s["slop"] = parse_int(q("SLOP?"), 0, 3, "SLOP?")
        s["sync"] = parse_int(q("SYNC?"), 0, 1, "SYNC?")
        s["nois"] = parse_int(q("NOIS?"), 0, 4, "NOIS?")
        s["ilin"] = parse_int(q("ILIN?"), 0, 3, "ILIN?")
        s["ifrq"] = parse_int(q("IFRQ?"), 0, 1, "IFRQ?")
        s["ithr"] = parse_int(q("ITHR?"), 0, 1, "ITHR?")
        s["drsv"] = parse_int(q("DRSV?"), 0, 2, "DRSV?")
        s["rsrc"] = parse_int(q("RSRC?"), 0, 2, "RSRC?")
        s["redg"] = parse_int(q("REDG?"), 0, 2, "REDG?")
        s["harm"] = parse_int(q("HARM?"), 1, 19999, "HARM?")
        s["opcr"] = parse_int(q("OPCR?"), 0, 65535, "OPCR?")
        s["ostr"] = parse_int(q("OSTR?"), 0, 1, "OSTR?")
        return s

    def read_sample(self) -> tuple:
        """``DOUT?`` と ``WRCR?`` を 1 回ずつ読む（同時刻ではない）

        Returns:
            (sample, warning, raw): sample は :func:`parse_dout` の dict か None、
            warning は 0〜3 か None、raw は ``DOUT?`` の応答文字列
        """
        raw = self.query("DOUT?")
        sample = parse_dout(raw)
        if sample is None:
            logger.warning(f"LI5640 の DOUT? 応答を解釈できません: {raw!r}")
        warning = parse_warning(self.query("WRCR?"))
        return sample, warning, raw

    # ------------------------------------------------------------------
    # 終了処理（abort_check を見ない）
    # ------------------------------------------------------------------
    def release(self, saved_output_type: Optional[str]) -> bool:
        """デバイスクリア -> OTYP 復元と読み返し -> GTL -> 切断

        どれかが失敗しても次へ進む（復元できなくても前面操作に戻して閉じる）。
        打ち切り要求は見ない。各 I/O はタイムアウト付き。

        Returns:
            bool: 閉じられたら True（close() が失敗したら例外を送出する）
        """
        try:
            self._inst.clear()           # 途中まで送ったコマンドが残っていれば捨てる
        except Exception as e:  # noqa: BLE001 - 後続を優先
            logger.warning(f"LI5640 のデバイスクリアに失敗しました: {e}")
        if saved_output_type is not None:
            try:
                self._raw_write(f"OTYP {saved_output_type}")
            except Exception as e:  # noqa: BLE001 - 届いている可能性があるので読み返しは試す
                logger.error(f"LI5640 の OTYP {saved_output_type} の送信に失敗しました: {e}")
            try:
                now = parse_output_type(self._raw_query("OTYP?"))
                if now != saved_output_type:
                    logger.error(f"LI5640 の OTYP を戻せませんでした: {now} != {saved_output_type}")
                else:
                    logger.info(f"LI5640 の OTYP を {saved_output_type} に戻しました")
            except Exception as e:  # noqa: BLE001 - 後続を優先
                logger.error(f"LI5640 の OTYP を戻せませんでした（読み返し失敗、"
                             f"{saved_output_type}）: {e}")
        try:
            self.local()
        except Exception as e:  # noqa: BLE001 - 閉じることを優先する
            logger.warning(f"LI5640 を前面操作に戻せません（GTL）: {e}")
        self.close()
        return True

    def local(self):
        """前面パネル操作に戻す（この機器宛ての GTL。バス全体の REN は触らない）"""
        self._inst.control_ren(_gtl_mode())

    def close(self):
        self._inst.close()


if __name__ == "__main__":
    # 接続確認（人が実行する）: *IDN?、設定の読み取り、DOUT? 1 回。OTYP は元に戻す。
    #   python -m src.devices.li5640.li5640 GPIB0::<address>::INSTR
    import json
    import sys

    logging.basicConfig(level=logging.INFO)
    if len(sys.argv) != 2:
        print("usage: python -m src.devices.li5640.li5640 <VISA resource>")
        sys.exit(2)
    lia = LI5640(sys.argv[1])
    saved = None
    try:
        lia.clear_status()
        print(lia.identify())
        lia.check_errors("接続確認")
        if lia.continuous_output():
            raise SystemExit("連続データ出力（OSTR 1）が動いています。止めてから実行してください")
        saved = lia.output_type()
        print(f"OTYP (saved): {saved}")
        lia.set_output_type(READ_OUTPUT_TYPE)
        snap = lia.snapshot()
        print(json.dumps(snap))
        print(describe(snap))
        sample, warning, raw = lia.read_sample()
        print(f"DOUT? {raw!r} -> {sample}, WRCR {warning}")
    finally:
        lia.release(saved)
