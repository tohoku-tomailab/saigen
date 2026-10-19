"""
LI5640 内部発振器（FREQ / AMPL）の値の規則（純粋関数。機器にもフローにも触らない）

説明書 5-23 と 7.4:

- ``FREQ <f>``: 0.0005 Hz〜105.00 kHz。分解能は 4 1/2 桁（最大 19999 カウント）か
  0.1 mHz の粗いほう。``FREQ?`` -> ``1.0000E+03``
- ``AMPL x,i``: x は実効値 [Vrms]。i はレンジ: 0 = 0.0000〜0.0500 V（0.1 mV 刻み）、
  1 = 0.000〜0.500 V（1 mV）、2 = 0.00〜5.00 V（10 mV）。``AMPL?`` -> ``0.100,1``

SAIGEN はレンジを切り替えない（切り替え時の出力の過渡が説明書に無い）。要求できる
振幅は、接続時に読んだレンジで表せる値だけ。表せない値は丸めずに拒否する。

書き込みは ``config.yaml`` の上限で許可する（既定 0 = 無効）:

- ``lockin_osc_max_vrms``: 0 なら書き込み無効。0 < 値 <= 5.0 で有効（設定値の上限。
  実際の出力電圧・継続時間・負荷を保護するものではない）
- ``lockin_osc_min_hz`` / ``lockin_osc_max_hz``: 周波数の範囲（既定 0.0005〜105000）
"""
from decimal import Decimal, InvalidOperation
from typing import NamedTuple, Optional

ACTION = "set_lockin_oscillator"

FREQ_MIN = Decimal("0.0005")
FREQ_MAX = Decimal("105000")
_FREQ_MAX_COUNT = 19999
_FREQ_MIN_EXP = -4              # 0.1 mHz

#: レンジ番号 -> (フルスケール [Vrms], 刻み [V], 小数点以下の桁数)
RANGES = {
    0: (Decimal("0.0500"), Decimal("0.0001"), 4),
    1: (Decimal("0.500"), Decimal("0.001"), 3),
    2: (Decimal("5.00"), Decimal("0.01"), 2),
}
AMPL_MAX = RANGES[2][0]


def to_decimal(value, what: str) -> Decimal:
    """有限の数（bool は不可）を Decimal にする。文字列表記の数値も受け付ける"""
    if isinstance(value, bool) or value is None:
        raise ValueError(f"{what} は数値でなければなりません: {value!r}")
    try:
        d = Decimal(str(value).strip())
    except (InvalidOperation, ValueError):
        raise ValueError(f"{what} を数値として読めません: {value!r}")
    if not d.is_finite():
        raise ValueError(f"{what} は有限の数でなければなりません: {value!r}")
    return d


def frequency_representable(f: Decimal) -> bool:
    """f が「19999 以下の整数カウント x 10^k（k >= -4）」で表せるか"""
    if not FREQ_MIN <= f <= FREQ_MAX:
        return False
    # 刻み 10^exp を細かい方から試す。整数でなくなったら、それより粗い刻みでも表せない
    for exp in range(_FREQ_MIN_EXP, 6):
        counts = f.scaleb(-exp)
        if counts != counts.to_integral_value():
            return False
        if counts <= _FREQ_MAX_COUNT:
            return True
    return False


def check_frequency(value) -> Decimal:
    """要求周波数を検査して Decimal で返す。範囲外・表せない値は ValueError"""
    f = to_decimal(value, "frequency")
    if not FREQ_MIN <= f <= FREQ_MAX:
        raise ValueError(f"frequency は {FREQ_MIN}〜{FREQ_MAX} Hz: {value!r}")
    if not frequency_representable(f):
        raise ValueError(f"frequency {value!r} Hz は LI5640 の分解能（4 1/2 桁、最大 19999 "
                         "カウント、または 0.1 mHz）で表せません")
    return f


def format_frequency(f: Decimal) -> str:
    """``FREQ`` に送る形（説明書の例と同じ ``1.0000E+03``。5 桁の仮数なので、表せる値なら
    丸めは起きない）"""
    return f"{float(f):.4E}"


def amplitude_fits(x: Decimal, rng: int) -> bool:
    """x がレンジ rng のフルスケール以内で、刻みの整数倍か"""
    full_scale, step, _ = RANGES[rng]
    if not Decimal(0) <= x <= full_scale:
        return False
    counts = x / step
    return counts == counts.to_integral_value()


def check_amplitude(value) -> Decimal:
    """要求振幅を検査して Decimal で返す（どれかのレンジで表せること）"""
    x = to_decimal(value, "amplitude")
    if not Decimal(0) <= x <= AMPL_MAX:
        raise ValueError(f"amplitude は 0〜{AMPL_MAX} Vrms: {value!r}")
    if not fitting_ranges([x]):
        raise ValueError(f"amplitude {value!r} Vrms はどのレンジの刻み（0.1 mV / 1 mV / 10 mV）"
                         "でも表せません")
    return x


def fitting_ranges(amplitudes) -> tuple:
    """すべての振幅を表せるレンジ番号（小さい順）"""
    return tuple(r for r in sorted(RANGES) if all(amplitude_fits(x, r) for x in amplitudes))


def format_amplitude(x: Decimal, rng: int) -> str:
    """``AMPL`` に送る形（例 ``0.300,1``）"""
    digits = RANGES[rng][2]
    return f"{x:.{digits}f},{rng}"


def parse_frequency_reply(reply) -> Decimal:
    """``FREQ?`` の応答。読めない・範囲外は ValueError"""
    f = to_decimal(reply, "FREQ? の応答")
    if not FREQ_MIN <= f <= FREQ_MAX:
        raise ValueError(f"FREQ? の応答が範囲外です: {reply!r}")
    return f


def parse_amplitude_reply(reply) -> tuple:
    """``AMPL?`` の応答 ``0.300,1`` -> (Decimal('0.300'), 1)。読めない・範囲外は ValueError"""
    parts = [s.strip() for s in str(reply).strip().split(",")]
    if len(parts) != 2 or parts[1] not in ("0", "1", "2"):
        raise ValueError(f"AMPL? の応答を解釈できません: {reply!r}")
    rng = int(parts[1])
    x = to_decimal(parts[0], "AMPL? の応答")
    if not Decimal(0) <= x <= RANGES[rng][0]:
        raise ValueError(f"AMPL? の応答がレンジ {rng} の範囲外です: {reply!r}")
    return x, rng


class OscState(NamedTuple):
    """発振器の設定（周波数 Hz、振幅 Vrms、レンジ番号）"""
    freq: Decimal
    ampl: Decimal
    range: int

    def as_record(self) -> dict:
        """記録用（JSON にできる値）"""
        return {"freq_hz": float(self.freq), "ampl_vrms": float(self.ampl), "range": self.range}


class OscLimits(NamedTuple):
    max_vrms: Decimal
    min_hz: Decimal
    max_hz: Decimal

    @property
    def enabled(self) -> bool:
        return self.max_vrms > 0

    def check_state(self, state: OscState, what: str):
        """state が上限の内側か。外なら ValueError"""
        if state.ampl > self.max_vrms:
            raise ValueError(f"{what} の振幅 {state.ampl} Vrms が lockin_osc_max_vrms "
                             f"{self.max_vrms} を超えています")
        if not self.min_hz <= state.freq <= self.max_hz:
            raise ValueError(f"{what} の周波数 {state.freq} Hz が lockin_osc_min_hz / "
                             f"lockin_osc_max_hz（{self.min_hz}〜{self.max_hz}）の外です")


def _limit_value(config: dict, key: str, default: str) -> Decimal:
    """キーが無ければ既定値。あれば int / float（bool・文字列・null は不可）"""
    if key not in config:
        return Decimal(default)
    value = config[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"shared_devices.{key} は数値でなければなりません: {value!r}")
    return to_decimal(value, f"shared_devices.{key}")


def parse_limits(config: dict) -> OscLimits:
    """共有デバイス設定から上限を読む。不正な値は ValueError（既定値へ黙って戻さない）"""
    config = config or {}
    max_vrms = _limit_value(config, "lockin_osc_max_vrms", "0")
    min_hz = _limit_value(config, "lockin_osc_min_hz", str(FREQ_MIN))
    max_hz = _limit_value(config, "lockin_osc_max_hz", str(FREQ_MAX))
    if not Decimal(0) <= max_vrms <= AMPL_MAX:
        raise ValueError(f"shared_devices.lockin_osc_max_vrms は 0（無効）〜{AMPL_MAX}: "
                         f"{config.get('lockin_osc_max_vrms')!r}")
    if not FREQ_MIN <= min_hz <= max_hz <= FREQ_MAX:
        raise ValueError(f"shared_devices.lockin_osc_min_hz / lockin_osc_max_hz は "
                         f"{FREQ_MIN} <= min <= max <= {FREQ_MAX}: "
                         f"{config.get('lockin_osc_min_hz')!r}, {config.get('lockin_osc_max_hz')!r}")
    return OscLimits(max_vrms, min_hz, max_hz)


class OscPlan(NamedTuple):
    """フローが発振器に書く内容の要約（ResourcePlan からデバイスへ渡す）

    first_frequency: 最初に要求する周波数（Mock の初期状態に使う）
    ranges: フローの全振幅を表せるレンジ（空ならフローは実行できない）
    amplitudes / frequencies: 要求値（検査用）
    """
    first_frequency: Decimal
    ranges: tuple
    amplitudes: tuple
    frequencies: tuple


def _step_value(step, key):
    return step.get(key) if isinstance(step, dict) else getattr(step, key, None)


def plan_from_steps(steps) -> Optional[OscPlan]:
    """フローの set_lockin_oscillator ステップから OscPlan を作る。無ければ None

    値の検査はしない（:func:`preflight_errors` が行う）。読めない値は計画から外す。
    """
    freqs, ampls = [], []
    for step in steps:
        if _step_value(step, "action") != ACTION:
            continue
        try:
            freqs.append(check_frequency(_step_value(step, "frequency")))
        except ValueError:
            pass
        try:
            ampls.append(check_amplitude(_step_value(step, "amplitude")))
        except ValueError:
            pass
    if not any(_step_value(s, "action") == ACTION for s in steps):
        return None
    return OscPlan(freqs[0] if freqs else FREQ_MIN, fitting_ranges(ampls), tuple(ampls),
                   tuple(freqs))


def preflight_errors(steps, config: dict) -> list:
    """デバイスを開く前に分かる問題（1 件 1 文）。発振器を書かないフローなら []

    接続してから分かること（保存されたレンジ・開始時の設定が上限の内側か）は含まない。
    """
    indexed = [(i, s) for i, s in enumerate(steps, 1) if _step_value(s, "action") == ACTION]
    if not indexed:
        return []
    try:
        limits = parse_limits(config)
    except ValueError as e:
        return [str(e)]
    if not limits.enabled:
        return [f"{ACTION} は無効です（config.yaml の shared_devices.lockin_osc_max_vrms が 0）。"
                "OSC OUT につないだ負荷に合わせて上限を設定したときだけ使えます"]
    errors, ampls = [], []
    for i, step in indexed:
        try:
            f = check_frequency(_step_value(step, "frequency"))
            if not limits.min_hz <= f <= limits.max_hz:
                errors.append(f"ステップ {i}: frequency {f} Hz が設定の範囲 "
                              f"{limits.min_hz}〜{limits.max_hz} Hz の外です")
        except ValueError as e:
            errors.append(f"ステップ {i}: {e}")
        try:
            x = check_amplitude(_step_value(step, "amplitude"))
            ampls.append(x)
            if x > limits.max_vrms:
                errors.append(f"ステップ {i}: amplitude {x} Vrms が lockin_osc_max_vrms "
                              f"{limits.max_vrms} を超えています")
        except ValueError as e:
            errors.append(f"ステップ {i}: {e}")
    if ampls and not fitting_ranges(ampls):
        errors.append("フローの全振幅を表せる共通のレンジがありません "
                      f"（{', '.join(str(a) for a in ampls)} Vrms）。SAIGEN はレンジを切り替えません")
    return errors
