"""Channel names, named signals and the output policy of the NI DAQ device.

No hardware and no nidaqmx here: this module is what keeps the three layers apart.

- **Code** (the driver and the ``daq_*`` steps) knows how to acquire, generate and
  count. It knows nothing about what is wired to the terminal block.
- **config.yaml** describes the wiring of one lab: the device name, the named
  signals (``daq_signals``: "photodiode" is ai0, differential, 2 mW per volt) and
  which outputs may be driven at all.
- **A flow** says what the experiment does, with the names. The same flow runs in
  another lab whose photodiode is on ai5.

A flow may also name a channel directly, without the device ("ai0", "ao1",
"port0/line3", "ctr0", "pfi0"). Inputs are always allowed. Outputs can switch
whatever is wired to the terminal block, so they are refused unless the physical
channel is listed in ``config.yaml`` (``daq_ao_channels`` / ``daq_do_lines`` /
``daq_co_counters``); an analog output must stay inside ``daq_ao_min_v`` ..
``daq_ao_max_v`` and a pulse train below ``daq_co_max_hz``. The real device and the
mock use these same classes, so a flow that passes ``--mock`` is not refused for
these reasons on the bench.
"""

import math
import re
from dataclasses import dataclass
from typing import Iterable, Optional

AI_PATTERN = r"^ai\d{1,2}$"
AO_PATTERN = r"^ao\d$"
LINE_PATTERN = r"^port\d/line\d{1,2}$"
COUNTER_PATTERN = r"^ctr\d$"
#: PFI terminals, and two internal timebases that need no wiring (counter self-test).
TERMINAL_PATTERN = r"^(pfi\d{1,2}|100khztimebase|20mhztimebase)$"
#: A signal name defined in config.yaml (daq_signals).
SIGNAL_NAME_PATTERN = r"^[a-z][a-z0-9_]{0,39}$"
#: Names that look like a physical channel; a signal may not be called like this.
RESERVED_PATTERN = r"^(ai|ao|ctr|pfi|port|line|dev)\d"

#: Voltage span of the analog outputs of the X Series devices (USB-6351: +/-10 V).
DEVICE_AO_MIN_V = -10.0
DEVICE_AO_MAX_V = 10.0

#: USB-6351 limits (NI specification; the device reports the same through DAQmx).
AI_MAX_RATE_SINGLE_HZ = 1_250_000.0     # one channel
AI_MAX_RATE_AGGREGATE_HZ = 1_000_000.0  # several channels, summed over the channels
AO_MAX_RATE_HZ = 2_000_000.0            # device: 2.86 MS/s on one channel; kept below it
CO_DEVICE_MAX_HZ = 1_000_000.0          # pulse trains; the 100 MHz timebase goes higher

TERMINALS = ("default", "rse", "nrse", "diff")
V_RANGES = (0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0)
EDGES = ("rising", "falling")
WAVEFORMS = ("sine", "square", "triangle", "sawtooth")

KINDS = ("ai", "ao", "line", "counter", "terminal")
_PATTERNS = {"ai": AI_PATTERN, "ao": AO_PATTERN, "line": LINE_PATTERN,
             "counter": COUNTER_PATTERN, "terminal": TERMINAL_PATTERN}
_KIND_LABEL = {"ai": "analog input", "ao": "analog output", "line": "digital line",
               "counter": "counter", "terminal": "terminal"}
_EXAMPLES = {"ai": "'ai0'", "ao": "'ao0'", "line": "'port0/line0'", "counter": "'ctr0'",
             "terminal": "'pfi0'"}


class DaqPolicyError(ValueError):
    """An output the configuration does not allow, or a channel / signal name that is wrong."""


def _clean(name) -> str:
    return str(name).strip().lower()


#: The pin a counter's pulse train comes out of is also a digital line (USB-6351:
#: PFI12..15 = port2/line4..7). One pin cannot serve both at the same time.
COUNTER_OUTPUT_LINES = {"ctr0": "port2/line4", "ctr1": "port2/line5",
                        "ctr2": "port2/line6", "ctr3": "port2/line7"}


def physical_kind(name) -> Optional[str]:
    """The kind of a physical channel name ("ai0" -> "ai"), or None if it is not one."""
    text = _clean(name)
    for kind, pattern in _PATTERNS.items():
        if re.match(pattern, text):
            return kind
    return None


def _normalize(name, kind: str) -> str:
    text = _clean(name)
    if not re.match(_PATTERNS[kind], text):
        raise DaqPolicyError(f"{_KIND_LABEL[kind]} channel name {name!r} is not valid "
                             f"(expected e.g. {_EXAMPLES[kind]}, without the device name)")
    return text


def normalize_ai(name) -> str:
    return _normalize(name, "ai")


def normalize_ao(name) -> str:
    return _normalize(name, "ao")


def normalize_line(name) -> str:
    return _normalize(name, "line")


def normalize_counter(name) -> str:
    return _normalize(name, "counter")


def normalize_terminal(name) -> str:
    return _normalize(name, "terminal")


def check_flow_name(name, kinds) -> str:
    """Validate a channel field of a flow step without the configuration.

    Accepts a physical channel of one of ``kinds`` or a signal name. A physical
    channel of another kind ("ai0" where an output is expected) and anything that
    merely looks physical ("port0", "Dev1/ai0") are rejected here, before a run.
    Whether a signal name exists is known only with config.yaml (``DaqSignals``).
    """
    kinds = (kinds,) if isinstance(kinds, str) else tuple(kinds)
    text = _clean(name)
    if physical_kind(text) in kinds:
        return text
    expected = " or ".join(_EXAMPLES[k] for k in kinds)
    if re.match(RESERVED_PATTERN, text) or not re.match(SIGNAL_NAME_PATTERN, text):
        raise ValueError(f"{name!r} is neither a channel ({expected}, without the device name) "
                         "nor a signal name from config.yaml (daq_signals)")
    return text


@dataclass(frozen=True)
class Signal:
    """One resolved channel: where it is wired and how its value is read."""

    label: str                      # what the run log shows ("ai0" or "photodiode:ai0")
    kind: str                       # one of KINDS
    channel: str                    # physical name without the device ("ai0")
    terminal: Optional[str] = None  # ai: wiring ("rse", "diff", ...)
    v_range: Optional[float] = None  # ai: input range +/- V
    scale: float = 1.0              # ai: value = volts * scale + offset
    offset: float = 0.0
    unit: str = ""                  # unit of the scaled value ("" = not scaled)

    @property
    def scaled(self) -> bool:
        return bool(self.unit) or self.scale != 1.0 or self.offset != 0.0

    def value(self, volts: float) -> float:
        return volts * self.scale + self.offset


class DaqSignals:
    """The named signals of ``config.yaml`` (``shared_devices.daq_signals``)."""

    _AI_KEYS = {"terminal", "v_range", "scale", "offset", "unit"}

    def __init__(self, signals: Optional[dict] = None):
        self._signals = {}
        for raw_name, spec in (signals or {}).items():
            name = _clean(raw_name)
            if re.match(RESERVED_PATTERN, name) or not re.match(SIGNAL_NAME_PATTERN, name):
                raise DaqPolicyError(
                    f"daq_signals: {raw_name!r} is not a usable signal name (lower-case letters, "
                    "digits and '_', starting with a letter, and not shaped like a channel)")
            if name in self._signals:
                raise DaqPolicyError(f"daq_signals: {raw_name!r} is defined twice")
            self._signals[name] = self._parse(name, spec)

    @staticmethod
    def _parse(name: str, spec) -> Signal:
        if isinstance(spec, str):
            spec = {"channel": spec}
        if not isinstance(spec, dict) or not spec.get("channel"):
            raise DaqPolicyError(f"daq_signals.{name}: give a channel, e.g. {{channel: ai0}}")
        channel = _clean(spec["channel"])
        kind = physical_kind(channel)
        if kind is None:
            raise DaqPolicyError(f"daq_signals.{name}: {spec['channel']!r} is not a channel "
                                 "(ai0, ao0, port0/line0, ctr0, pfi0; without the device name)")
        extra = set(spec) - {"channel"} - (DaqSignals._AI_KEYS if kind == "ai" else set())
        if extra:
            raise DaqPolicyError(f"daq_signals.{name}: unknown or unusable key(s) {sorted(extra)} "
                                 f"for a channel of kind {kind!r}")
        terminal = spec.get("terminal")
        if terminal is not None and terminal not in TERMINALS:
            raise DaqPolicyError(f"daq_signals.{name}: terminal must be one of {TERMINALS}")
        v_range = spec.get("v_range")
        if v_range is not None:
            if isinstance(v_range, bool) or float(v_range) not in V_RANGES:
                raise DaqPolicyError(f"daq_signals.{name}: v_range must be one of {V_RANGES}")
            v_range = float(v_range)
        try:
            scale = float(spec.get("scale", 1.0))
            offset = float(spec.get("offset", 0.0))
        except (TypeError, ValueError):
            raise DaqPolicyError(f"daq_signals.{name}: scale and offset must be numbers") from None
        if not (math.isfinite(scale) and math.isfinite(offset)) or scale == 0.0:
            raise DaqPolicyError(f"daq_signals.{name}: scale must be a finite, non-zero number "
                                 "and offset a finite number")
        return Signal(label=f"{name}:{channel}", kind=kind, channel=channel, terminal=terminal,
                      v_range=v_range, scale=scale, offset=offset, unit=str(spec.get("unit") or ""))

    @classmethod
    def from_config(cls, config: dict) -> "DaqSignals":
        signals = (config or {}).get("daq_signals") or {}
        if not isinstance(signals, dict):
            raise DaqPolicyError("shared_devices.daq_signals must be a mapping of name -> channel")
        return cls(signals)

    def names(self) -> list:
        return sorted(self._signals)

    def resolve(self, name, kinds) -> Signal:
        """Resolve a channel or signal name used in a flow; ``kinds`` is what the step accepts."""
        kinds = (kinds,) if isinstance(kinds, str) else tuple(kinds)
        text = _clean(name)
        kind = physical_kind(text)
        if kind is not None:
            signal = Signal(label=text, kind=kind, channel=text)
        elif text in self._signals:
            signal = self._signals[text]
        else:
            expected = " or ".join(_EXAMPLES[k] for k in kinds)
            raise DaqPolicyError(
                f"{name!r} is neither a channel ({expected}) nor a signal defined under "
                f"shared_devices.daq_signals in config.yaml (defined: {self.names() or 'none'})")
        if signal.kind not in kinds:
            raise DaqPolicyError(
                f"{signal.label} is a {_KIND_LABEL[signal.kind]}; this step needs "
                + " or ".join(_KIND_LABEL[k] for k in kinds))
        return signal


class DaqOutputPolicy:
    """Which outputs a run may drive, and how far."""

    def __init__(self, ao_channels: Iterable = (), do_lines: Iterable = (),
                 ao_min_v: float = 0.0, ao_max_v: float = 5.0,
                 co_counters: Iterable = (), co_max_hz: float = 1000.0):
        self.ao_channels = frozenset(normalize_ao(c) for c in (ao_channels or ()))
        self.do_lines = frozenset(normalize_line(c) for c in (do_lines or ()))
        self.co_counters = frozenset(normalize_counter(c) for c in (co_counters or ()))
        self.ao_min_v = float(ao_min_v)
        self.ao_max_v = float(ao_max_v)
        self.co_max_hz = float(co_max_hz)
        if not (DEVICE_AO_MIN_V <= self.ao_min_v <= self.ao_max_v <= DEVICE_AO_MAX_V):
            raise DaqPolicyError(
                f"daq_ao_min_v / daq_ao_max_v must satisfy {DEVICE_AO_MIN_V:g} <= min <= max <= "
                f"{DEVICE_AO_MAX_V:g} V, got {self.ao_min_v:g} .. {self.ao_max_v:g}")
        if not (0.0 < self.co_max_hz <= CO_DEVICE_MAX_HZ):
            raise DaqPolicyError(
                f"daq_co_max_hz must be in (0, {CO_DEVICE_MAX_HZ:g}] Hz, got {self.co_max_hz:g}")

    @classmethod
    def from_config(cls, config: dict) -> "DaqOutputPolicy":
        config = config or {}
        for key in ("daq_ao_min_v", "daq_ao_max_v", "daq_co_max_hz"):
            if key in config and config[key] is None:
                # a key with no value is never read as "no limit"
                raise DaqPolicyError(f"shared_devices.{key} has no value (null); give a number")
        return cls(ao_channels=config.get("daq_ao_channels") or (),
                   do_lines=config.get("daq_do_lines") or (),
                   ao_min_v=config.get("daq_ao_min_v", 0.0),
                   ao_max_v=config.get("daq_ao_max_v", 5.0),
                   co_counters=config.get("daq_co_counters") or (),
                   co_max_hz=config.get("daq_co_max_hz", 1000.0))

    def describe(self) -> str:
        return (f"AO {sorted(self.ao_channels) or 'none'} [{self.ao_min_v:g}..{self.ao_max_v:g} V], "
                f"DO {sorted(self.do_lines) or 'none'}, "
                f"CO {sorted(self.co_counters) or 'none'} [<= {self.co_max_hz:g} Hz]")

    def _check_ao_enabled(self, channel) -> str:
        name = normalize_ao(channel)
        if name not in self.ao_channels:
            raise DaqPolicyError(
                f"analog output {name} is not enabled: list it under shared_devices.daq_ao_channels "
                f"in config.yaml after checking what is wired to it "
                f"(enabled: {sorted(self.ao_channels) or 'none'})")
        return name

    def _check_span(self, name: str, volts: float) -> float:
        volts = float(volts)
        if volts != volts or not (self.ao_min_v <= volts <= self.ao_max_v):
            raise DaqPolicyError(
                f"analog output {name} = {volts:g} V is outside the configured span "
                f"{self.ao_min_v:g} .. {self.ao_max_v:g} V (daq_ao_min_v / daq_ao_max_v)")
        return volts

    def check_analog_output(self, channel, voltage) -> tuple:
        """Return (channel, voltage) normalised, or raise DaqPolicyError."""
        name = self._check_ao_enabled(channel)
        return name, self._check_span(name, voltage)

    def check_analog_waveform(self, channel, offset_v, amplitude_v) -> tuple:
        """A waveform swings offset +/- amplitude; both ends must be inside the span."""
        name = self._check_ao_enabled(channel)
        offset, amplitude = float(offset_v), float(amplitude_v)
        if amplitude != amplitude or amplitude < 0.0:
            raise DaqPolicyError(f"analog output {name}: amplitude must be >= 0 V")
        self._check_span(name, offset - amplitude)
        self._check_span(name, offset + amplitude)
        return name, offset, amplitude

    def check_digital_output(self, line) -> str:
        """Return the normalised line name, or raise DaqPolicyError."""
        name = normalize_line(line)
        if name not in self.do_lines:
            raise DaqPolicyError(
                f"digital output {name} is not enabled: list it under shared_devices.daq_do_lines "
                f"in config.yaml after checking what is wired to it "
                f"(enabled: {sorted(self.do_lines) or 'none'})")
        return name

    def check_counter_output(self, counter, frequency_hz, duty_cycle) -> tuple:
        """Return (counter, frequency, duty) normalised, or raise DaqPolicyError."""
        name = normalize_counter(counter)
        if name not in self.co_counters:
            raise DaqPolicyError(
                f"counter output {name} is not enabled: list it under shared_devices.daq_co_counters "
                f"in config.yaml after checking what is wired to its output terminal "
                f"(enabled: {sorted(self.co_counters) or 'none'})")
        freq, duty = float(frequency_hz), float(duty_cycle)
        if freq != freq or not (0.0 < freq <= self.co_max_hz):
            raise DaqPolicyError(
                f"counter output {name}: {freq:g} Hz is outside 0 .. {self.co_max_hz:g} Hz "
                "(daq_co_max_hz)")
        if duty != duty or not (0.0 < duty < 1.0):
            raise DaqPolicyError(f"counter output {name}: duty_cycle must be between 0 and 1")
        return name, freq, duty
