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

from .profiles import (RTD, THERMOCOUPLE, USB_6351, DaqProfileError, DeviceProfile,
                       any_kind_of, get_profile)

# Which channels exist depends on the model (profiles.py). A flow is checked against
# the schema before the model is known: there a name is accepted when some supported
# model has it; with the configuration (DaqSignals, DaqOutputPolicy) it must exist on
# the model in use, for the mock as for the device.
#: A signal name defined in config.yaml (daq_signals).
SIGNAL_NAME_PATTERN = r"^[a-z][a-z0-9_]{0,39}$"
#: Names that look like a physical channel; a signal may not be called like this.
RESERVED_PATTERN = r"^(ai|ao|ctr|pfi|port|line|dev)\d"

#: Limits of the default model, kept under their old names (the code asks the profile).
DEVICE_AO_MIN_V = USB_6351.ao_min_v
DEVICE_AO_MAX_V = USB_6351.ao_max_v
AI_MAX_RATE_SINGLE_HZ = USB_6351.ai_max_rate_single_hz
AI_MAX_RATE_AGGREGATE_HZ = USB_6351.ai_max_rate_aggregate_hz
AO_MAX_RATE_HZ = USB_6351.ao_max_rate_hz
CO_DEVICE_MAX_HZ = USB_6351.co_max_hz
DIO_MAX_RATE_HZ = USB_6351.dio_max_rate_hz
TIMED_PORT = USB_6351.timed_port

#: Every wiring mode and input range some model has (the schema's choice lists).
TERMINALS = ("default", "rse", "nrse", "diff")
V_RANGES = (0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0)
EDGES = ("rising", "falling")
WAVEFORMS = ("sine", "square", "triangle", "sawtooth")
ENCODER_DECODINGS = ("x1", "x2", "x4")

#: Sensors an analog input can be configured for (config.yaml, daq_signals.<name>.sensor).
#: The device has no cold-junction sensor and no excitation source: the cold-junction
#: temperature is a number the lab gives, the RTD current comes from outside. Bridge
#: (strain) channels are not supported by the USB-6351; read them as a voltage with a scale.
SENSORS = ("thermocouple", "rtd")
TC_TYPES = ("B", "E", "J", "K", "N", "R", "S", "T")
RTD_TYPES = ("pt3750", "pt3851", "pt3911", "pt3916", "pt3920", "pt3928")
SENSOR_UNIT = "degC"

KINDS = ("ai", "ao", "line", "counter", "terminal")
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
COUNTER_OUTPUT_LINES = USB_6351.counter_output_lines


def physical_kind(name, profile: Optional[DeviceProfile] = None) -> Optional[str]:
    """The kind of a physical channel name ("ai0" -> "ai"), or None if it is not one.

    With a profile: on that model. Without: on any supported model.
    """
    text = _clean(name)
    return any_kind_of(text) if profile is None else profile.kind_of(text)


def _normalize(name, kind: str) -> str:
    text = _clean(name)
    if any_kind_of(text) != kind:
        raise DaqPolicyError(f"{_KIND_LABEL[kind]} channel name {name!r} is not valid "
                             f"(expected e.g. {_EXAMPLES[kind]}, without the device name)")
    return text


def require_on_model(profile: DeviceProfile, kind: str, name: str, where: str = "") -> str:
    """A channel that is a valid name must also exist on the model in use."""
    if not profile.has(kind, name):
        raise DaqPolicyError(f"{where}the {profile.model} has no {_KIND_LABEL[kind]} {name} "
                             f"(it has {profile.available(kind)})")
    return name


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


def check_timed_lines(lines, profile: Optional[DeviceProfile] = None) -> list:
    """Lines of a hardware-timed digital step: on the timed port (port 0), each once."""
    profile = profile or USB_6351
    names = [normalize_line(line) for line in lines]
    if not names:
        raise DaqPolicyError("at least one digital line is needed")
    if len(set(names)) != len(names):
        raise DaqPolicyError("the same digital line is listed twice")
    if profile.timed_port is None:
        raise DaqPolicyError(f"the {profile.model} has no hardware-timed digital I/O")
    for name in names:
        require_on_model(profile, "line", name)
        if not name.startswith(profile.timed_port + "/"):
            raise DaqPolicyError(f"{name}: only {profile.timed_port} has hardware timing "
                                 "(patterns, clocked acquisition, change detection)")
    return names


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
    sensor: Optional[dict] = None   # ai: {"type": "thermocouple", ...}; the device then
    #                                 returns degrees Celsius instead of volts

    @property
    def scaled(self) -> bool:
        return (bool(self.unit) or self.scale != 1.0 or self.offset != 0.0
                or self.sensor is not None)

    @property
    def raw_unit(self) -> str:
        """Unit of what the device returns for this channel."""
        return SENSOR_UNIT if self.sensor is not None else "V"

    def value(self, volts: float) -> float:
        return volts * self.scale + self.offset


def check_differential(profile: DeviceProfile, channel: str, where: str = "") -> None:
    """A differential reading uses two pins; only the first channels can be its + input."""
    if int(channel[2:]) >= profile.ai_diff_channels:
        raise DaqPolicyError(
            f"{where}{channel} cannot be read differentially on the {profile.model} "
            f"(ai0..ai{profile.ai_diff_channels - 1} can; {channel} is the - input of a pair)")


def profile_from_config(config: Optional[dict]) -> DeviceProfile:
    """The model named in config.yaml (``shared_devices.daq_model``; "" = the default)."""
    try:
        return get_profile((config or {}).get("daq_model"))
    except DaqProfileError as exc:
        raise DaqPolicyError(f"shared_devices.daq_model: {exc}") from None


class DaqSignals:
    """The named signals of ``config.yaml`` (``shared_devices.daq_signals``)."""

    _AI_KEYS = {"terminal", "v_range", "scale", "offset", "unit", "sensor", "tc_type", "cjc_c",
                "rtd_type", "r0_ohm", "excitation_a", "wires", "min_c", "max_c"}
    _SENSOR_KEYS = {"thermocouple": {"tc_type", "cjc_c", "min_c", "max_c"},
                    "rtd": {"rtd_type", "r0_ohm", "excitation_a", "wires", "min_c", "max_c"}}

    def __init__(self, signals: Optional[dict] = None, profile: Optional[DeviceProfile] = None):
        self.profile = profile or USB_6351
        self._signals = {}
        for raw_name, spec in (signals or {}).items():
            name = _clean(raw_name)
            if re.match(RESERVED_PATTERN, name) or not re.match(SIGNAL_NAME_PATTERN, name):
                raise DaqPolicyError(
                    f"daq_signals: {raw_name!r} is not a usable signal name (lower-case letters, "
                    "digits and '_', starting with a letter, and not shaped like a channel)")
            if name in self._signals:
                raise DaqPolicyError(f"daq_signals: {raw_name!r} is defined twice")
            self._signals[name] = self._parse(name, spec, self.profile)

    @staticmethod
    def _parse(name: str, spec, profile: DeviceProfile = USB_6351) -> Signal:
        if isinstance(spec, str):
            spec = {"channel": spec}
        if not isinstance(spec, dict) or not spec.get("channel"):
            raise DaqPolicyError(f"daq_signals.{name}: give a channel, e.g. {{channel: ai0}}")
        channel = _clean(spec["channel"])
        kind = physical_kind(channel)
        if kind is None:
            raise DaqPolicyError(f"daq_signals.{name}: {spec['channel']!r} is not a channel "
                                 "(ai0, ao0, port0/line0, ctr0, pfi0; without the device name)")
        require_on_model(profile, kind, channel, f"daq_signals.{name}: ")
        extra = set(spec) - {"channel"} - (DaqSignals._AI_KEYS if kind == "ai" else set())
        if extra:
            raise DaqPolicyError(f"daq_signals.{name}: unknown or unusable key(s) {sorted(extra)} "
                                 f"for a channel of kind {kind!r}")
        terminal = spec.get("terminal")
        if terminal is not None and terminal not in profile.ai_terminals:
            raise DaqPolicyError(f"daq_signals.{name}: terminal must be one of "
                                 f"{profile.ai_terminals} on the {profile.model}")
        if terminal == "diff":
            check_differential(profile, channel, f"daq_signals.{name}: ")
        v_range = spec.get("v_range")
        if v_range is not None:
            if isinstance(v_range, bool) or float(v_range) not in profile.v_ranges:
                raise DaqPolicyError(f"daq_signals.{name}: v_range must be one of "
                                     f"{profile.v_ranges} on the {profile.model}")
            v_range = float(v_range)
        try:
            scale = float(spec.get("scale", 1.0))
            offset = float(spec.get("offset", 0.0))
        except (TypeError, ValueError):
            raise DaqPolicyError(f"daq_signals.{name}: scale and offset must be numbers") from None
        if not (math.isfinite(scale) and math.isfinite(offset)) or scale == 0.0:
            raise DaqPolicyError(f"daq_signals.{name}: scale must be a finite, non-zero number "
                                 "and offset a finite number")
        sensor = DaqSignals._parse_sensor(name, spec) if kind == "ai" else None
        unit = str(spec.get("unit") or "")
        if sensor is not None:
            if v_range is not None:
                raise DaqPolicyError(f"daq_signals.{name}: v_range does not apply to a sensor "
                                     "channel (give min_c / max_c)")
            unit = unit or SENSOR_UNIT
            try:
                profile.require(THERMOCOUPLE if sensor["type"] == "thermocouple" else RTD)
            except DaqProfileError as exc:
                raise DaqPolicyError(f"daq_signals.{name}: {exc}") from None
        return Signal(label=f"{name}:{channel}", kind=kind, channel=channel, terminal=terminal,
                      v_range=v_range, scale=scale, offset=offset, unit=unit, sensor=sensor)

    @staticmethod
    def _parse_sensor(name: str, spec: dict) -> Optional[dict]:
        kind = spec.get("sensor")
        all_keys = set().union(*DaqSignals._SENSOR_KEYS.values())
        if kind is None:
            stray = sorted(set(spec) & all_keys)
            if stray:
                raise DaqPolicyError(f"daq_signals.{name}: {stray} need sensor: thermocouple or rtd")
            return None
        if kind not in SENSORS:
            raise DaqPolicyError(f"daq_signals.{name}: sensor must be one of {SENSORS}")
        stray = sorted((set(spec) & all_keys) - DaqSignals._SENSOR_KEYS[kind])
        if stray:
            raise DaqPolicyError(f"daq_signals.{name}: {stray} do not apply to a {kind}")

        def number(key, default=None):
            value = spec.get(key, default)
            try:
                value = float(value)
            except (TypeError, ValueError):
                value = float("nan")
            if isinstance(spec.get(key, default), bool) or not math.isfinite(value):
                raise DaqPolicyError(f"daq_signals.{name}: {key} must be a number")
            return value

        low, high = number("min_c", 0.0), number("max_c", 200.0)
        if not low < high:
            raise DaqPolicyError(f"daq_signals.{name}: min_c must be below max_c")
        sensor = {"type": kind, "min_c": low, "max_c": high}
        if kind == "thermocouple":
            tc_type = str(spec.get("tc_type") or "").upper()
            if tc_type not in TC_TYPES:
                raise DaqPolicyError(f"daq_signals.{name}: tc_type must be one of {TC_TYPES}")
            if "cjc_c" not in spec:
                raise DaqPolicyError(
                    f"daq_signals.{name}: give cjc_c, the temperature of the terminal block "
                    "(the device has no cold-junction sensor; an error there is an error "
                    "of the reading)")
            sensor.update(tc_type=tc_type, cjc_c=number("cjc_c"))
        else:
            rtd_type = str(spec.get("rtd_type") or "pt3851").lower()
            if rtd_type not in RTD_TYPES:
                raise DaqPolicyError(f"daq_signals.{name}: rtd_type must be one of {RTD_TYPES}")
            wires = spec.get("wires", 4)
            if isinstance(wires, bool) or wires not in (2, 3, 4):
                raise DaqPolicyError(f"daq_signals.{name}: wires must be 2, 3 or 4")
            if "excitation_a" not in spec:
                raise DaqPolicyError(
                    f"daq_signals.{name}: give excitation_a, the current of the external "
                    "source through the RTD (the device supplies none)")
            r0, current = number("r0_ohm", 100.0), number("excitation_a")
            if not (r0 > 0.0 and 0.0 < current <= 0.01):
                raise DaqPolicyError(f"daq_signals.{name}: r0_ohm must be > 0 and excitation_a "
                                     "in (0, 0.01] A")
            sensor.update(rtd_type=rtd_type, r0_ohm=r0, excitation_a=current, wires=int(wires))
        return sensor

    @classmethod
    def from_config(cls, config: dict, profile: Optional[DeviceProfile] = None) -> "DaqSignals":
        signals = (config or {}).get("daq_signals") or {}
        if not isinstance(signals, dict):
            raise DaqPolicyError("shared_devices.daq_signals must be a mapping of name -> channel")
        return cls(signals, profile)

    def names(self) -> list:
        return sorted(self._signals)

    def resolve(self, name, kinds) -> Signal:
        """Resolve a channel or signal name used in a flow; ``kinds`` is what the step accepts."""
        kinds = (kinds,) if isinstance(kinds, str) else tuple(kinds)
        text = _clean(name)
        kind = physical_kind(text)
        if kind is not None:
            require_on_model(self.profile, kind, text)
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
                 co_counters: Iterable = (), co_max_hz: float = 1000.0,
                 profile: Optional[DeviceProfile] = None):
        self.profile = p = profile or USB_6351
        self.ao_channels = frozenset(
            require_on_model(p, "ao", normalize_ao(c), "daq_ao_channels: ")
            for c in (ao_channels or ()))
        self.do_lines = frozenset(
            require_on_model(p, "line", normalize_line(c), "daq_do_lines: ")
            for c in (do_lines or ()))
        self.co_counters = frozenset(
            require_on_model(p, "counter", normalize_counter(c), "daq_co_counters: ")
            for c in (co_counters or ()))
        if self.co_counters and not p.co_max_hz > 0.0:
            raise DaqPolicyError(f"daq_co_counters: the {p.model} cannot output pulse trains; "
                                 "leave the list empty")
        self.ao_min_v = float(ao_min_v)
        self.ao_max_v = float(ao_max_v)
        self.co_max_hz = float(co_max_hz)
        if not (p.ao_min_v <= self.ao_min_v <= self.ao_max_v <= p.ao_max_v):
            raise DaqPolicyError(
                f"daq_ao_min_v / daq_ao_max_v must satisfy {p.ao_min_v:g} <= min <= max <= "
                f"{p.ao_max_v:g} V, got {self.ao_min_v:g} .. {self.ao_max_v:g}")
        # a model without counter output ignores the frequency limit (nothing can use it)
        limit = p.co_max_hz if p.co_max_hz > 0.0 else USB_6351.co_max_hz
        if not (0.0 < self.co_max_hz <= limit):
            raise DaqPolicyError(
                f"daq_co_max_hz must be in (0, {limit:g}] Hz, got {self.co_max_hz:g}")

    @classmethod
    def from_config(cls, config: dict, profile: Optional[DeviceProfile] = None) -> "DaqOutputPolicy":
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
                   co_max_hz=config.get("daq_co_max_hz", 1000.0),
                   profile=profile)

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

    def check_digital_pattern(self, lines) -> list:
        """Every line of a hardware-timed pattern must be enabled and on port 0."""
        names = [self.check_digital_output(line) for line in lines]
        return check_timed_lines(names, self.profile)

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
