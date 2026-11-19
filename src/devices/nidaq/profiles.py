"""What each supported NI DAQ model has: one table entry per model.

The driver, the mock, the steps and the flows are the same for every model. What
differs between a USB-6351 and a USB-6002 (how many channels, how fast, which
measurements exist at all) is written down here once, as a ``DeviceProfile``.

- The real driver picks the profile from the product type the device reports.
- The mock picks it from ``shared_devices.daq_model`` in config.yaml, so ``--mock``
  refuses what the lab's device could not do.
- A step the model does not support is refused with the model's name before anything
  is sent to the device.

Adding a model is: one ``DeviceProfile`` below, taken from the specification and from
``python -m src.devices.nidaq.probe`` (which only asks the device what it has), then
the bench checks in ``src/devices/nidaq/README.md``. That is enough for a model that
fits what the driver assumes (listed in that README: channels numbered from 0, the
first channels differential, 32-bit counters, one analog-output sample clock, outputs
whose safe state is 0 V / low); a model that differs there needs a new profile field
or driver code.
"""

import re
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple

KINDS = ("ai", "ao", "line", "counter", "terminal")

# Features a step (or an option of a step) may need. Everything not listed here
# (single readings and writes, waveform acquisition, continuous recording, waveform
# output, software-timed pulses, condition waits, edge counting) is expected of
# every supported model.
ANALOG_TRIGGER = "analog_trigger"          # daq_acquire trigger_level
PRETRIGGER = "pretrigger"                  # daq_acquire pretrigger_s (reference trigger)
EXTERNAL_CLOCK = "external_clock"          # clock= of an acquisition
SYNCHRONISED_AO_AI = "synchronised_ao_ai"  # daq_stimulate_acquire
TIMED_DIGITAL = "timed_digital"            # clocked digital acquisition and patterns
CHANGE_DETECTION = "change_detection"      # daq_wait_digital_change
FREQUENCY = "frequency_measurement"        # daq_measure_frequency
TIME_MEASUREMENT = "time_measurement"      # pulse width, period, edge separation
ENCODER = "encoder"                        # daq_*_encoder
COUNTER_OUTPUT = "counter_output"          # daq_generate_pulses
THERMOCOUPLE = "thermocouple"              # daq_signals: sensor: thermocouple
RTD = "rtd"                                # daq_signals: sensor: rtd

FEATURE_LABEL = {
    ANALOG_TRIGGER: "an analog trigger (trigger_level)",
    PRETRIGGER: "pre-trigger samples (pretrigger_s)",
    EXTERNAL_CLOCK: "an external sample clock (clock)",
    SYNCHRONISED_AO_AI: "an acquisition started by the analog output (daq_stimulate_acquire)",
    TIMED_DIGITAL: "hardware-timed digital I/O",
    CHANGE_DETECTION: "digital change detection",
    FREQUENCY: "frequency measurement with a counter",
    TIME_MEASUREMENT: "pulse-width / period / edge-separation measurement",
    ENCODER: "a quadrature encoder input",
    COUNTER_OUTPUT: "pulse-train output",
    THERMOCOUPLE: "thermocouple channels",
    RTD: "RTD channels",
}

#: The feature a step needs as a whole.
ACTION_FEATURES = {
    "daq_stimulate_acquire": SYNCHRONISED_AO_AI,
    "daq_acquire_digital": TIMED_DIGITAL,
    "daq_write_digital_pattern": TIMED_DIGITAL,
    "daq_wait_digital_change": CHANGE_DETECTION,
    "daq_measure_frequency": FREQUENCY,
    "daq_measure_pulse_width": TIME_MEASUREMENT,
    "daq_measure_period": TIME_MEASUREMENT,
    "daq_measure_edge_separation": TIME_MEASUREMENT,
    "daq_start_encoder": ENCODER,
    "daq_read_encoder": ENCODER,
    "daq_stop_encoder": ENCODER,
    "daq_generate_pulses": COUNTER_OUTPUT,
}

#: The feature an option of a step needs when it is given (not None).
OPTION_FEATURES = {
    ("daq_acquire", "trigger_level"): ANALOG_TRIGGER,
    ("daq_acquire", "pretrigger_s"): PRETRIGGER,
    ("daq_acquire", "clock"): EXTERNAL_CLOCK,
}


class DaqProfileError(ValueError):
    """An unknown model, or something the model does not have."""


@dataclass(frozen=True)
class DeviceProfile:
    """The terminals, limits and measurements of one model."""

    model: str                               # as DAQmx reports it ("USB-6351")
    ai_channels: int                         # ai0 .. ai(n-1)
    ai_diff_channels: int                    # ai0 .. ai(n-1) can be read differentially
    ai_terminals: Tuple[str, ...]            # wiring modes ("default", "rse", "nrse", "diff")
    v_ranges: Tuple[float, ...]              # input ranges, +/- V
    ai_max_rate_single_hz: float             # one channel
    ai_max_rate_aggregate_hz: float          # several channels, summed over the channels
    ao_channels: int
    ao_min_v: float
    ao_max_v: float
    ao_max_rate_hz: float                    # waveform generation
    port_lines: Tuple[int, ...]              # lines of port0, port1, ...
    counters: int                            # ctr0 .. ctr(n-1)
    pfi: int                                 # pfi0 .. pfi(n-1)
    internal_terminals: Tuple[str, ...] = ()  # signals inside the device (lower case)
    timed_port: Optional[str] = None         # the port with hardware timing, if any
    dio_max_rate_hz: float = 0.0             # hardware-timed digital I/O
    co_max_hz: float = 0.0                   # pulse trains (0: no counter output)
    #: The pin a counter's pulse train comes out of is also a digital line.
    counter_output_lines: Dict[str, str] = field(default_factory=dict)
    #: A PFI terminal is the same pin as a digital line ("pfi0" -> "port2/line0").
    pfi_lines: Dict[str, str] = field(default_factory=dict)
    features: frozenset = frozenset()
    notes: str = ""

    def __post_init__(self):
        # A profile that contradicts itself would silently switch a check off.
        if self.co_max_hz > 0.0 and set(self.counter_output_lines) != set(self.counter_names):
            raise ValueError(f"{self.model}: counter_output_lines must name the output pin "
                             "of every counter when the model has counter output")
        if set(self.pfi_lines) != {f"pfi{i}" for i in range(self.pfi)}:
            raise ValueError(f"{self.model}: pfi_lines must name the digital line of every PFI")
        for line in list(self.counter_output_lines.values()) + list(self.pfi_lines.values()):
            if self.kind_of(line) != "line":
                raise ValueError(f"{self.model}: {line} is not a digital line of this model")
        if (self.timed_port is None) != (not self.dio_max_rate_hz > 0.0):
            raise ValueError(f"{self.model}: timed_port and dio_max_rate_hz go together")
        if not (0 < self.ai_diff_channels <= self.ai_channels and self.v_ranges
                and "default" in self.ai_terminals):
            raise ValueError(f"{self.model}: inconsistent analog input description")

    # -- names ----------------------------------------------------------
    def kind_of(self, name) -> Optional[str]:
        """The kind of a physical channel this model has ("ai0" -> "ai"), else None."""
        text = str(name).strip().lower()
        m = re.match(r"^ai(0|[1-9]\d?)$", text)
        if m:
            return "ai" if int(m.group(1)) < self.ai_channels else None
        m = re.match(r"^ao(\d)$", text)
        if m:
            return "ao" if int(m.group(1)) < self.ao_channels else None
        m = re.match(r"^port(\d)/line(0|[1-9]\d?)$", text)
        if m:
            port, line = int(m.group(1)), int(m.group(2))
            return "line" if port < len(self.port_lines) and line < self.port_lines[port] else None
        m = re.match(r"^ctr(\d)$", text)
        if m:
            return "counter" if int(m.group(1)) < self.counters else None
        m = re.match(r"^pfi(0|[1-9]\d?)$", text)
        if m:
            return "terminal" if int(m.group(1)) < self.pfi else None
        return "terminal" if text in self.internal_terminals else None

    def has(self, kind: str, name) -> bool:
        return self.kind_of(name) == kind

    def available(self, kind: str) -> str:
        """What the model has of one kind, for an error message."""
        def span(prefix, n):
            return "none" if n < 1 else (f"{prefix}0" if n == 1 else f"{prefix}0..{prefix}{n - 1}")
        if kind == "ai":
            return span("ai", self.ai_channels)
        if kind == "ao":
            return span("ao", self.ao_channels)
        if kind == "counter":
            return span("ctr", self.counters)
        if kind == "line":
            return ", ".join(f"port{p}/line0..{n - 1}" if n > 1 else f"port{p}/line0"
                             for p, n in enumerate(self.port_lines)) or "none"
        return ", ".join(filter(None, [span("pfi", self.pfi)] + list(self.internal_terminals)))

    @property
    def counter_names(self) -> Tuple[str, ...]:
        return tuple(f"ctr{i}" for i in range(self.counters))

    # -- features -------------------------------------------------------
    def supports(self, feature: str) -> bool:
        return feature in self.features

    def require(self, feature: str) -> None:
        if feature not in self.features:
            raise DaqProfileError(f"the {self.model} does not support "
                                  f"{FEATURE_LABEL.get(feature, feature)}")

    def check_step(self, action: str, params: Optional[dict] = None) -> None:
        """Raise DaqProfileError when this model cannot run the step as written."""
        feature = ACTION_FEATURES.get(action)
        if feature is not None and feature not in self.features:
            raise DaqProfileError(f"{action}: the {self.model} does not support "
                                  f"{FEATURE_LABEL[feature]}")
        for (act, option), feature in OPTION_FEATURES.items():
            if act == action and (params or {}).get(option) is not None \
                    and feature not in self.features:
                raise DaqProfileError(f"{action}: the {self.model} does not support "
                                      f"{FEATURE_LABEL[feature]}")

    def ai_rate_limit(self, n_channels: int) -> float:
        return (self.ai_max_rate_single_hz if n_channels == 1
                else self.ai_max_rate_aggregate_hz / n_channels)


USB_6351 = DeviceProfile(
    model="USB-6351",
    ai_channels=16, ai_diff_channels=8,
    ai_terminals=("default", "rse", "nrse", "diff"),
    v_ranges=(0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0),
    ai_max_rate_single_hz=1_250_000.0,
    ai_max_rate_aggregate_hz=1_000_000.0,
    ao_channels=2, ao_min_v=-10.0, ao_max_v=10.0,
    ao_max_rate_hz=2_000_000.0,          # device: 2.86 MS/s on one channel; kept below it
    port_lines=(8, 8, 8),
    counters=4, pfi=16,
    internal_terminals=("100khztimebase", "20mhztimebase"),
    timed_port="port0",
    dio_max_rate_hz=1_000_000.0,
    co_max_hz=1_000_000.0,               # the 100 MHz timebase goes higher
    counter_output_lines={"ctr0": "port2/line4", "ctr1": "port2/line5",
                          "ctr2": "port2/line6", "ctr3": "port2/line7"},
    # PFI 0-7 = P1.0-7, PFI 8-15 = P2.0-7
    pfi_lines={**{f"pfi{i}": f"port1/line{i}" for i in range(8)},
               **{f"pfi{i + 8}": f"port2/line{i}" for i in range(8)}},
    features=frozenset({ANALOG_TRIGGER, PRETRIGGER, EXTERNAL_CLOCK, SYNCHRONISED_AO_AI,
                        TIMED_DIGITAL, CHANGE_DETECTION, FREQUENCY, TIME_MEASUREMENT,
                        ENCODER, COUNTER_OUTPUT, THERMOCOUPLE, RTD}),
    notes="X Series. Bridge (strain) channels are refused by the device.",
)

USB_6002 = DeviceProfile(
    model="USB-6002",
    ai_channels=8, ai_diff_channels=4,
    ai_terminals=("default", "rse", "diff"),
    v_ranges=(10.0,),                    # one fixed range
    ai_max_rate_single_hz=50_000.0,
    ai_max_rate_aggregate_hz=50_000.0,
    ao_channels=2, ao_min_v=-10.0, ao_max_v=10.0,
    ao_max_rate_hz=5_000.0,
    port_lines=(8, 4, 1),
    counters=1, pfi=2,
    pfi_lines={"pfi0": "port2/line0", "pfi1": "port1/line1"},   # P2.0/PFI 0, P1.1/PFI 1
    # Refused by the device (asked with task verification, 2026-11-16): NRSE wiring,
    # an analog or reference trigger, an external sample clock, any counter measurement
    # but edge counting; it has no timed digital I/O and no counter output. Thermocouple
    # and RTD channels can be created, but one count of its 16 bits over +/-10 V is
    # 305 uV (about 7 K of a type K thermocouple): read an amplified sensor as a
    # voltage with a scale instead.
    features=frozenset({SYNCHRONISED_AO_AI}),
    notes="Low-cost multifunction device: software-timed digital I/O, one edge counter, "
          "digital start trigger on PFI0 / PFI1 only, no counter output.",
)

PROFILES: Dict[str, DeviceProfile] = {p.model: p for p in (USB_6351, USB_6002)}
DEFAULT_MODEL = USB_6351.model


def _key(model) -> str:
    return re.sub(r"[^a-z0-9]", "", str(model).lower())


def get_profile(model=None) -> DeviceProfile:
    """The profile of a model name ("USB-6351", "usb6351", "NI USB-6002"); None or ""
    gives the default model."""
    if model is None or not str(model).strip():
        return PROFILES[DEFAULT_MODEL]      # for config.yaml; a device always names its model
    key = _key(model)
    key = key[2:] if key.startswith("ni") else key
    for name, profile in PROFILES.items():
        if _key(name) == key:
            return profile
    raise DaqProfileError(
        f"NI DAQ model {model!r} has no profile (supported: {', '.join(sorted(PROFILES))}). "
        "Its channel counts and limits are not known here; add a DeviceProfile to "
        "src/devices/nidaq/profiles.py (see the README of that folder)")


def any_kind_of(name) -> Optional[str]:
    """The kind of a physical channel name on any supported model (flows are checked
    against the schema before the device, and so the model, is known)."""
    for profile in PROFILES.values():
        kind = profile.kind_of(name)
        if kind is not None:
            return kind
    return None
