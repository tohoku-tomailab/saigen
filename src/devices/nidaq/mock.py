"""A DAQ device without hardware: the same methods as ``NIDaq``, deterministic answers.

Used by ``--mock`` runs and by the tests. It keeps what was driven (``analog_outputs``,
``digital_outputs``, ``waveforms``, ``pulse_trains``) so a test can assert on it, applies
the same argument checks as the real driver, and never waits.
"""

import time
from typing import Dict, List, Optional, Sequence

from .nidaq import COUNTERS, check_ai_rate
from .policy import (AO_MAX_RATE_HZ, COUNTER_OUTPUT_LINES, DEVICE_AO_MAX_V, DEVICE_AO_MIN_V, EDGES, TERMINALS,
                     normalize_ai, normalize_ao, normalize_counter, normalize_line,
                     normalize_terminal, physical_kind)


class MockNIDaq:
    #: What every analog input reads (V).
    MOCK_VOLTAGE = 1.234
    #: The signal every counter input sees (Hz).
    MOCK_FREQUENCY_HZ = 1000.0

    def __init__(self, device: str = "MockDev"):
        self.device = device
        self.product_type = "USB-6351 (mock)"
        self.analog_outputs: Dict[str, float] = {}      # {"ao0": 2.5}; 0.0 after a stop
        self.digital_outputs: Dict[str, bool] = {}      # {"port0/line0": True}
        self.waveforms: Dict[str, dict] = {}            # running: {"ao0": {"points":.., "rate_hz":..}}
        self.pulse_trains: Dict[str, dict] = {}         # running: {"ctr0": {"frequency_hz":..}}
        self._stream = None
        self.ai_rate_hz: Optional[float] = None
        self._closed = False

    def _require_open(self) -> None:
        if self._closed:
            raise RuntimeError("the DAQ has been closed (the run is over); no output is driven")

    def _require_pin_free(self, name: str) -> None:
        for counter, pin in COUNTER_OUTPUT_LINES.items():
            if pin == name and counter in self.pulse_trains:
                raise RuntimeError(f"{name} is the output pin of {counter}, which is generating "
                                   "a pulse train; stop it first")

    def list_devices(self) -> List[str]:
        return [self.device]

    def connect(self) -> bool:
        self._closed = False
        return True

    # -- analog input ---------------------------------------------------
    def _check_ai(self, specs) -> List[str]:
        names = []
        for channel, v_range, terminal in specs:
            name = normalize_ai(channel)
            if name in names:
                raise ValueError(f"{name} is listed twice")
            if terminal not in TERMINALS:
                raise ValueError(f"terminal must be one of {TERMINALS}, got {terminal!r}")
            if not (0 < float(v_range) <= 10.0):
                raise ValueError("v_range must be in (0, 10] V")
            names.append(name)
        if not names:
            raise ValueError("at least one analog input channel is needed")
        return names

    def _require_ai_free(self) -> None:
        if self._stream is not None:
            raise RuntimeError(
                "the analog inputs are busy with a continuous acquisition (daq_start_logging); "
                "stop it first, or add the channel to the logged channels")

    def read_analog(self, channel: str, samples: int = 1, rate_hz: float = 1000.0,
                    v_range: float = 10.0, terminal: str = "default") -> float:
        if int(samples) < 1:
            raise ValueError("samples must be >= 1")
        self._check_ai([(channel, v_range, terminal)])
        self._require_ai_free()
        return self.MOCK_VOLTAGE

    def acquire(self, specs: Sequence, rate_hz: float, samples: int,
                trigger: Optional[str] = None, trigger_edge: str = "rising",
                timeout_s: float = 10.0) -> List[List[float]]:
        if int(samples) < 2:
            raise ValueError("samples must be >= 2")
        check_ai_rate(len(specs), rate_hz)
        names = self._check_ai(specs)
        self._require_ai_free()
        if trigger is not None:
            normalize_terminal(trigger)
            if trigger_edge not in EDGES:
                raise ValueError(f"edge must be one of {EDGES}, got {trigger_edge!r}")
        self.ai_rate_hz = float(rate_hz)
        return [[self.MOCK_VOLTAGE] * int(samples) for _ in names]

    def start_stream(self, specs: Sequence, rate_hz: float) -> None:
        check_ai_rate(len(specs), rate_hz)
        names = self._check_ai(specs)
        self._require_ai_free()
        self.ai_rate_hz = float(rate_hz)
        self._stream = {"channels": len(names), "rate_hz": float(rate_hz),
                        "t0": time.monotonic(), "delivered": 0}

    def read_stream(self) -> List[List[float]]:
        if self._stream is None:
            raise RuntimeError("no continuous acquisition is running")
        stream = self._stream
        # as many samples as the wall clock says, and at least one per call
        due = max(int((time.monotonic() - stream["t0"]) * stream["rate_hz"]),
                  stream["delivered"] + 1)
        count = min(due - stream["delivered"], 100000)
        stream["delivered"] += count
        return [[self.MOCK_VOLTAGE] * count for _ in range(stream["channels"])]

    def stop_stream(self) -> None:
        self._stream = None

    # -- analog output --------------------------------------------------
    def write_analog(self, channel: str, voltage: float) -> float:
        self._require_open()
        return self._set_ao(channel, voltage)

    def _set_ao(self, channel: str, voltage: float) -> float:
        name = normalize_ao(channel)
        volts = float(voltage)
        if not (DEVICE_AO_MIN_V <= volts <= DEVICE_AO_MAX_V):
            raise ValueError(f"{name}: {volts:g} V is outside the device span "
                             f"{DEVICE_AO_MIN_V:g} .. {DEVICE_AO_MAX_V:g} V")
        self.waveforms.pop(name, None)
        self.analog_outputs[name] = volts
        return volts

    def start_waveform(self, channel: str, period: Sequence[float], rate_hz: float) -> float:
        name = normalize_ao(channel)
        points = [float(v) for v in period]
        if len(points) < 2:
            raise ValueError("a waveform needs at least 2 points")
        if any(v != v or not (DEVICE_AO_MIN_V <= v <= DEVICE_AO_MAX_V) for v in points):
            raise ValueError(f"{name}: the waveform leaves the device span "
                             f"{DEVICE_AO_MIN_V:g} .. {DEVICE_AO_MAX_V:g} V")
        if not (0.0 < float(rate_hz) <= AO_MAX_RATE_HZ):
            raise ValueError(f"rate_hz = {float(rate_hz):g} is outside 0 .. {AO_MAX_RATE_HZ:g} Hz")
        self._require_open()
        others = sorted(set(self.waveforms) - {name})
        if others:
            raise RuntimeError(f"{others[0]} is generating a waveform; the device has one "
                               "analog-output sample clock, so stop it first")
        self.waveforms[name] = {"points": points, "rate_hz": float(rate_hz)}
        self.analog_outputs[name] = points[-1]
        return float(rate_hz)

    # -- digital --------------------------------------------------------
    def write_digital(self, line: str, state: bool) -> bool:
        self._require_open()
        name = normalize_line(line)
        self._require_pin_free(name)
        self.digital_outputs[name] = bool(state)
        return bool(state)

    def read_digital(self, line: str) -> bool:
        name = normalize_line(line)
        if name in self.digital_outputs:
            # same as the real driver: a driven line cannot be read as an input
            raise RuntimeError(f"{name} is being driven as an output in this session; "
                               "reading it as an input would release it")
        self._require_pin_free(name)
        return False

    # -- counters -------------------------------------------------------
    def _input_counter(self, counter: Optional[str]) -> str:
        if counter is not None:
            name = normalize_counter(counter)
            if name not in COUNTERS:
                raise ValueError(f"{name}: the device has {COUNTERS}")
            if name in self.pulse_trains:
                raise RuntimeError(f"{name} is generating a pulse train; stop it first "
                                   "or use another counter")
            return name
        for name in reversed(COUNTERS):
            if name not in self.pulse_trains:
                return name
        raise RuntimeError("all four counters are generating pulse trains")

    def count_edges(self, terminal: str, duration_s: float, edge: str = "rising",
                    counter: Optional[str] = None, sleep=None) -> int:
        if not float(duration_s) > 0.0:
            raise ValueError("duration_s must be > 0")
        if edge not in EDGES:
            raise ValueError(f"edge must be one of {EDGES}, got {edge!r}")
        normalize_terminal(terminal)
        self._input_counter(counter)
        return int(round(float(duration_s) * self.MOCK_FREQUENCY_HZ))

    def measure_frequency(self, terminal: str, min_hz: float = 1.0, max_hz: float = 100000.0,
                          timeout_s: float = 5.0, edge: str = "rising",
                          counter: Optional[str] = None) -> float:
        if not (0.0 < float(min_hz) < float(max_hz)):
            raise ValueError("min_hz and max_hz must satisfy 0 < min_hz < max_hz")
        if edge not in EDGES:
            raise ValueError(f"edge must be one of {EDGES}, got {edge!r}")
        normalize_terminal(terminal)
        self._input_counter(counter)
        return min(max(self.MOCK_FREQUENCY_HZ, float(min_hz)), float(max_hz))

    def start_pulses(self, counter: str, frequency_hz: float, duty_cycle: float = 0.5,
                     count: Optional[int] = None) -> None:
        name = normalize_counter(counter)
        if name not in COUNTERS:
            raise ValueError(f"{name}: the device has {COUNTERS}")
        if not float(frequency_hz) > 0.0 or not (0.0 < float(duty_cycle) < 1.0):
            raise ValueError("frequency_hz must be > 0 and duty_cycle between 0 and 1")
        if count is not None and int(count) < 1:
            raise ValueError("count must be >= 1")
        self._require_open()
        if COUNTER_OUTPUT_LINES[name] in self.digital_outputs:
            raise RuntimeError(f"the output pin of {name} ({COUNTER_OUTPUT_LINES[name]}) has "
                               "been driven as a digital line in this run; use another counter")
        self.pulse_trains[name] = {"frequency_hz": float(frequency_hz),
                                   "duty_cycle": float(duty_cycle), "count": count}

    def pulses_done(self, counter: str) -> bool:
        return True     # a finite train is over at once in the mock

    # -- safe state -----------------------------------------------------
    def stop_output(self, channel: str) -> bool:
        name = str(channel).strip().lower()
        kind = physical_kind(name)
        if kind == "ao":
            if name not in self.analog_outputs:
                return False
            self._set_ao(name, 0.0)
            return True
        if kind == "line":
            if name not in self.digital_outputs:
                return False
            self.digital_outputs[name] = False
            return True
        if kind == "counter":
            return self.pulse_trains.pop(name, None) is not None
        raise ValueError(f"{channel!r} is not an output channel (ao0, port0/line0, ctr0)")

    def reset_outputs(self) -> List[str]:
        self.pulse_trains.clear()
        self.waveforms.clear()
        for name in self.analog_outputs:
            self.analog_outputs[name] = 0.0
        for name in self.digital_outputs:
            self.digital_outputs[name] = False
        return []

    def close(self) -> List[str]:
        self._closed = True
        self.stop_stream()
        return self.reset_outputs()
