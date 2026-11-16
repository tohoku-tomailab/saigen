"""A DAQ device without hardware: the same methods as ``NIDaq``, deterministic answers.

Used by ``--mock`` runs and by the tests. It keeps what was driven (``analog_outputs``,
``digital_outputs``, ``waveforms``, ``pulse_trains``) so a test can assert on it, applies
the same argument checks as the real driver, and never waits.
"""

import time
from typing import Dict, List, Optional, Sequence

import threading

from .nidaq import COUNTERS, _locked, check_ai_rate
from .policy import (AO_MAX_RATE_HZ, COUNTER_OUTPUT_LINES, DIO_MAX_RATE_HZ, ENCODER_DECODINGS,
                     check_timed_lines, DEVICE_AO_MAX_V, DEVICE_AO_MIN_V, EDGES, TERMINALS,
                     normalize_ai, normalize_ao, normalize_counter, normalize_line,
                     normalize_terminal, physical_kind)


class MockNIDaq:
    #: What every analog input reads (V).
    MOCK_VOLTAGE = 1.234
    #: The signal every counter input sees (Hz).
    MOCK_FREQUENCY_HZ = 1000.0
    #: What every sensor channel (thermocouple, RTD) reads (degrees Celsius).
    MOCK_TEMPERATURE_C = 25.0
    #: What the encoder reads (degrees).
    MOCK_ANGLE_DEG = 90.0

    def __init__(self, device: str = "MockDev"):
        self.device = device
        self.product_type = "USB-6351 (mock)"
        self.analog_outputs: Dict[str, float] = {}      # {"ao0": 2.5}; 0.0 after a stop
        self.digital_outputs: Dict[str, bool] = {}      # {"port0/line0": True}
        self.waveforms: Dict[str, dict] = {}            # running: {"ao0": {"points":.., "rate_hz":..}}
        self.pulse_trains: Dict[str, dict] = {}         # running: {"ctr0": {"frequency_hz":..}}
        self.patterns: List[dict] = []                  # digital patterns played
        self.stimuli: List[dict] = []                   # stimulate_acquire calls
        self.encoder: Optional[dict] = None             # the encoder being followed
        self._stream = None
        self._stream_id = 0
        self.ai_rate_hz: Optional[float] = None
        self.di_rate_hz: Optional[float] = None
        self.do_rate_hz: Optional[float] = None
        self._closed = False
        self._lock = threading.RLock()      # same serialisation as the real driver

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
        return not self._closed         # one object serves one run

    # -- analog input ---------------------------------------------------
    def _reading(self, spec) -> float:
        """What one channel reads: volts, or degrees Celsius for a sensor channel."""
        sensor = spec[3] if len(spec) > 3 else None
        return self.MOCK_VOLTAGE if sensor is None else self.MOCK_TEMPERATURE_C

    def _check_ai(self, specs) -> List[str]:
        names = []
        for spec in specs:
            channel, v_range, terminal = spec[:3]
            sensor = spec[3] if len(spec) > 3 else None
            if sensor is not None and sensor.get("type") not in ("thermocouple", "rtd"):
                raise ValueError(f"unknown sensor type {sensor.get('type')!r}")
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
                    v_range: float = 10.0, terminal: str = "default",
                    sensor: Optional[dict] = None) -> float:
        if int(samples) < 1:
            raise ValueError("samples must be >= 1")
        spec = (channel, v_range, terminal, sensor)
        self._check_ai([spec])
        self._require_ai_free()
        return self._reading(spec)

    def acquire(self, specs: Sequence, rate_hz: float, samples: int,
                trigger: Optional[str] = None, trigger_edge: str = "rising",
                timeout_s: float = 10.0, trigger_level: Optional[float] = None,
                pretrigger: int = 0, clock: Optional[str] = None) -> List[List[float]]:
        samples = int(samples)
        if samples < 2:
            raise ValueError("samples must be >= 2")
        check_ai_rate(len(specs), rate_hz)
        pretrigger = int(pretrigger)
        if pretrigger and trigger is None:
            raise ValueError("pretrigger samples need a trigger")
        if pretrigger and not (2 <= pretrigger <= samples - 2):
            raise ValueError("pretrigger must leave at least 2 samples before and after the trigger")
        if trigger_level is not None and trigger is None:
            raise ValueError("trigger_level needs the analog input to trigger on")
        specs = list(specs)
        names = self._check_ai(specs)
        self._require_ai_free()
        if trigger is not None:
            if trigger_edge not in EDGES:
                raise ValueError(f"edge must be one of {EDGES}, got {trigger_edge!r}")
            if trigger_level is not None:
                if normalize_ai(trigger) != names[0]:
                    raise ValueError("an analog trigger must be the first channel acquired "
                                     f"({names[0]}), got {trigger}")
            else:
                normalize_terminal(trigger)
        if clock is not None:
            normalize_terminal(clock)
        self.ai_rate_hz = float(rate_hz)
        return [[self._reading(spec)] * samples for spec in specs]

    @_locked
    def start_stream(self, specs: Sequence, rate_hz: float) -> int:
        check_ai_rate(len(specs), rate_hz)
        self._require_open()
        specs = list(specs)
        names = self._check_ai(specs)
        self._require_ai_free()
        self.ai_rate_hz = float(rate_hz)
        self._stream = {"channels": len(names), "rate_hz": float(rate_hz),
                        "readings": [self._reading(spec) for spec in specs],
                        "id": self._stream_id + 1,
                        "t0": time.monotonic(), "delivered": 0}
        self._stream_id += 1
        return self._stream_id

    @_locked
    def read_stream(self, stream: Optional[int] = None) -> List[List[float]]:
        if self._stream is None or (stream is not None and stream != self._stream_id):
            raise RuntimeError("no continuous acquisition is running"
                               if self._stream is None else
                               "that continuous acquisition is over; another one is running")
        stream = self._stream
        # as many samples as the wall clock says, and at least one per call
        due = max(int((time.monotonic() - stream["t0"]) * stream["rate_hz"]),
                  stream["delivered"] + 1)
        count = min(due - stream["delivered"], 100000)
        stream["delivered"] += count
        return [[value] * count for value in stream["readings"]]

    def stop_stream(self) -> None:
        self._stream = None

    # -- analog output --------------------------------------------------
    @_locked
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

    @_locked
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
    @_locked
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
            if self.encoder is not None and self.encoder["counter"] == name:
                raise RuntimeError(f"{name} is following the encoder; use another counter")
            return name
        busy = set(self.pulse_trains) | ({self.encoder["counter"]} if self.encoder else set())
        for name in reversed(COUNTERS):
            if name not in busy:
                return name
        raise RuntimeError("all four counters are in use")

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

    @_locked
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
        if self.encoder is not None and self.encoder["counter"] == name:
            raise RuntimeError(f"{name} is following the encoder; use another counter")
        if COUNTER_OUTPUT_LINES[name] in self.digital_outputs:
            raise RuntimeError(f"the output pin of {name} ({COUNTER_OUTPUT_LINES[name]}) has "
                               "been driven as a digital line in this run; use another counter")
        self.pulse_trains[name] = {"frequency_hz": float(frequency_hz),
                                   "duty_cycle": float(duty_cycle), "count": count}

    def pulses_done(self, counter: str) -> bool:
        return True     # a finite train is over at once in the mock

    # -- stimulus and response -------------------------------------------
    def stimulate_acquire(self, channel: str, period: Sequence[float], ao_rate_hz: float,
                          specs: Sequence, rate_hz: float, samples: int):
        if int(samples) < 2:
            raise ValueError("samples must be >= 2")
        check_ai_rate(len(specs), rate_hz)
        self._require_ai_free()
        self._require_open()
        specs = list(specs)
        names = self._check_ai(specs)
        self.ai_rate_hz = float(rate_hz)
        actual = self.start_waveform(channel, period, ao_rate_hz)
        self.stimuli.append({"channel": normalize_ao(channel), "points": len(list(period)),
                             "rate_hz": float(ao_rate_hz), "inputs": names})
        self.stop_output(normalize_ao(channel))
        return [[self._reading(spec)] * int(samples) for spec in specs], actual

    # -- hardware-timed digital I/O ---------------------------------------
    def _check_di(self, lines) -> List[str]:
        names = check_timed_lines(lines)
        for name in names:
            if name in self.digital_outputs:
                raise RuntimeError(f"{name} is being driven as an output in this session; "
                                   "reading it as an input would release it")
        return names

    def acquire_digital(self, lines: Sequence[str], rate_hz: float, samples: int,
                        clock: Optional[str] = None) -> List[List[bool]]:
        if int(samples) < 2:
            raise ValueError("samples must be >= 2")
        if not (0.0 < float(rate_hz) <= DIO_MAX_RATE_HZ):
            raise ValueError(f"rate_hz = {float(rate_hz):g} is outside 0 .. {DIO_MAX_RATE_HZ:g} Hz")
        names = self._check_di(lines)
        if clock is not None:
            normalize_terminal(clock)
        self.di_rate_hz = float(rate_hz)
        return [[False] * int(samples) for _ in names]

    def wait_digital_change(self, lines: Sequence[str], edge: str = "both",
                            timeout_s: float = 10.0) -> List[bool]:
        if edge not in ("rising", "falling", "both"):
            raise ValueError("edge must be 'rising', 'falling' or 'both'")
        names = self._check_di(lines)
        return [edge != "falling"] * len(names)     # the change is taken as seen

    @_locked
    def write_digital_pattern(self, lines: Sequence[str], rows: Sequence[Sequence[bool]],
                              rate_hz: float,
                              record: Optional[Sequence[str]] = None) -> Optional[List[List[bool]]]:
        names = check_timed_lines(lines)
        steps = [[bool(v) for v in row] for row in rows]
        if len(steps) < 2 or any(len(row) != len(names) for row in steps):
            raise ValueError("a pattern needs at least 2 rows, each with one state per line")
        if not (0.0 < float(rate_hz) <= DIO_MAX_RATE_HZ):
            raise ValueError(f"rate_hz = {float(rate_hz):g} is outside 0 .. {DIO_MAX_RATE_HZ:g} Hz")
        self._require_open()
        for name in names:
            self._require_pin_free(name)
        record_names = None
        if record:
            record_names = [n for n in check_timed_lines(record)]
            if set(record_names) & set(names):
                raise ValueError("a line cannot be played and recorded at the same time")
            for name in record_names:
                if name in self.digital_outputs:
                    raise RuntimeError(f"{name} is being driven as an output in this session; "
                                       "reading it as an input would release it")
        self.patterns.append({"lines": names, "rows": steps, "rate_hz": float(rate_hz)})
        self.do_rate_hz = float(rate_hz)
        for name in names:
            self.digital_outputs[name] = False      # left low when the pattern is over
        if record_names is None:
            return None
        return [[False] * len(steps) for _ in record_names]

    # -- counters: time measurements and the encoder -----------------------
    def _check_time(self, terminals, min_s, max_s, counter, *edges) -> float:
        if not (0.0 < float(min_s) < float(max_s)):
            raise ValueError("min_s and max_s must satisfy 0 < min_s < max_s")
        for edge in edges:
            if edge not in EDGES:
                raise ValueError(f"edge must be one of {EDGES}, got {edge!r}")
        for terminal in terminals:
            normalize_terminal(terminal)
        self._input_counter(counter)
        return float(min_s), float(max_s)

    def measure_pulse_width(self, terminal: str, min_s: float = 1e-6, max_s: float = 1.0,
                            level: str = "high", timeout_s: float = 5.0,
                            counter: Optional[str] = None) -> float:
        if level not in ("high", "low"):
            raise ValueError("level must be 'high' or 'low'")
        low, high = self._check_time([terminal], min_s, max_s, counter)
        return min(max(0.5 / self.MOCK_FREQUENCY_HZ, low), high)

    def measure_period(self, terminal: str, min_s: float = 1e-6, max_s: float = 1.0,
                       edge: str = "rising", timeout_s: float = 5.0,
                       counter: Optional[str] = None) -> float:
        low, high = self._check_time([terminal], min_s, max_s, counter, edge)
        return min(max(1.0 / self.MOCK_FREQUENCY_HZ, low), high)

    def measure_edge_separation(self, first: str, second: str, min_s: float = 1e-6,
                                max_s: float = 1.0, first_edge: str = "rising",
                                second_edge: str = "rising", timeout_s: float = 5.0,
                                counter: Optional[str] = None) -> float:
        low, high = self._check_time([first, second], min_s, max_s, counter, first_edge, second_edge)
        return min(max(0.25 / self.MOCK_FREQUENCY_HZ, low), high)

    @_locked
    def start_encoder(self, a: str, b: str, pulses_per_rev: int, decoding: str = "x4",
                      counter: Optional[str] = None) -> str:
        if decoding not in ENCODER_DECODINGS:
            raise ValueError(f"decoding must be one of {ENCODER_DECODINGS}")
        if int(pulses_per_rev) < 1:
            raise ValueError("pulses_per_rev must be >= 1")
        if self.encoder is not None:
            raise RuntimeError("an encoder is already being followed; stop it first")
        self._require_open()
        if normalize_terminal(a) == normalize_terminal(b):
            raise ValueError("the encoder's A and B signals need two different terminals")
        name = self._input_counter(counter)
        self.encoder = {"counter": name, "a": normalize_terminal(a), "b": normalize_terminal(b),
                        "pulses_per_rev": int(pulses_per_rev), "decoding": decoding}
        return name

    @_locked
    def read_encoder(self) -> float:
        if self.encoder is None:
            raise RuntimeError("no encoder is being followed (start_encoder comes first)")
        return self.MOCK_ANGLE_DEG

    @_locked
    def stop_encoder(self) -> bool:
        encoder, self.encoder = self.encoder, None
        return encoder is not None

    # -- safe state -----------------------------------------------------
    @_locked
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

    @_locked
    def reset_outputs(self) -> List[str]:
        self.pulse_trains.clear()
        self.waveforms.clear()
        for name in self.analog_outputs:
            self.analog_outputs[name] = 0.0
        for name in self.digital_outputs:
            self.digital_outputs[name] = False
        return []

    @_locked
    def shutdown_outputs(self) -> List[str]:
        self._closed = True
        return self.reset_outputs()

    @_locked
    def close(self) -> List[str]:
        self._closed = True
        failed = self.reset_outputs()
        self.stop_stream()
        self.encoder = None
        return failed
