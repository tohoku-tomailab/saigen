"""Driver for a National Instruments multifunction DAQ device (written for the USB-6351).

What the board can do, as plain methods through NI-DAQmx:

- analog input    ``read_analog`` (mean of N samples), ``acquire`` (hardware-timed
                  waveform of several channels, optional digital start trigger),
                  ``start_stream`` / ``read_stream`` / ``stop_stream`` (continuous)
- analog output   ``write_analog`` (DC), ``start_waveform`` (hardware-timed, repeating)
- digital I/O     ``read_digital`` / ``write_digital`` (one line)
- counters        ``count_edges``, ``measure_frequency``, ``start_pulses`` (pulse train / PWM)
- safe state      ``stop_output``, ``reset_outputs``, ``close``

Requirements: the NI-DAQmx driver (from ni.com, Windows or Linux) and the ``nidaqmx``
Python package. ``nidaqmx`` is imported inside the methods, so importing this module,
the mock path and the tests need neither.

The driver knows nothing about flows, signal names or what may be driven: that is
``DaqService`` (names from config.yaml, output policy), reached through the safety
wrapper ``SharedDevices``. ``MockNIDaq`` has the same methods without hardware.

Outputs keep their task open until ``close()``, which first returns every output this
object drove to 0 V / low and stops every pulse train. An X Series device otherwise
holds its last output value after the program has gone.

    from src.devices.nidaq import NIDaq

    daq = NIDaq(device="Dev1")
    if daq.connect():
        print(daq.read_analog("ai0", samples=100))
        daq.close()
"""

import functools
import logging
import threading
import time
from typing import Dict, List, Optional, Sequence, Tuple

from .policy import (AI_MAX_RATE_AGGREGATE_HZ, AI_MAX_RATE_SINGLE_HZ, AO_MAX_RATE_HZ,
                     COUNTER_OUTPUT_LINES, DIO_MAX_RATE_HZ, ENCODER_DECODINGS, DEVICE_AO_MAX_V, DEVICE_AO_MIN_V, EDGES, TERMINALS, normalize_ai, normalize_ao,
                     normalize_counter, normalize_line, normalize_terminal, physical_kind,
                     check_timed_lines)

logger = logging.getLogger(__name__)

#: How the device spells the internal timebases (terminal names are lower-cased in flows).
_INTERNAL_TERMINALS = {"100khztimebase": "100kHzTimebase", "20mhztimebase": "20MHzTimebase"}
COUNTERS = ("ctr0", "ctr1", "ctr2", "ctr3")
#: DAQmx status codes of a read that ran out of time (samples / counter value not there yet).
_TIMEOUT_CODES = (-200284, -200474, -200281)   # -200281: the reference trigger did not come

#: count_edges reads the 32-bit counter this often, so a wrap is seen as long as the
#: process is not stalled for tens of seconds (100 MHz, the fastest input, wraps after 43 s).
_COUNT_POLL_S = 10.0
_COUNTER_MODULUS = 2 ** 32

#: One analog input of a multi-channel task: (channel, v_range, terminal) or, for a
#: sensor channel, (channel, v_range, terminal, sensor) with the dict of DaqSignals.
AiSpec = tuple


def _load_nidaqmx():
    """Import nidaqmx lazily, with a message that says what to install."""
    try:
        import nidaqmx
        import nidaqmx.constants  # noqa: F401
        import nidaqmx.system  # noqa: F401
    except ImportError as exc:  # pragma: no cover - depends on the environment
        raise RuntimeError(
            "the 'nidaqmx' Python package is not installed (pip install nidaqmx); "
            "the NI-DAQmx driver from ni.com is needed as well") from exc
    return nidaqmx


def check_ai_rate(n_channels: int, rate_hz: float) -> None:
    """Refuse a sample rate the USB-6351 cannot deliver for this many channels."""
    if n_channels < 1:
        raise ValueError("at least one analog input channel is needed")
    rate = float(rate_hz)
    limit = AI_MAX_RATE_SINGLE_HZ if n_channels == 1 else AI_MAX_RATE_AGGREGATE_HZ / n_channels
    if not (0.0 < rate <= limit):
        raise ValueError(f"rate_hz = {rate:g} is outside 0 .. {limit:g} Hz "
                         f"(the limit for {n_channels} analog input channel(s))")


def _locked(method):
    """Serialise the methods that touch the output tasks: ``close()`` then waits for a
    call still running in a worker thread, and nothing can slip in behind it."""
    @functools.wraps(method)
    def wrapper(self, *args, **kwargs):
        with self._lock:
            return method(self, *args, **kwargs)
    return wrapper


class NIDaq:
    """One NI-DAQmx device, addressed by its name in NI MAX (e.g. "Dev1")."""

    def __init__(self, device: str = "Dev1"):
        """
        Args:
            device: DAQmx device name as shown in NI MAX. Placeholder default; the real
                name belongs in config.yaml (shared_devices.daq_device).
        """
        self.device = device
        self.product_type: Optional[str] = None
        self._ao_tasks: Dict[str, object] = {}
        self._ao_waveforms: set = set()         # ao channels whose task generates a waveform
        self._ao_driven: set = set()            # ao channels not known to be back at 0 V
        self.ai_rate_hz: Optional[float] = None  # sample rate the device really used last
        self._lock = threading.RLock()
        self._closed = False
        self._do_tasks: Dict[str, object] = {}
        self._do_states: Dict[str, bool] = {}
        self._do_dirty: set = set()             # lines not known to be back at low
        self.do_rate_hz: Optional[float] = None  # clock a digital pattern really used
        self._co_tasks: Dict[str, object] = {}  # running pulse trains
        self._stream_task = None
        self._stream_channels = 0
        self._stream_id = 0
        self._pattern_task = None               # a digital pattern being played
        self._pattern_lines: List[str] = []
        self._encoder = None                    # (counter, task) of the encoder being followed
        self.di_rate_hz: Optional[float] = None
        self._connected = False

    # ------------------------------------------------------------------
    # connection
    # ------------------------------------------------------------------
    def list_devices(self) -> List[str]:
        """Names of the DAQmx devices this PC knows."""
        nidaqmx = _load_nidaqmx()
        return [d.name for d in nidaqmx.system.System.local().devices]

    def connect(self) -> bool:
        """Check that the device exists. Opens no channel and drives no output."""
        try:
            nidaqmx = _load_nidaqmx()
            devices = {d.name: d for d in nidaqmx.system.System.local().devices}
        except Exception as exc:  # driver missing, service stopped, ...
            logger.error(f"NI-DAQmx is not available: {exc}")
            return False
        if self.device not in devices:
            logger.error(f"DAQ device {self.device!r} not found "
                         f"(devices known to NI-DAQmx: {sorted(devices) or 'none'})")
            return False
        try:
            self.product_type = str(devices[self.device].product_type)
        except Exception as exc:
            logger.error(f"DAQ device {self.device!r} does not answer: {exc}")
            return False
        if self._closed:
            logger.error("this DAQ object has been closed; create a new one for a new run")
            return False
        self._connected = True
        return True

    def _require_open(self) -> None:
        if self._closed:
            raise RuntimeError("the DAQ has been closed (the run is over); no output is driven")

    @staticmethod
    def _actual_rate(task, requested: float) -> float:
        """The sample clock DAQmx really set (it coerces to what the timebase can divide)."""
        try:
            rate = float(task.timing.samp_clk_rate)
        except Exception:
            return float(requested)
        return rate if rate > 0.0 else float(requested)

    def _physical(self, channel: str) -> str:
        return f"{self.device}/{channel}"

    def _terminal(self, terminal: str) -> str:
        name = normalize_terminal(terminal)
        return f"/{self.device}/{_INTERNAL_TERMINALS.get(name, name.upper())}"

    @staticmethod
    def _edge(nidaqmx, edge: str):
        if edge not in EDGES:
            raise ValueError(f"edge must be one of {EDGES}, got {edge!r}")
        return getattr(nidaqmx.constants.Edge, edge.upper())

    # ------------------------------------------------------------------
    # analog input
    # ------------------------------------------------------------------
    def _require_ai_free(self) -> None:
        if self._stream_task is not None:
            raise RuntimeError(
                "the analog inputs are busy with a continuous acquisition (daq_start_logging); "
                "stop it first, or add the channel to the logged channels")

    def _add_ai(self, nidaqmx, task, specs: Sequence[AiSpec]) -> List[str]:
        names = []
        constants = nidaqmx.constants
        for spec in specs:
            channel, v_range, terminal = spec[:3]
            sensor = spec[3] if len(spec) > 3 else None
            name = normalize_ai(channel)
            if name in names:
                raise ValueError(f"{name} is listed twice")
            if terminal not in TERMINALS:
                raise ValueError(f"terminal must be one of {TERMINALS}, got {terminal!r}")
            if not (0 < float(v_range) <= 10.0):
                raise ValueError("v_range must be in (0, 10] V")
            terminal_config = getattr(constants.TerminalConfiguration, terminal.upper())
            if sensor is None:
                task.ai_channels.add_ai_voltage_chan(
                    self._physical(name), terminal_config=terminal_config,
                    min_val=-float(v_range), max_val=float(v_range))
            else:
                if sensor["type"] == "thermocouple":
                    chan = task.ai_channels.add_ai_thrmcpl_chan(
                        self._physical(name), min_val=sensor["min_c"], max_val=sensor["max_c"],
                        units=constants.TemperatureUnits.DEG_C,
                        thermocouple_type=getattr(constants.ThermocoupleType, sensor["tc_type"]),
                        cjc_source=constants.CJCSource.CONSTANT_USER_VALUE,
                        cjc_val=sensor["cjc_c"])
                elif sensor["type"] == "rtd":
                    wires = {2: "TWO_WIRE", 3: "THREE_WIRE", 4: "FOUR_WIRE"}[sensor["wires"]]
                    chan = task.ai_channels.add_ai_rtd_chan(
                        self._physical(name), min_val=sensor["min_c"], max_val=sensor["max_c"],
                        units=constants.TemperatureUnits.DEG_C,
                        rtd_type=getattr(constants.RTDType,
                                         sensor["rtd_type"].upper().replace("PT", "PT_")),
                        resistance_config=getattr(constants.ResistanceConfiguration, wires),
                        current_excit_source=constants.ExcitationSource.EXTERNAL,
                        current_excit_val=sensor["excitation_a"], r_0=sensor["r0_ohm"])
                else:
                    raise ValueError(f"unknown sensor type {sensor['type']!r}")
                if terminal != "default":
                    chan.ai_term_cfg = terminal_config
            names.append(name)
        if not names:
            raise ValueError("at least one analog input channel is needed")
        return names

    def read_analog(self, channel: str, samples: int = 1, rate_hz: float = 1000.0,
                    v_range: float = 10.0, terminal: str = "default",
                    sensor: Optional[dict] = None) -> float:
        """Mean of ``samples`` readings of one analog input: volts, or degrees Celsius
        for a sensor channel (``sensor`` = the dict of ``DaqSignals``).

        Args:
            channel: "ai0" .. (without the device name)
            samples: number of readings averaged; 1 = a single on-demand reading
            rate_hz: sample clock when samples > 1
            v_range: the input range is +/- v_range volts (pick the smallest that fits)
            terminal: "default", "rse", "nrse" or "diff" (how the signal is wired)
        """
        name = normalize_ai(channel)
        samples = int(samples)
        if samples < 1:
            raise ValueError("samples must be >= 1")
        if terminal not in TERMINALS:
            raise ValueError(f"terminal must be one of {TERMINALS}, got {terminal!r}")
        if not (0 < float(v_range) <= 10.0):
            raise ValueError("v_range must be in (0, 10] V")
        self._require_ai_free()
        nidaqmx = _load_nidaqmx()
        constants = nidaqmx.constants
        with nidaqmx.Task() as task:
            self._add_ai(nidaqmx, task, [(name, v_range, terminal, sensor)])
            if samples == 1:
                return float(task.read())
            task.timing.cfg_samp_clk_timing(
                float(rate_hz), sample_mode=constants.AcquisitionType.FINITE,
                samps_per_chan=samples)
            data = task.read(number_of_samples_per_channel=samples,
                             timeout=samples / float(rate_hz) + 5.0)
        if len(data) != samples:
            raise RuntimeError(f"{name}: asked for {samples} samples, got {len(data)}")
        return float(sum(data) / len(data))

    def acquire(self, specs: Sequence[AiSpec], rate_hz: float, samples: int,
                trigger: Optional[str] = None, trigger_edge: str = "rising",
                timeout_s: float = 10.0, trigger_level: Optional[float] = None,
                pretrigger: int = 0, clock: Optional[str] = None) -> List[List[float]]:
        """Hardware-timed acquisition of ``samples`` points on each channel.

        Args:
            specs: [(channel, v_range, terminal), ...]; all channels share one sample clock
            rate_hz: sample rate per channel
            samples: points per channel (>= 2)
            trigger: None = start at once; "pfi0" .. = start on a digital edge there;
                with ``trigger_level``, the first analog input of ``specs`` ("ai0" ..)
            trigger_edge: "rising" or "falling"
            timeout_s: how long to wait for the trigger, on top of the acquisition time
            trigger_level: volts; makes the trigger an analog one (the signal crossing
                this level in the direction of ``trigger_edge``)
            pretrigger: of the ``samples`` points, how many lie before the trigger
            clock: None = the device's clock; "pfi0" .. = one sample per edge there
                (``rate_hz`` is then the highest rate that clock will have)

        Returns:
            one list of volts per channel, in the order of ``specs``
        """
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
        self._require_ai_free()
        nidaqmx = _load_nidaqmx()
        with nidaqmx.Task() as task:
            names = self._add_ai(nidaqmx, task, specs)
            kwargs = {"source": self._terminal(clock)} if clock is not None else {}
            task.timing.cfg_samp_clk_timing(
                float(rate_hz), sample_mode=nidaqmx.constants.AcquisitionType.FINITE,
                samps_per_chan=samples, **kwargs)
            if trigger is not None:
                edge = self._edge(nidaqmx, trigger_edge)
                triggers = task.triggers
                if trigger_level is not None:
                    if normalize_ai(trigger) != names[0]:
                        raise ValueError("an analog trigger must be the first channel acquired "
                                         f"({names[0]}), got {trigger}")
                    slope = getattr(nidaqmx.constants.Slope, trigger_edge.upper())
                    source = self._physical(names[0])
                    if pretrigger:
                        triggers.reference_trigger.cfg_anlg_edge_ref_trig(
                            source, pretrigger_samples=pretrigger, trigger_slope=slope,
                            trigger_level=float(trigger_level))
                    else:
                        triggers.start_trigger.cfg_anlg_edge_start_trig(
                            source, trigger_slope=slope, trigger_level=float(trigger_level))
                elif pretrigger:
                    triggers.reference_trigger.cfg_dig_edge_ref_trig(
                        self._terminal(trigger), pretrigger_samples=pretrigger, trigger_edge=edge)
                else:
                    triggers.start_trigger.cfg_dig_edge_start_trig(
                        self._terminal(trigger), trigger_edge=edge)
            # with an external clock the rate is whatever arrives; the nominal one is kept
            self.ai_rate_hz = (float(rate_hz) if clock is not None
                               else self._actual_rate(task, rate_hz))
            try:
                data = task.read(number_of_samples_per_channel=samples,
                                 timeout=samples / float(rate_hz) + float(timeout_s))
            except Exception as exc:
                if getattr(exc, "error_code", None) in _TIMEOUT_CODES:
                    if trigger is not None:
                        raise TimeoutError(f"no {trigger_edge} edge on {trigger} within "
                                           f"{float(timeout_s):g} s; nothing was acquired") from exc
                    if clock is not None:
                        raise TimeoutError(f"the clock on {clock} did not deliver {samples} "
                                           "edges; nothing was acquired") from exc
                raise
        return self._columns(names, data, samples)

    @_locked
    def start_stream(self, specs: Sequence[AiSpec], rate_hz: float) -> int:
        """Start a continuous acquisition; fetch the samples with ``read_stream``.

        Returns the stream's number: a reader that passes it to ``read_stream`` can
        never be handed the samples of a later stream.
        """
        check_ai_rate(len(specs), rate_hz)
        self._require_open()            # a late worker must not start one behind close()
        self._require_ai_free()
        nidaqmx = _load_nidaqmx()
        task = nidaqmx.Task()
        try:
            names = self._add_ai(nidaqmx, task, specs)
            task.timing.cfg_samp_clk_timing(
                float(rate_hz), sample_mode=nidaqmx.constants.AcquisitionType.CONTINUOUS,
                samps_per_chan=max(int(float(rate_hz) * 10), 1000))   # ~10 s of buffer
            self.ai_rate_hz = self._actual_rate(task, rate_hz)
            task.start()
        except BaseException:
            task.close()
            raise
        self._stream_task = task
        self._stream_channels = len(names)
        self._stream_id += 1
        return self._stream_id

    @_locked
    def read_stream(self, stream: Optional[int] = None) -> List[List[float]]:
        """Everything acquired since the last call: one list of volts per channel.

        ``stream`` = the number ``start_stream`` returned; a reader of an earlier
        stream is refused instead of being given this one's samples.
        """
        if self._stream_task is None or (stream is not None and stream != self._stream_id):
            raise RuntimeError("no continuous acquisition is running"
                               if self._stream_task is None else
                               "that continuous acquisition is over; another one is running")
        nidaqmx = _load_nidaqmx()
        data = self._stream_task.read(
            number_of_samples_per_channel=nidaqmx.constants.READ_ALL_AVAILABLE, timeout=10.0)
        if self._stream_channels == 1:
            return [list(data)]
        return [list(col) for col in data]

    @_locked
    def stop_stream(self) -> None:
        task, self._stream_task = self._stream_task, None
        self._stream_channels = 0
        if task is not None:
            try:
                task.stop()
            finally:
                task.close()

    # ------------------------------------------------------------------
    # analog output
    # ------------------------------------------------------------------
    def _release_ao(self, name: str) -> None:
        task = self._ao_tasks.get(name)
        if task is not None:
            try:
                task.stop()
            finally:
                task.close()            # the entry stays if this fails
                self._ao_tasks.pop(name, None)
        self._ao_waveforms.discard(name)

    @_locked
    def write_analog(self, channel: str, voltage: float) -> float:
        """Set one analog output to a DC voltage and hold it. Returns the value set."""
        self._require_open()
        return self._set_ao(channel, voltage)

    def _set_ao(self, channel: str, voltage: float) -> float:
        name = normalize_ao(channel)
        volts = float(voltage)
        if not (DEVICE_AO_MIN_V <= volts <= DEVICE_AO_MAX_V):
            raise ValueError(f"{name}: {volts:g} V is outside the device span "
                             f"{DEVICE_AO_MIN_V:g} .. {DEVICE_AO_MAX_V:g} V")
        # From here the channel counts as driven until a write of 0 V has succeeded:
        # a failure half-way must not make close() forget it.
        self._ao_driven.add(name)
        if name in self._ao_waveforms:
            self._release_ao(name)          # a waveform task cannot take a single value
        task = self._ao_tasks.get(name)
        if task is None:
            nidaqmx = _load_nidaqmx()
            task = nidaqmx.Task()
            try:
                task.ao_channels.add_ao_voltage_chan(
                    self._physical(name), min_val=DEVICE_AO_MIN_V, max_val=DEVICE_AO_MAX_V)
            except Exception:
                task.close()
                raise
            self._ao_tasks[name] = task
        task.write(volts, auto_start=True)
        if volts == 0.0:
            self._ao_driven.discard(name)
        return volts

    @_locked
    def start_waveform(self, channel: str, period: Sequence[float], rate_hz: float) -> float:
        """Repeat ``period`` (one cycle, in volts) on an analog output at ``rate_hz`` points/s.

        Hardware-timed; runs until ``stop_output`` / ``write_analog`` / ``close``. One
        waveform at a time: the device has a single analog-output sample clock.
        Returns the point rate the device really uses.
        """
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
        others = sorted(self._ao_waveforms - {name})
        if others:
            raise RuntimeError(f"{others[0]} is generating a waveform; the device has one "
                               "analog-output sample clock, so stop it first")
        nidaqmx = _load_nidaqmx()
        self._ao_driven.add(name)       # see _set_ao: not forgotten if this fails half-way
        self._release_ao(name)
        task = nidaqmx.Task()
        try:
            task.ao_channels.add_ao_voltage_chan(
                self._physical(name), min_val=DEVICE_AO_MIN_V, max_val=DEVICE_AO_MAX_V)
            task.timing.cfg_samp_clk_timing(
                float(rate_hz), sample_mode=nidaqmx.constants.AcquisitionType.CONTINUOUS,
                samps_per_chan=len(points))
            actual = self._actual_rate(task, rate_hz)
            task.write(points, auto_start=False)
            self._ao_tasks[name] = task
            self._ao_waveforms.add(name)
            task.start()
        except BaseException:
            self._ao_waveforms.discard(name)
            task.close()                    # the entry stays if this fails
            self._ao_tasks.pop(name, None)
            raise
        return actual

    # ------------------------------------------------------------------
    # digital I/O
    # ------------------------------------------------------------------
    def _require_pin_free(self, name: str) -> None:
        for counter, pin in COUNTER_OUTPUT_LINES.items():
            if pin == name and counter in self._co_tasks:
                raise RuntimeError(f"{name} is the output pin of {counter}, which is generating "
                                   "a pulse train; stop it first")

    @_locked
    def write_digital(self, line: str, state: bool) -> bool:
        """Drive one digital line high (True) or low (False) and hold it."""
        self._require_open()
        return self._set_do(line, state)

    def _set_do(self, line: str, state: bool) -> bool:
        name = normalize_line(line)
        self._require_pin_free(name)
        if self._pattern_task is not None and name in self._pattern_lines:
            raise RuntimeError(f"{name} is playing a digital pattern")
        # From here the line counts as driven until a write of low has succeeded.
        self._do_dirty.add(name)
        task = self._do_tasks.get(name)
        if task is None:
            nidaqmx = _load_nidaqmx()
            task = nidaqmx.Task()
            try:
                task.do_channels.add_do_chan(
                    self._physical(name),
                    line_grouping=nidaqmx.constants.LineGrouping.CHAN_PER_LINE)
            except Exception:
                task.close()
                raise
            self._do_tasks[name] = task
        task.write(bool(state), auto_start=True)
        self._do_states[name] = bool(state)
        if not state:
            self._do_dirty.discard(name)
        return bool(state)

    @_locked
    def read_digital(self, line: str) -> bool:
        """Read one digital line as an input.

        A line this object is driving is refused: configuring it as an input would
        release the output.
        """
        name = normalize_line(line)
        if name in self._do_tasks or name in self._pattern_lines:
            raise RuntimeError(f"{name} is being driven as an output in this session; "
                               "reading it as an input would release it")
        self._require_pin_free(name)
        nidaqmx = _load_nidaqmx()
        with nidaqmx.Task() as task:
            task.di_channels.add_di_chan(
                self._physical(name),
                line_grouping=nidaqmx.constants.LineGrouping.CHAN_PER_LINE)
            return bool(task.read())

    # ------------------------------------------------------------------
    # counters
    # ------------------------------------------------------------------
    def _input_counter(self, counter: Optional[str]) -> str:
        """The counter a measurement uses: the one asked for, else the highest free one."""
        if counter is not None:
            name = normalize_counter(counter)
            if name not in COUNTERS:
                raise ValueError(f"{name}: the device has {COUNTERS}")
            if name in self._co_tasks:
                raise RuntimeError(f"{name} is generating a pulse train; stop it first "
                                   "or use another counter")
            if self._encoder is not None and self._encoder[0] == name:
                raise RuntimeError(f"{name} is following the encoder; use another counter")
            return name
        busy = set(self._co_tasks) | ({self._encoder[0]} if self._encoder is not None else set())
        for name in reversed(COUNTERS):
            if name not in busy:
                return name
        raise RuntimeError("all four counters are in use")

    def count_edges(self, terminal: str, duration_s: float, edge: str = "rising",
                    counter: Optional[str] = None, sleep=time.sleep) -> int:
        """Count the edges arriving on a terminal ("pfi0" ..) during ``duration_s``."""
        duration = float(duration_s)
        if not duration > 0.0:
            raise ValueError("duration_s must be > 0")
        source = self._terminal(terminal)
        name = self._input_counter(counter)
        nidaqmx = _load_nidaqmx()
        with nidaqmx.Task() as task:
            chan = task.ci_channels.add_ci_count_edges_chan(
                self._physical(name), edge=self._edge(nidaqmx, edge), initial_count=0)
            chan.ci_count_edges_term = source
            task.start()
            # The counter is 32 bits wide: read it often enough to see every wrap.
            count, last, remaining = 0, 0, duration
            while remaining > 0.0:
                step = min(remaining, _COUNT_POLL_S)
                sleep(step)
                remaining -= step
                now = int(task.read())
                count += (now - last) % _COUNTER_MODULUS
                last = now
            task.stop()
        return count

    def measure_frequency(self, terminal: str, min_hz: float = 1.0, max_hz: float = 100000.0,
                          timeout_s: float = 5.0, edge: str = "rising",
                          counter: Optional[str] = None) -> float:
        """Frequency (Hz) of the signal on a terminal, from one period (one counter).

        ``min_hz`` .. ``max_hz`` is the expected range; no signal within ``timeout_s``
        raises.
        """
        low, high = float(min_hz), float(max_hz)
        if not (0.0 < low < high):
            raise ValueError("min_hz and max_hz must satisfy 0 < min_hz < max_hz")
        source = self._terminal(terminal)
        name = self._input_counter(counter)
        nidaqmx = _load_nidaqmx()
        with nidaqmx.Task() as task:
            chan = task.ci_channels.add_ci_freq_chan(
                self._physical(name), min_val=low, max_val=high, edge=self._edge(nidaqmx, edge))
            chan.ci_freq_term = source
            try:
                return float(task.read(timeout=float(timeout_s)))
            except Exception as exc:
                if getattr(exc, "error_code", None) in _TIMEOUT_CODES:
                    raise TimeoutError(f"no signal on {terminal} within {float(timeout_s):g} s "
                                       f"(expected {low:g} .. {high:g} Hz)") from exc
                raise

    @_locked
    def start_pulses(self, counter: str, frequency_hz: float, duty_cycle: float = 0.5,
                     count: Optional[int] = None) -> None:
        """Generate a pulse train on a counter's output terminal (idle low).

        ``count`` = None runs until ``stop_output``; a number stops after that many
        pulses (see ``pulses_done``). On the USB-6351 the default outputs are
        ctr0 -> PFI12, ctr1 -> PFI13, ctr2 -> PFI14, ctr3 -> PFI15.
        """
        name = normalize_counter(counter)
        if name not in COUNTERS:
            raise ValueError(f"{name}: the device has {COUNTERS}")
        freq, duty = float(frequency_hz), float(duty_cycle)
        if not freq > 0.0 or not (0.0 < duty < 1.0):
            raise ValueError("frequency_hz must be > 0 and duty_cycle between 0 and 1")
        if count is not None and int(count) < 1:
            raise ValueError("count must be >= 1")
        self._require_open()
        if self._encoder is not None and self._encoder[0] == name:
            raise RuntimeError(f"{name} is following the encoder; use another counter")
        if COUNTER_OUTPUT_LINES[name] in self._do_tasks:
            raise RuntimeError(f"the output pin of {name} ({COUNTER_OUTPUT_LINES[name]}) has "
                               "been driven as a digital line in this run; use another counter")
        nidaqmx = _load_nidaqmx()
        self._stop_counter(name)
        task = nidaqmx.Task()
        try:
            task.co_channels.add_co_pulse_chan_freq(
                self._physical(name), idle_state=nidaqmx.constants.Level.LOW,
                freq=freq, duty_cycle=duty)
            if count is None:
                task.timing.cfg_implicit_timing(
                    sample_mode=nidaqmx.constants.AcquisitionType.CONTINUOUS)
            else:
                task.timing.cfg_implicit_timing(
                    sample_mode=nidaqmx.constants.AcquisitionType.FINITE,
                    samps_per_chan=int(count))
            self._co_tasks[name] = task     # registered first: an interrupted start is cleaned up
            task.start()
        except BaseException:
            task.close()                    # stops the train; the entry stays if this fails
            self._co_tasks.pop(name, None)
            raise

    @_locked
    def pulses_done(self, counter: str) -> bool:
        """True when a finite pulse train has finished (or none is running)."""
        task = self._co_tasks.get(normalize_counter(counter))
        return True if task is None else bool(task.is_task_done())

    def _stop_counter(self, name: str) -> None:
        task = self._co_tasks.get(name)
        if task is not None:
            try:
                task.stop()
            finally:
                task.close()                # also stops it; the entry stays if this fails
                self._co_tasks.pop(name, None)

    # ------------------------------------------------------------------
    # stimulus and response on one clock edge
    # ------------------------------------------------------------------
    def stimulate_acquire(self, channel: str, period: Sequence[float], ao_rate_hz: float,
                          specs: Sequence[AiSpec], rate_hz: float,
                          samples: int) -> Tuple[List[List[float]], float]:
        """Start a waveform on an analog output and an acquisition at the same instant.

        The acquisition is armed first and starts on the output's own start trigger, so
        sample 0 is the first point of the waveform. The output is returned to 0 V when
        the acquisition is over, also when it fails.

        Returns (one list per input channel, the point rate the output really used).
        """
        samples = int(samples)
        if samples < 2:
            raise ValueError("samples must be >= 2")
        check_ai_rate(len(specs), rate_hz)
        self._require_ai_free()
        self._require_open()
        ao_name = normalize_ao(channel)
        nidaqmx = _load_nidaqmx()
        with nidaqmx.Task() as task:
            names = self._add_ai(nidaqmx, task, specs)
            task.timing.cfg_samp_clk_timing(
                float(rate_hz), sample_mode=nidaqmx.constants.AcquisitionType.FINITE,
                samps_per_chan=samples)
            task.triggers.start_trigger.cfg_dig_edge_start_trig(
                f"/{self.device}/ao/StartTrigger", trigger_edge=nidaqmx.constants.Edge.RISING)
            self.ai_rate_hz = self._actual_rate(task, rate_hz)
            task.start()                    # armed: waits for the output to start
            actual = self.start_waveform(ao_name, period, ao_rate_hz)
            try:
                data = task.read(number_of_samples_per_channel=samples,
                                 timeout=samples / float(rate_hz) + 10.0)
            finally:
                self.stop_output(ao_name)
        return self._columns(names, data, samples), actual

    @staticmethod
    def _columns(names, data, samples) -> List[List[float]]:
        columns = [list(data)] if len(names) == 1 else [list(col) for col in data]
        for name, column in zip(names, columns):
            if len(column) != samples:
                raise RuntimeError(f"{name}: asked for {samples} samples, got {len(column)}")
        return columns

    # ------------------------------------------------------------------
    # hardware-timed digital I/O (port 0)
    # ------------------------------------------------------------------
    def _add_di(self, nidaqmx, task, lines: Sequence[str]) -> List[str]:
        names = check_timed_lines(lines)
        for name in names:
            if name in self._do_tasks or name in self._pattern_lines:
                raise RuntimeError(f"{name} is being driven as an output in this session; "
                                   "reading it as an input would release it")
            task.di_channels.add_di_chan(
                self._physical(name), line_grouping=nidaqmx.constants.LineGrouping.CHAN_PER_LINE)
        return names

    @staticmethod
    def _bool_columns(names, data) -> List[List[bool]]:
        if len(names) == 1:
            return [[bool(v) for v in data]]
        return [[bool(v) for v in col] for col in data]

    def acquire_digital(self, lines: Sequence[str], rate_hz: float, samples: int,
                        clock: Optional[str] = None) -> List[List[bool]]:
        """Clocked acquisition of digital lines of port 0: one list of states per line."""
        samples = int(samples)
        if samples < 2:
            raise ValueError("samples must be >= 2")
        if not (0.0 < float(rate_hz) <= DIO_MAX_RATE_HZ):
            raise ValueError(f"rate_hz = {float(rate_hz):g} is outside 0 .. {DIO_MAX_RATE_HZ:g} Hz")
        nidaqmx = _load_nidaqmx()
        with nidaqmx.Task() as task:
            with self._lock:
                names = self._add_di(nidaqmx, task, lines)
            kwargs = {"source": self._terminal(clock)} if clock is not None else {}
            task.timing.cfg_samp_clk_timing(
                float(rate_hz), sample_mode=nidaqmx.constants.AcquisitionType.FINITE,
                samps_per_chan=samples, **kwargs)
            self.di_rate_hz = (float(rate_hz) if clock is not None
                               else self._actual_rate(task, rate_hz))
            try:
                data = task.read(number_of_samples_per_channel=samples,
                                 timeout=samples / float(rate_hz) + 10.0)
            except Exception as exc:
                if clock is not None and getattr(exc, "error_code", None) in _TIMEOUT_CODES:
                    raise TimeoutError(f"the clock on {clock} did not deliver {samples} edges; "
                                       "nothing was acquired") from exc
                raise
        return self._bool_columns(names, data)

    def wait_digital_change(self, lines: Sequence[str], edge: str = "both",
                            timeout_s: float = 10.0) -> List[bool]:
        """Block until one of the lines (port 0) changes; returns their states then.

        Hardware change detection: a pulse too short for polling is not missed.
        """
        if edge not in ("rising", "falling", "both"):
            raise ValueError("edge must be 'rising', 'falling' or 'both'")
        nidaqmx = _load_nidaqmx()
        with nidaqmx.Task() as task:
            with self._lock:
                names = self._add_di(nidaqmx, task, lines)
            physical = ",".join(self._physical(n) for n in names)
            task.timing.cfg_change_detection_timing(
                rising_edge_chan=physical if edge in ("rising", "both") else "",
                falling_edge_chan=physical if edge in ("falling", "both") else "",
                sample_mode=nidaqmx.constants.AcquisitionType.CONTINUOUS, samps_per_chan=1000)
            task.start()
            try:
                data = task.read(number_of_samples_per_channel=1, timeout=float(timeout_s))
            except Exception as exc:
                if getattr(exc, "error_code", None) in _TIMEOUT_CODES:
                    raise TimeoutError(f"no {edge} change on {', '.join(names)} within "
                                       f"{float(timeout_s):g} s") from exc
                raise
        return [col[0] for col in self._bool_columns(names, data)]

    def write_digital_pattern(self, lines: Sequence[str], rows: Sequence[Sequence[bool]],
                              rate_hz: float,
                              record: Optional[Sequence[str]] = None) -> Optional[List[List[bool]]]:
        """Play a pattern on lines of port 0, one row per clock tick, then leave them low.

        ``rows[i][j]`` is the state of ``lines[j]`` at tick ``i``. ``record`` = lines of
        port 0 sampled on the same clock while the pattern plays (the response).
        Blocks until the pattern is over; ``close()`` from another thread ends it early.
        """
        names = check_timed_lines(lines)
        steps = [[bool(v) for v in row] for row in rows]
        if len(steps) < 2 or any(len(row) != len(names) for row in steps):
            raise ValueError("a pattern needs at least 2 rows, each with one state per line")
        if not (0.0 < float(rate_hz) <= DIO_MAX_RATE_HZ):
            raise ValueError(f"rate_hz = {float(rate_hz):g} is outside 0 .. {DIO_MAX_RATE_HZ:g} Hz")
        nidaqmx = _load_nidaqmx()
        finite = nidaqmx.constants.AcquisitionType.FINITE
        di = None
        try:
            with self._lock:
                self._require_open()
                if self._pattern_task is not None:
                    raise RuntimeError("a digital pattern is already playing")
                for name in names:
                    self._require_pin_free(name)
                if record:
                    if set(check_timed_lines(record)) & set(names):
                        raise ValueError("a line cannot be played and recorded at the same time")
                    di = nidaqmx.Task()
                    record_names = self._add_di(nidaqmx, di, list(record))
                    di.timing.cfg_samp_clk_timing(
                        float(rate_hz), source=f"/{self.device}/do/SampleClock",
                        sample_mode=finite, samps_per_chan=len(steps))
                    di.start()              # armed: samples on the pattern's clock
                # whatever happens from here, close() must still set these lines low
                self._do_dirty.update(names)
                for name in names:
                    self._do_states.pop(name, None)
                for name in names:          # a static task on the line would block the pattern
                    static = self._do_tasks.get(name)
                    if static is not None:
                        static.close()
                        self._do_tasks.pop(name, None)
                task = nidaqmx.Task()
                try:
                    for name in names:
                        task.do_channels.add_do_chan(
                            self._physical(name),
                            line_grouping=nidaqmx.constants.LineGrouping.CHAN_PER_LINE)
                    task.timing.cfg_samp_clk_timing(float(rate_hz), sample_mode=finite,
                                                    samps_per_chan=len(steps))
                    self.do_rate_hz = self._actual_rate(task, rate_hz)
                    columns = [[row[j] for row in steps] for j in range(len(names))]
                    task.write(columns[0] if len(names) == 1 else columns, auto_start=False)
                    self._pattern_task, self._pattern_lines = task, list(names)
                    task.start()
                except BaseException:
                    self._pattern_task, self._pattern_lines = task, list(names)
                    try:
                        self._end_pattern()     # closes it and sets the lines low
                    except Exception as exc:    # the lines stay marked for close()
                        logger.error(f"could not leave the pattern lines low: {exc}")
                    raise
            try:
                task.wait_until_done(timeout=len(steps) / float(rate_hz) + 10.0)
                recorded = None
                if di is not None:
                    recorded = self._bool_columns(
                        record_names,
                        di.read(number_of_samples_per_channel=len(steps), timeout=10.0))
            finally:
                with self._lock:
                    if self._pattern_task is task:
                        self._end_pattern()
            return recorded
        finally:
            if di is not None:
                di.close()

    def _end_pattern(self) -> None:
        """Close the pattern task and leave its lines low (caller holds the lock)."""
        task, lines = self._pattern_task, list(self._pattern_lines)
        self._pattern_task, self._pattern_lines = None, []
        problems = []
        if task is not None:
            try:
                task.close()
            except Exception as exc:
                problems.append(f"closing the pattern task: {exc}")
        for name in lines:                  # every line is attempted, whatever the others do
            try:
                self._set_do(name, False)
            except Exception as exc:        # the line stays in _do_dirty for close()
                problems.append(f"{name}: {exc}")
        if problems:
            raise RuntimeError("the digital pattern did not end cleanly ("
                               + "; ".join(problems) + ")")

    # ------------------------------------------------------------------
    # counters: time measurements and the encoder
    # ------------------------------------------------------------------
    def _measure_time(self, what: str, add: str, terminals: dict, min_s: float, max_s: float,
                      timeout_s: float, counter: Optional[str], **edges) -> float:
        low, high = float(min_s), float(max_s)
        if not (0.0 < low < high):
            raise ValueError("min_s and max_s must satisfy 0 < min_s < max_s")
        sources = {attr: self._terminal(t) for attr, t in terminals.items()}
        name = self._input_counter(counter)
        nidaqmx = _load_nidaqmx()
        edge_args = {k: self._edge(nidaqmx, v) for k, v in edges.items()}
        with nidaqmx.Task() as task:
            chan = getattr(task.ci_channels, add)(self._physical(name), min_val=low, max_val=high,
                                                  **edge_args)
            for attr, source in sources.items():
                setattr(chan, attr, source)
            try:
                return float(task.read(timeout=float(timeout_s)))
            except Exception as exc:
                if getattr(exc, "error_code", None) in _TIMEOUT_CODES:
                    raise TimeoutError(
                        f"no {what} on {', '.join(str(t) for t in terminals.values())} within "
                        f"{float(timeout_s):g} s (expected {low:g} .. {high:g} s)") from exc
                raise

    def measure_pulse_width(self, terminal: str, min_s: float = 1e-6, max_s: float = 1.0,
                            level: str = "high", timeout_s: float = 5.0,
                            counter: Optional[str] = None) -> float:
        """Width (s) of one pulse on a terminal; ``level`` "high" or "low" = which part."""
        if level not in ("high", "low"):
            raise ValueError("level must be 'high' or 'low'")
        return self._measure_time("pulse", "add_ci_pulse_width_chan",
                                  {"ci_pulse_width_term": terminal}, min_s, max_s, timeout_s, counter,
                                  starting_edge="rising" if level == "high" else "falling")

    def measure_period(self, terminal: str, min_s: float = 1e-6, max_s: float = 1.0,
                       edge: str = "rising", timeout_s: float = 5.0,
                       counter: Optional[str] = None) -> float:
        """Period (s) of the signal on a terminal, from one cycle."""
        return self._measure_time("signal", "add_ci_period_chan", {"ci_period_term": terminal},
                                  min_s, max_s, timeout_s, counter, edge=edge)

    def measure_edge_separation(self, first: str, second: str, min_s: float = 1e-6,
                                max_s: float = 1.0, first_edge: str = "rising",
                                second_edge: str = "rising", timeout_s: float = 5.0,
                                counter: Optional[str] = None) -> float:
        """Time (s) from an edge on ``first`` to the next edge on ``second``."""
        return self._measure_time("pair of edges", "add_ci_two_edge_sep_chan",
                                  {"ci_two_edge_sep_first_term": first,
                                   "ci_two_edge_sep_second_term": second},
                                  min_s, max_s, timeout_s, counter,
                                  first_edge=first_edge, second_edge=second_edge)

    @_locked
    def start_encoder(self, a: str, b: str, pulses_per_rev: int, decoding: str = "x4",
                      counter: Optional[str] = None) -> str:
        """Start following a quadrature encoder (A and B on two terminals) from 0 degrees.

        Returns the counter used. One encoder at a time; ``read_encoder`` gives the angle.
        """
        if decoding not in ENCODER_DECODINGS:
            raise ValueError(f"decoding must be one of {ENCODER_DECODINGS}")
        if int(pulses_per_rev) < 1:
            raise ValueError("pulses_per_rev must be >= 1")
        if self._encoder is not None:
            raise RuntimeError("an encoder is already being followed; stop it first")
        self._require_open()
        term_a, term_b = self._terminal(a), self._terminal(b)
        if term_a == term_b:
            raise ValueError("the encoder's A and B signals need two different terminals")
        name = self._input_counter(counter)
        nidaqmx = _load_nidaqmx()
        task = nidaqmx.Task()
        try:
            chan = task.ci_channels.add_ci_ang_encoder_chan(
                self._physical(name),
                decoding_type=getattr(nidaqmx.constants.EncoderType,
                                      decoding.upper().replace("X", "X_")),
                units=nidaqmx.constants.AngleUnits.DEGREES, pulses_per_rev=int(pulses_per_rev),
                initial_angle=0.0)
            chan.ci_encoder_a_input_term = term_a
            chan.ci_encoder_b_input_term = term_b
            task.start()
        except BaseException:
            task.close()
            raise
        self._encoder = (name, task)
        return name

    @_locked
    def read_encoder(self) -> float:
        """Angle (degrees) turned since ``start_encoder``; negative = the other way."""
        if self._encoder is None:
            raise RuntimeError("no encoder is being followed (start_encoder comes first)")
        return float(self._encoder[1].read())

    @_locked
    def stop_encoder(self) -> bool:
        """Stop following the encoder. False if none was being followed."""
        encoder, self._encoder = self._encoder, None
        if encoder is None:
            return False
        try:
            encoder[1].stop()
        finally:
            encoder[1].close()
        return True

    # ------------------------------------------------------------------
    # safe state / shutdown
    # ------------------------------------------------------------------
    @_locked
    def stop_output(self, channel: str) -> bool:
        """Return one output to its safe state: 0 V, low, or pulse train stopped.

        Only an output this object is driving is touched. Returns False when the
        channel was not being driven (nothing is sent to the device then).
        """
        name = str(channel).strip().lower()
        kind = physical_kind(name)
        if kind == "ao":
            if name not in self._ao_tasks and name not in self._ao_driven:
                return False
            self._set_ao(name, 0.0)
            return True
        if kind == "line":
            if name not in self._do_tasks:
                return False
            self._set_do(name, False)
            return True
        if kind == "counter":
            if name not in self._co_tasks:
                return False
            self._stop_counter(name)
            return True
        raise ValueError(f"{channel!r} is not an output channel (ao0, port0/line0, ctr0)")

    @_locked
    def reset_outputs(self) -> List[str]:
        """Return every output driven by this object to 0 V / low, stop every pulse train.

        Every channel is attempted even if one fails. Returns the channels that could
        NOT be reset (empty when all went to the safe state).
        """
        failed = []
        if self._pattern_task is not None:
            try:
                self._end_pattern()         # closes the task, sets its lines low
            except Exception as exc:        # its lines are retried below
                logger.error(str(exc))
        for name in list(self._co_tasks):
            try:
                self._stop_counter(name)
            except Exception as exc:
                logger.error(f"could not stop the pulse train on {name}: {exc}")
                failed.append(name)
        for name in sorted(set(self._ao_tasks) | self._ao_driven):
            try:
                self._set_ao(name, 0.0)
            except Exception as exc:
                logger.error(f"could not return {name} to 0 V: {exc}")
                failed.append(name)
        for name in sorted(set(self._do_tasks) | self._do_dirty):
            try:
                self._set_do(name, False)
            except Exception as exc:
                logger.error(f"could not return {name} to low: {exc}")
                failed.append(name)
        return failed

    @_locked
    def shutdown_outputs(self) -> List[str]:
        """The first step of ending a run: from now on no output call is accepted, and
        every output goes to its safe state. Inputs keep working so that a recording
        can be wound up; ``close()`` follows. Returns the channels not reset."""
        self._closed = True
        return self.reset_outputs()

    @_locked
    def close(self) -> List[str]:
        """Reset the outputs, then release every task. Returns the channels not reset.

        Waits for an output call still running in another thread; afterwards every
        output call is refused: one object serves one run.
        """
        self._closed = True
        failed = self.reset_outputs()       # outputs first, whatever the inputs do
        try:
            self.stop_stream()
        except Exception as exc:
            logger.warning(f"stopping the continuous acquisition failed: {exc}")
        try:
            self.stop_encoder()
        except Exception as exc:
            logger.warning(f"stopping the encoder failed: {exc}")
        for tasks in (self._ao_tasks, self._do_tasks, self._co_tasks):
            for name, task in list(tasks.items()):
                try:
                    task.close()
                except Exception as exc:
                    logger.warning(f"closing the task of {name} failed: {exc}")
            tasks.clear()
        self._ao_waveforms.clear()
        self._ao_driven.clear()
        self._do_dirty.clear()
        self._connected = False
        return failed


# Bench check (reads only, drives nothing):
#   python -m src.devices.nidaq.nidaq            -> list the devices NI-DAQmx knows
#   python -m src.devices.nidaq.nidaq Dev1 ai0   -> read one analog input
if __name__ == "__main__":
    import sys

    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    if len(sys.argv) < 2:
        try:
            print("devices:", NIDaq().list_devices())
        except Exception as exc:  # driver not installed, service stopped, ...
            sys.exit(f"NI-DAQmx is not available: {exc}")
        sys.exit(0)
    daq = NIDaq(device=sys.argv[1])
    if not daq.connect():
        sys.exit(1)
    print(f"{daq.device}: {daq.product_type}")
    for ch in sys.argv[2:]:
        print(f"{ch}: {daq.read_analog(ch, samples=100):+.5f} V (mean of 100 samples at 1 kHz)")
    daq.close()
