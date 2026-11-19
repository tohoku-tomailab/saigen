"""The ``daq_*`` flow steps: names resolved, outputs checked, then the device.

``DaqService`` sits between the flow executor and a DAQ backend (``NIDaq`` on the
bench, ``MockNIDaq`` for ``--mock``). Both ``SharedDevices`` and ``MockSharedDevices``
own one, so the real and the mock run go through exactly this code:

1. the channel a step names is resolved (``DaqSignals``: "photodiode" -> ai0, diff, mW),
2. an output is checked against ``DaqOutputPolicy`` (refused unless config.yaml lists it),
3. only then the backend is called.

Each step returns a dict for the run log: ``daq_channel`` and, where they apply,
``voltage`` (V), ``digital_state``, ``daq_value`` + ``daq_unit`` (scaled value, count,
Hz) and ``data_path`` (CSV of a waveform or of a logging session).
"""

import asyncio
import csv
import logging
import math
import os
import statistics
import threading
import time
from datetime import datetime
from typing import List, Optional, Sequence

from .policy import (EDGES, WAVEFORMS, DaqOutputPolicy, DaqPolicyError, DaqSignals, Signal,
                     check_differential, check_timed_lines, profile_from_config)
from .profiles import USB_6351, DaqProfileError, DeviceProfile

logger = logging.getLogger(__name__)

#: Step defaults (the flow schema documents the same values).
DEFAULT_SAMPLES = 100
DEFAULT_RATE_HZ = 1000.0
DEFAULT_V_RANGE = 10.0
DEFAULT_TERMINAL = "default"

#: Upper bound of one daq_acquire (points summed over the channels): the CSV stays usable.
MAX_ACQUIRE_POINTS = 2_000_000
#: Longest digital pattern of one step (s).
MAX_PATTERN_S = 60.0
#: Points per cycle of a generated waveform (fewer only when the frequency demands it).
WAVEFORM_POINTS = 1000
WAVEFORM_MIN_POINTS = 20
#: How often the logging thread moves samples from the device to the CSV (s).
LOG_FLUSH_S = 0.2
#: How long daq_stop_logging waits for the last samples still on their way (s).
LOG_TAIL_S = 2.0

ACTIONS = ("daq_read_analog", "daq_write_analog", "daq_read_digital", "daq_write_digital",
           "daq_acquire", "daq_start_logging", "daq_stop_logging",
           "daq_generate_waveform", "daq_stop_output",
           "daq_pulse_digital", "daq_wait_digital", "daq_wait_analog",
           "daq_count_edges", "daq_measure_frequency", "daq_generate_pulses",
           "daq_stimulate_acquire", "daq_acquire_digital", "daq_write_digital_pattern",
           "daq_wait_digital_change", "daq_measure_pulse_width", "daq_measure_period",
           "daq_measure_edge_separation", "daq_start_encoder", "daq_read_encoder",
           "daq_stop_encoder")


def waveform_period(waveform: str, amplitude_v: float, offset_v: float, points: int) -> List[float]:
    """One cycle of a waveform, in volts: offset +/- amplitude, starting at the offset
    (sine, triangle) or at the low level (square, sawtooth)."""
    if waveform not in WAVEFORMS:
        raise ValueError(f"waveform must be one of {WAVEFORMS}, got {waveform!r}")
    out = []
    for i in range(points):
        x = i / points
        if waveform == "sine":
            y = math.sin(2.0 * math.pi * x)
        elif waveform == "square":
            y = -1.0 if x < 0.5 else 1.0
        elif waveform == "triangle":
            y = 4.0 * x if x < 0.25 else (2.0 - 4.0 * x if x < 0.75 else 4.0 * x - 4.0)
        else:  # sawtooth
            y = 2.0 * x - 1.0
        out.append(offset_v + amplitude_v * y)
    return out


def _columns(signals: Sequence[Signal]) -> List[str]:
    header = ["time_s"]
    for sig in signals:
        if sig.sensor is None:              # a sensor channel has no voltage to show
            header.append(f"{sig.label}_V")
        if sig.scaled:
            header.append(f"{sig.label}_{sig.unit or 'scaled'}")
    return header


def _time(index: int, rate_hz: float) -> str:
    # microseconds are enough up to 100 kHz; above that neighbours would share a time
    return f"{index / rate_hz:.6f}" if rate_hz <= 1e5 else f"{index / rate_hz:.9f}"


def _row(index: int, rate_hz: float, volts: Sequence[float], signals: Sequence[Signal]) -> list:
    row = [_time(index, rate_hz)]
    for sig, v in zip(signals, volts):
        if sig.sensor is None:
            row.append(f"{v:.6f}")
        if sig.scaled:
            row.append(f"{sig.value(v):.6g}")
    return row


def _default_path(tag: str) -> str:
    return os.path.join("daq_data", f"{tag}_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}.csv")


class _LoggingSession:
    def __init__(self, path, signals, rate_hz):
        self.path, self.signals, self.rate_hz = path, signals, rate_hz
        self.stream = None              # the backend's number of this session's stream
        self.samples = 0
        self.t0 = time.monotonic()      # when the acquisition started
        self.error: Optional[BaseException] = None
        self.stop = threading.Event()
        self.abandoned = threading.Event()   # the stream is no longer this session's
        self.thread: Optional[threading.Thread] = None


class DaqService:
    """Executes the ``daq_*`` steps on one backend, under one configuration."""

    def __init__(self, backend, config: Optional[dict] = None, mock: bool = False):
        """
        Args:
            backend: ``NIDaq`` or ``MockNIDaq``
            config: the ``shared_devices`` section (daq_signals, daq_ao_channels, ...)
            mock: waits are cut short and wait conditions are taken as met

        Raises:
            DaqPolicyError: the configuration is unusable
        """
        self.backend = backend
        self.mock = mock
        #: What the model in use has. The backend knows it: the real driver from the
        #: device (after connect), the mock from config.yaml (daq_model).
        self.profile: DeviceProfile = getattr(backend, "profile", None) or USB_6351
        # config.yaml must name the model in use ("" = the default model): --mock imitates
        # what it names, so a difference would let a flow pass --mock and fail on the bench.
        named = profile_from_config(config)
        if named is not self.profile:
            raise DaqPolicyError(
                f"shared_devices.daq_model is {(config or {}).get('daq_model') or '(empty)'}, "
                f"which means {named.model}, but the device is a {self.profile.model}; set "
                f"daq_model: {self.profile.model} in config.yaml so that --mock imitates the "
                "device that is connected")
        self.policy = DaqOutputPolicy.from_config(config or {}, self.profile)
        self.signals = DaqSignals.from_config(config or {}, self.profile)
        self._logging: Optional[_LoggingSession] = None
        self._encoder_label = "encoder"
        self._tag = "[MOCK] " if mock else ""

    # ------------------------------------------------------------------
    # plumbing
    # ------------------------------------------------------------------
    async def run(self, action: str, **params) -> dict:
        """Execute one ``daq_*`` step. ``params`` are the step's fields except ``action``."""
        if action not in ACTIONS:
            raise ValueError(f"unknown DAQ action: {action!r}")
        handler = getattr(self, "_" + action[len("daq_"):])
        self.check_step(action, params)
        try:
            return await handler(**params)
        except DaqPolicyError:
            raise                       # nothing was sent to the device
        except Exception as exc:
            logger.error(f"DAQ {action} failed: {exc}")
            raise

    def check_step(self, action: str, params: Optional[dict] = None) -> None:
        """Refuse a step the model in use cannot run (nothing is sent to the device)."""
        try:
            self.profile.check_step(action, params)
        except DaqProfileError as exc:
            raise DaqPolicyError(str(exc)) from None

    def unsupported_steps(self, steps) -> List[str]:
        """Steps and options of a flow that the model does not have at all, one line per
        step (1-based). Channel names, ranges and rates are checked when the step runs."""
        problems = []
        for index, step in enumerate(steps, start=1):
            action = step.get("action") if isinstance(step, dict) else getattr(step, "action", None)
            if action not in ACTIONS:
                continue
            params = step if isinstance(step, dict) else step.__dict__
            try:
                self.check_step(action, params)
            except DaqPolicyError as exc:
                problems.append(f"step {index}: {exc}")
        return problems

    async def _sleep(self, seconds: float) -> None:
        await asyncio.sleep(min(seconds, 0.02) if self.mock else seconds)

    async def _call(self, fn, *args, **kwargs):
        """Run a blocking device call off the event loop."""
        return await asyncio.to_thread(fn, *args, **kwargs)

    def _actual_ai_rate(self, requested: float) -> float:
        """The sample rate the device really used (DAQmx coerces it); it is the time axis."""
        actual = getattr(self.backend, "ai_rate_hz", None) or requested
        if abs(actual - requested) > 1e-9 * requested:
            logger.warning(f"  {self._tag}DAQ: 要求 {requested:g} Hz に対し、装置の実サンプルレートは "
                           f"{actual:.9g} Hz です（時間軸は実レートで記録します）")
        return float(actual)

    def _ai(self, name, v_range=None, terminal=None):
        """Resolve an analog input; a value given in the step wins over the signal's."""
        sig = self.signals.resolve(name, "ai")
        if sig.sensor is not None and v_range is not None:
            raise ValueError(f"{sig.label} is a {sig.sensor['type']} channel: v_range does not "
                             "apply (its range is min_c / max_c in config.yaml)")
        p = self.profile
        default_v = DEFAULT_V_RANGE if DEFAULT_V_RANGE in p.v_ranges else max(p.v_ranges)
        v = float(v_range if v_range is not None else (sig.v_range or default_v))
        t = terminal if terminal is not None else (sig.terminal or DEFAULT_TERMINAL)
        if v not in p.v_ranges:
            raise ValueError(f"v_range must be one of {p.v_ranges} on the {p.model}, got {v:g}")
        if t not in p.ai_terminals:
            raise ValueError(f"terminal must be one of {p.ai_terminals} on the {p.model}, "
                             f"got {t!r}")
        if t == "diff":
            check_differential(p, sig.channel)
        return sig, v, t

    @staticmethod
    def _finite(sig: Signal, reading: float) -> float:
        """A reading that is not a number (an open sensor, a failed conversion) is an
        error of the step, not a measurement."""
        if not (math.isfinite(reading) and math.isfinite(sig.value(reading))):
            raise RuntimeError(f"{sig.label}: the reading is not a finite number "
                               f"({reading} {sig.raw_unit})")
        return reading

    @staticmethod
    def _spec(sig: Signal, v: float, t: str) -> tuple:
        """What the backend needs to open the channel."""
        return (sig.channel, v, t) if sig.sensor is None else (sig.channel, v, t, sig.sensor)

    @staticmethod
    def _with_value(result: dict, sig: Signal, volts: float) -> dict:
        """Put a reading on record: volts (unless it is a sensor channel, which has
        none) and the converted value with its unit."""
        if sig.sensor is not None:
            result.pop("voltage", None)
        if sig.scaled:
            result["daq_value"] = sig.value(volts)
            result["daq_unit"] = sig.unit
        return result

    # ------------------------------------------------------------------
    # analog input
    # ------------------------------------------------------------------
    async def _read_analog(self, channel, samples=None, rate_hz=None, v_range=None,
                           terminal=None) -> dict:
        sig, v, t = self._ai(channel, v_range, terminal)
        samples = DEFAULT_SAMPLES if samples is None else int(samples)
        rate = DEFAULT_RATE_HZ if rate_hz is None else float(rate_hz)
        volts = self._finite(sig, await self._call(
            self.backend.read_analog, *self._read_args(sig, samples, rate, v, t)))
        if sig.sensor is not None:
            logger.info(f"✓ {self._tag}DAQ {sig.label}: {sig.value(volts):.6g} {sig.unit}"
                        f"（{sig.sensor['type']}、{samples} 点平均）")
        else:
            extra = f" = {sig.value(volts):.6g} {sig.unit}" if sig.scaled else ""
            logger.info(f"✓ {self._tag}DAQ {sig.label}: {volts:+.5f} V{extra}（{samples} 点平均）")
        return self._with_value({"voltage": volts, "daq_channel": sig.label}, sig, volts)

    @staticmethod
    def _read_args(sig: Signal, samples: int, rate: float, v: float, t: str) -> tuple:
        args = (sig.channel, samples, rate, v, t)
        return args if sig.sensor is None else args + (sig.sensor,)

    def _ai_group(self, channels, v_range, terminal):
        if isinstance(channels, str):
            channels = [channels]
        resolved = [self._ai(c, v_range, terminal) for c in (channels or [])]
        if not resolved:
            raise ValueError("channels: at least one analog input is needed")
        signals = [r[0] for r in resolved]
        if len({s.channel for s in signals}) != len(signals):
            raise ValueError("channels: the same analog input is listed twice")
        return signals, [self._spec(s, v, t) for s, v, t in resolved]

    async def _acquire(self, channels, rate_hz=None, duration_s=1.0, v_range=None, terminal=None,
                       trigger=None, trigger_edge="rising", timeout_s=10.0,
                       trigger_level=None, pretrigger_s=None, clock=None,
                       file_path=None) -> dict:
        signals, specs = self._ai_group(channels, v_range, terminal)
        rate = DEFAULT_RATE_HZ if rate_hz is None else float(rate_hz)
        samples = int(round(rate * float(duration_s)))
        if samples < 2:
            raise ValueError("rate_hz * duration_s must give at least 2 samples")
        if samples * len(signals) > MAX_ACQUIRE_POINTS:
            raise ValueError(f"{samples} samples x {len(signals)} channel(s) exceeds "
                             f"{MAX_ACQUIRE_POINTS} points per step; shorten it or lower rate_hz")
        if trigger_edge not in EDGES:
            raise ValueError(f"trigger_edge must be one of {EDGES}")
        extras = {}
        if trigger_level is not None:
            # an analog trigger: the first acquired channel crossing a level (in volts)
            if trigger is None:
                raise ValueError("trigger_level needs trigger = the analog input to watch")
            trig = self.signals.resolve(trigger, "ai")
            if trig.channel != signals[0].channel:
                raise ValueError(f"an analog trigger must be the first of channels "
                                 f"({signals[0].label}), got {trig.label}")
            if trig.sensor is not None:
                raise ValueError(f"{trig.label} is a sensor channel; trigger on a voltage input")
            level = float(trigger_level)
            if not math.isfinite(level) or abs(level) > specs[0][1]:
                raise ValueError(f"trigger_level = {level:g} V is outside the input range "
                                 f"±{specs[0][1]:g} V of {trig.label}")
            extras["trigger_level"] = level
        else:
            trig = self.signals.resolve(trigger, "terminal") if trigger is not None else None
        pre = 0
        if pretrigger_s is not None:
            if trig is None:
                raise ValueError("pretrigger_s needs a trigger")
            pre = int(round(rate * float(pretrigger_s)))
            if not (2 <= pre <= samples - 2):
                raise ValueError("pretrigger_s must leave at least 2 samples before and after "
                                 "the trigger (0 < pretrigger_s < duration_s)")
            extras["pretrigger"] = pre
        if clock is not None:
            extras["clock"] = self.signals.resolve(clock, "terminal").channel
        if trig is not None:
            what = (f"{extras['trigger_level']:g} V を {trigger_edge} で横切るの"
                    if "trigger_level" in extras else f"{trigger_edge} エッジ")
            logger.info(f"  {self._tag}DAQ: {trig.label} の {what}を待ちます"
                        f"（最大 {float(timeout_s):g} 秒）")
        data = await self._call(self.backend.acquire, specs, rate, samples,
                                trig.channel if trig else None, trigger_edge, float(timeout_s),
                                **extras)
        rate = self._actual_ai_rate(rate)
        path = file_path or _default_path("acquire")
        await self._call(self._write_waveform, path, signals, rate, data, -pre)
        for sig, column in zip(signals, data):
            logger.info(f"✓ {self._tag}DAQ {sig.label}: {samples} 点 @ {rate:g} Hz  "
                        f"mean {statistics.fmean(column):+.5f} {sig.raw_unit}  min {min(column):+.5f}  "
                        f"max {max(column):+.5f}  sd {statistics.pstdev(column):.5f}")
        logger.info(f"  波形を保存: {path}")
        result = {"daq_channel": "+".join(s.label for s in signals), "data_path": path}
        if len(signals) == 1:
            mean = self._finite(signals[0], statistics.fmean(data[0]))
            result["voltage"] = mean
            self._with_value(result, signals[0], mean)
        return result

    @staticmethod
    def _write_waveform(path, signals, rate_hz, data, first_index=0, extra=None) -> None:
        """``first_index`` < 0: that many samples lie before the trigger (time 0).
        ``extra`` = (header, values): one more column, e.g. the commanded stimulus."""
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, "w", encoding="utf-8", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(_columns(signals) + ([extra[0]] if extra else []))
            for i, volts in enumerate(zip(*data)):
                row = _row(i + first_index, rate_hz, volts, signals)
                if extra:
                    row.append(f"{extra[1][i]:.6f}")
                writer.writerow(row)

    async def _start_logging(self, channels, rate_hz=None, v_range=None, terminal=None,
                             file_path=None) -> dict:
        if self._logging is not None:
            raise RuntimeError("a logging session is already running; "
                               "stop it with daq_stop_logging first")
        signals, specs = self._ai_group(channels, v_range, terminal)
        rate = 100.0 if rate_hz is None else float(rate_hz)
        path = file_path or _default_path("logging")
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, "w", encoding="utf-8", newline="") as f:
            csv.writer(f).writerow(_columns(signals))
        stream = await self._call(self.backend.start_stream, specs, rate)
        rate = self._actual_ai_rate(rate)
        session = _LoggingSession(path, signals, rate)   # t0 = now: the stream has just started
        session.stream = stream
        session.thread = threading.Thread(target=self._log_loop, args=(session,),
                                          name="daq-logging", daemon=True)
        try:
            session.thread.start()
        except BaseException:
            self.backend.stop_stream()      # nobody would read it
            raise
        self._logging = session
        logger.info(f"✓ {self._tag}DAQ 連続記録を開始: "
                    f"{', '.join(s.label for s in signals)} @ {rate:g} Hz -> {path}")
        return {"daq_channel": "+".join(s.label for s in signals), "data_path": path}

    def _log_loop(self, session: _LoggingSession) -> None:
        try:
            with open(session.path, "a", encoding="utf-8", newline="") as f:
                writer = csv.writer(f)
                while not session.stop.wait(LOG_FLUSH_S):
                    self._drain(session, writer)
                    f.flush()
                # The device hands samples over in blocks (USB): wait for the ones
                # acquired up to the stop request instead of cutting the tail off.
                target = int((time.monotonic() - session.t0) * session.rate_hz)
                deadline = time.monotonic() + LOG_TAIL_S
                while True:
                    self._drain(session, writer)
                    if session.samples >= target or time.monotonic() >= deadline:
                        break
                    time.sleep(0.02)
                if session.samples < 0.9 * target and not session.abandoned.is_set():
                    raise RuntimeError(f"only {session.samples} of about {target} samples "
                                       "arrived; the recording is incomplete")
        except Exception as exc:  # reported by daq_stop_logging / at the end of the run
            session.error = exc
            logger.error(f"DAQ 連続記録が止まりました: {exc}")

    def _drain(self, session: _LoggingSession, writer) -> None:
        if session.abandoned.is_set():      # never touch a stream that is no longer ours
            return
        data = self.backend.read_stream(session.stream)   # never another session's samples
        for volts in zip(*data):
            writer.writerow(_row(session.samples, session.rate_hz, volts, session.signals))
            session.samples += 1

    def _finish_logging(self) -> Optional[_LoggingSession]:
        """Stop the logging thread and the acquisition (blocking). None if none ran."""
        session, self._logging = self._logging, None
        if session is None:
            return None
        session.stop.set()
        if session.thread is not None:
            session.thread.join(timeout=15.0)
            if session.thread.is_alive():
                session.abandoned.set()
                session.error = session.error or RuntimeError(
                    "the logging thread did not stop within 15 s; the CSV may be incomplete")
        try:
            self.backend.stop_stream()
        except Exception as exc:
            session.error = session.error or exc
        return session

    async def _stop_logging(self) -> dict:
        if self._logging is None:
            raise RuntimeError("no logging session is running (daq_start_logging comes first)")
        session = await self._call(self._finish_logging)
        if session.error is not None:
            raise RuntimeError(f"the logging session failed after {session.samples} samples: "
                               f"{session.error} (data so far: {session.path})")
        logger.info(f"✓ {self._tag}DAQ 連続記録を終了: {session.samples} 点 "
                    f"（{session.samples / session.rate_hz:.2f} 秒分） -> {session.path}")
        return {"daq_channel": "+".join(s.label for s in session.signals),
                "data_path": session.path, "daq_value": session.samples, "daq_unit": "samples"}

    async def _wait_analog(self, channel, threshold, condition="above", timeout_s=60.0,
                           samples=10, v_range=None, terminal=None, poll_s=0.05) -> dict:
        if condition not in ("above", "below"):
            raise ValueError("condition must be 'above' or 'below'")
        sig, v, t = self._ai(channel, v_range, terminal)
        threshold = float(threshold)
        if not math.isfinite(threshold):
            raise ValueError("threshold must be a finite number")
        unit = sig.unit if sig.scaled else "V"
        deadline = time.monotonic() + float(timeout_s)
        while True:
            volts = await self._call(self.backend.read_analog,
                                     *self._read_args(sig, int(samples), DEFAULT_RATE_HZ, v, t))
            value = sig.value(volts)
            if not math.isfinite(value):
                raise RuntimeError(f"{sig.label}: the reading is not a finite number ({value})")
            met = value > threshold if condition == "above" else value < threshold
            if met or self.mock:
                note = "" if met else "（Mock: 条件は成立したものとして進みます）"
                logger.info(f"✓ {self._tag}DAQ {sig.label}: {value:.6g} {unit} "
                            f"（{condition} {threshold:g} {unit}）{note}")
                return self._with_value({"voltage": volts, "daq_channel": sig.label}, sig, volts)
            if time.monotonic() >= deadline:
                raise TimeoutError(
                    f"{sig.label} did not go {condition} {threshold:g} {unit} within "
                    f"{float(timeout_s):g} s (last value {value:.6g} {unit})")
            await self._sleep(float(poll_s))

    # ------------------------------------------------------------------
    # analog output
    # ------------------------------------------------------------------
    async def _write_analog(self, channel, voltage) -> dict:
        sig = self.signals.resolve(channel, "ao")
        name, volts = self.policy.check_analog_output(sig.channel, voltage)
        await self._call(self.backend.write_analog, name, volts)
        logger.info(f"✓ {self._tag}DAQ {sig.label} = {volts:+.4f} V")
        return {"voltage": volts, "daq_channel": sig.label}

    async def _generate_waveform(self, channel, frequency_hz, amplitude_v, waveform="sine",
                                 offset_v=0.0, duration_s=None) -> dict:
        sig = self.signals.resolve(channel, "ao")
        name, offset, amplitude = self.policy.check_analog_waveform(sig.channel, offset_v,
                                                                    amplitude_v)
        freq = float(frequency_hz)
        if not freq > 0.0:
            raise ValueError("frequency_hz must be > 0")
        points = min(WAVEFORM_POINTS, int(self.profile.ao_max_rate_hz // freq))
        if points < WAVEFORM_MIN_POINTS:
            raise ValueError(f"frequency_hz = {freq:g} is too high: fewer than "
                             f"{WAVEFORM_MIN_POINTS} points per cycle at {self.profile.ao_max_rate_hz:g} points/s")
        period = waveform_period(waveform, amplitude, offset, points)
        actual = await self._call(self.backend.start_waveform, name, period, freq * points)
        if actual and abs(actual - freq * points) > 1e-9 * freq * points:
            logger.warning(f"  {self._tag}DAQ: 要求 {freq:g} Hz に対し、実際の波形周波数は "
                           f"{actual / points:.9g} Hz です")
            freq = actual / points
        logger.info(f"✓ {self._tag}DAQ {sig.label}: {waveform} {freq:g} Hz, "
                    f"{offset:+.4f} ± {amplitude:.4f} V"
                    + (f"（{float(duration_s):g} 秒間）" if duration_s is not None
                       else "（daq_stop_output か実行終了まで継続）"))
        if duration_s is not None:
            try:
                await self._sleep(float(duration_s))
            finally:
                self.backend.stop_output(name)
                logger.info(f"✓ {self._tag}DAQ {sig.label} = 0 V（波形出力を終了）")
        return {"daq_channel": sig.label, "daq_value": freq, "daq_unit": "Hz"}

    async def _stop_output(self, channel) -> dict:
        sig = self.signals.resolve(channel, ("ao", "line", "counter"))
        was_driven = await self._call(self.backend.stop_output, sig.channel)
        state = {"ao": "0 V", "line": "Low", "counter": "停止"}[sig.kind]
        logger.info(f"✓ {self._tag}DAQ {sig.label}: {state}"
                    + ("" if was_driven else "（このランでは駆動していないので何も送っていません）"))
        result = {"daq_channel": sig.label}
        if not was_driven:
            return result                   # nothing was sent: no level to put on record
        if sig.kind == "ao":
            result["voltage"] = 0.0
        elif sig.kind == "line":
            result["digital_state"] = False
        return result

    # ------------------------------------------------------------------
    # digital
    # ------------------------------------------------------------------
    async def _read_digital(self, line) -> dict:
        sig = self.signals.resolve(line, "line")
        state = bool(await self._call(self.backend.read_digital, sig.channel))
        logger.info(f"✓ {self._tag}DAQ {sig.label}: {'High' if state else 'Low'}")
        return {"digital_state": state, "daq_channel": sig.label}

    async def _write_digital(self, line, state) -> dict:
        sig = self.signals.resolve(line, "line")
        name = self.policy.check_digital_output(sig.channel)
        await self._call(self.backend.write_digital, name, bool(state))
        logger.info(f"✓ {self._tag}DAQ {sig.label} = {'High' if state else 'Low'}")
        return {"digital_state": bool(state), "daq_channel": sig.label}

    async def _pulse_digital(self, line, duration_s, state=True) -> dict:
        sig = self.signals.resolve(line, "line")
        name = self.policy.check_digital_output(sig.channel)
        duration = float(duration_s)
        if not duration > 0.0:
            raise ValueError("duration_s must be > 0")
        active = bool(state)
        self.backend.write_digital(name, active)
        try:
            await self._sleep(duration)
        except BaseException:
            self.backend.stop_output(name)  # interrupted: low is the safe state, whatever the pulse
            raise
        self.backend.write_digital(name, not active)
        logger.info(f"✓ {self._tag}DAQ {sig.label}: {'High' if active else 'Low'} を "
                    f"{duration:g} 秒 -> {'Low' if active else 'High'}")
        return {"digital_state": not active, "daq_channel": sig.label}

    async def _wait_digital(self, line, state=True, timeout_s=60.0, poll_s=0.01) -> dict:
        sig = self.signals.resolve(line, "line")
        wanted = bool(state)
        deadline = time.monotonic() + float(timeout_s)
        while True:
            now = bool(await self._call(self.backend.read_digital, sig.channel))
            if now == wanted or self.mock:
                note = "" if now == wanted else "（Mock: 条件は成立したものとして進みます）"
                logger.info(f"✓ {self._tag}DAQ {sig.label}: {'High' if wanted else 'Low'}{note}")
                return {"digital_state": wanted, "daq_channel": sig.label}
            if time.monotonic() >= deadline:
                raise TimeoutError(f"{sig.label} did not go {'high' if wanted else 'low'} "
                                   f"within {float(timeout_s):g} s")
            await self._sleep(float(poll_s))

    # ------------------------------------------------------------------
    # counters
    # ------------------------------------------------------------------
    def _counter(self, counter) -> Optional[str]:
        return None if counter is None else self.signals.resolve(counter, "counter").channel

    async def _count_edges(self, terminal, duration_s=1.0, edge="rising", counter=None) -> dict:
        sig = self.signals.resolve(terminal, "terminal")
        duration = float(duration_s)
        if self.mock:
            count = self.backend.count_edges(sig.channel, duration, edge, self._counter(counter))
            await self._sleep(duration)
        else:
            count = await self._call(self.backend.count_edges, sig.channel, duration, edge,
                                     self._counter(counter))
        logger.info(f"✓ {self._tag}DAQ {sig.label}: {count} エッジ / {duration:g} 秒 "
                    f"（{count / duration:g} Hz 相当）")
        return {"daq_channel": sig.label, "daq_value": int(count), "daq_unit": "count"}

    async def _measure_frequency(self, terminal, min_hz=1.0, max_hz=100000.0, timeout_s=5.0,
                                 edge="rising", counter=None) -> dict:
        sig = self.signals.resolve(terminal, "terminal")
        freq = await self._call(self.backend.measure_frequency, sig.channel, float(min_hz),
                                float(max_hz), float(timeout_s), edge, self._counter(counter))
        logger.info(f"✓ {self._tag}DAQ {sig.label}: {freq:.6g} Hz")
        return {"daq_channel": sig.label, "daq_value": float(freq), "daq_unit": "Hz"}

    async def _generate_pulses(self, counter, frequency_hz, duty_cycle=0.5, count=None,
                               duration_s=None) -> dict:
        sig = self.signals.resolve(counter, "counter")
        name, freq, duty = self.policy.check_counter_output(sig.channel, frequency_hz, duty_cycle)
        if count is not None and duration_s is not None:
            raise ValueError("give count or duration_s, not both")
        n = None if count is None else int(count)
        await self._call(self.backend.start_pulses, name, freq, duty, n)
        how = (f"{n} パルス" if n is not None else
               f"{float(duration_s):g} 秒間" if duration_s is not None else
               "daq_stop_output か実行終了まで継続")
        logger.info(f"✓ {self._tag}DAQ {sig.label}: パルス列 {freq:g} Hz, duty {duty:g}（{how}）")
        if n is not None or duration_s is not None:
            try:
                if n is not None:
                    deadline = time.monotonic() + n / freq + 5.0
                    while not self.backend.pulses_done(name):
                        if time.monotonic() >= deadline:
                            raise TimeoutError(f"{sig.label}: the pulse train did not finish")
                        await self._sleep(min(0.05, 1.0 / freq))
                else:
                    await self._sleep(float(duration_s))
            finally:
                self.backend.stop_output(name)
                logger.info(f"✓ {self._tag}DAQ {sig.label}: パルス列を終了")
        return {"daq_channel": sig.label, "daq_value": freq, "daq_unit": "Hz"}

    # ------------------------------------------------------------------
    # stimulus and response
    # ------------------------------------------------------------------
    async def _stimulate_acquire(self, channel, frequency_hz, amplitude_v, channels,
                                 waveform="sine", offset_v=0.0, rate_hz=None, duration_s=1.0,
                                 v_range=None, terminal=None, file_path=None) -> dict:
        out = self.signals.resolve(channel, "ao")
        name, offset, amplitude = self.policy.check_analog_waveform(out.channel, offset_v,
                                                                    amplitude_v)
        signals, specs = self._ai_group(channels, v_range, terminal)
        freq = float(frequency_hz)
        if not freq > 0.0:
            raise ValueError("frequency_hz must be > 0")
        points = min(WAVEFORM_POINTS, int(self.profile.ao_max_rate_hz // freq))
        if points < WAVEFORM_MIN_POINTS:
            raise ValueError(f"frequency_hz = {freq:g} is too high: fewer than "
                             f"{WAVEFORM_MIN_POINTS} points per cycle at {self.profile.ao_max_rate_hz:g} points/s")
        rate = DEFAULT_RATE_HZ if rate_hz is None else float(rate_hz)
        samples = int(round(rate * float(duration_s)))
        if samples < 2:
            raise ValueError("rate_hz * duration_s must give at least 2 samples")
        if samples * len(signals) > MAX_ACQUIRE_POINTS:
            raise ValueError(f"{samples} samples x {len(signals)} channel(s) exceeds "
                             f"{MAX_ACQUIRE_POINTS} points per step; shorten it or lower rate_hz")
        period = waveform_period(waveform, amplitude, offset, points)
        data, ao_rate = await self._call(self.backend.stimulate_acquire, name, period,
                                         freq * points, specs, rate, samples)
        rate = self._actual_ai_rate(rate)
        ao_rate = float(ao_rate or freq * points)
        freq = ao_rate / points
        path = file_path or _default_path("stimulate")
        # the stimulus as it was commanded, at the nominal acquisition times: both tasks
        # start on one trigger, but their clocks are separate, so the real alignment is
        # good to about one sample and has to be checked by loop-back
        # (the small term keeps an exact boundary from rounding down to the point before)
        stimulus = [period[int(i * ao_rate / rate + 1e-6) % points] for i in range(samples)]
        await self._call(self._write_waveform, path, signals, rate, data, 0,
                         (f"{out.label}_set_V", stimulus))
        for sig, column in zip(signals, data):
            logger.info(f"✓ {self._tag}DAQ {out.label} -> {sig.label}: {samples} 点 @ {rate:g} Hz  "
                        f"mean {statistics.fmean(column):+.5f} {sig.raw_unit}  "
                        f"min {min(column):+.5f}  max {max(column):+.5f}")
        logger.info(f"  {waveform} {freq:g} Hz, {offset:+.4f} ± {amplitude:.4f} V と同時に取り込み"
                    f"（出力は 0 V に戻しました）: {path}")
        return {"daq_channel": f"{out.label}>" + "+".join(s.label for s in signals),
                "data_path": path, "daq_value": freq, "daq_unit": "Hz"}

    # ------------------------------------------------------------------
    # hardware-timed digital I/O
    # ------------------------------------------------------------------
    def _lines(self, lines) -> List[Signal]:
        if isinstance(lines, str):
            lines = [lines]
        return [self.signals.resolve(line, "line") for line in (lines or [])]

    @staticmethod
    def _write_digital_csv(path, rate_hz, columns) -> None:
        """columns = [(header, [states...]), ...]"""
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, "w", encoding="utf-8", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["time_s"] + [header for header, _ in columns])
            for i, states in enumerate(zip(*[column for _, column in columns])):
                writer.writerow([_time(i, rate_hz)] + [int(bool(s)) for s in states])

    async def _acquire_digital(self, lines, rate_hz=None, duration_s=1.0, clock=None,
                               file_path=None) -> dict:
        signals = self._lines(lines)
        names = check_timed_lines([s.channel for s in signals], self.profile)
        rate = DEFAULT_RATE_HZ if rate_hz is None else float(rate_hz)
        samples = int(round(rate * float(duration_s)))
        if samples < 2:
            raise ValueError("rate_hz * duration_s must give at least 2 samples")
        if samples * len(names) > MAX_ACQUIRE_POINTS:
            raise ValueError(f"{samples} samples x {len(names)} line(s) exceeds "
                             f"{MAX_ACQUIRE_POINTS} points per step; shorten it or lower rate_hz")
        clk = self.signals.resolve(clock, "terminal") if clock is not None else None
        data = await self._call(self.backend.acquire_digital, names, rate, samples,
                                clk.channel if clk else None)
        rate = float(getattr(self.backend, "di_rate_hz", None) or rate)
        path = file_path or _default_path("digital")
        await self._call(self._write_digital_csv, path, rate,
                         [(s.label, column) for s, column in zip(signals, data)])
        for sig, column in zip(signals, data):
            changes = sum(1 for a, b in zip(column, column[1:]) if a != b)
            logger.info(f"✓ {self._tag}DAQ {sig.label}: {samples} 点 @ {rate:g} Hz  "
                        f"High {sum(column)} 点, 変化 {changes} 回")
        logger.info(f"  デジタル波形を保存: {path}")
        return {"daq_channel": "+".join(s.label for s in signals), "data_path": path}

    async def _write_digital_pattern(self, lines, pattern, rate_hz, record=None,
                                     file_path=None) -> dict:
        signals = self._lines(lines)
        names = self.policy.check_digital_pattern([s.channel for s in signals])
        rows = []
        for row in pattern or []:
            if isinstance(row, str):
                if set(row) - {"0", "1"}:
                    raise ValueError(f"pattern row {row!r}: use one '0' or '1' per line")
                row = [c == "1" for c in row]
            rows.append([bool(v) for v in row])
        if len(rows) < 2 or any(len(row) != len(names) for row in rows):
            raise ValueError(f"pattern: at least 2 rows, each with {len(names)} state(s) "
                             "(one per line, in the order of lines)")
        rate = float(rate_hz)
        if not rate > 0.0 or len(rows) / rate > MAX_PATTERN_S:
            raise ValueError(f"rate_hz must be > 0 and the pattern at most {MAX_PATTERN_S:g} s long")
        recorded_signals = self._lines(record) if record else []
        recorded_names = check_timed_lines([s.channel for s in recorded_signals], self.profile) if record else None
        data = await self._call(self.backend.write_digital_pattern, names, rows, rate,
                                recorded_names)
        rate = float(getattr(self.backend, "do_rate_hz", None) or rate)   # the clock really used
        label = "+".join(s.label for s in signals)
        logger.info(f"✓ {self._tag}DAQ {label}: {len(rows)} ステップのパターン @ {rate:g} Hz "
                    f"（{len(rows) / rate:g} 秒）-> Low")
        result = {"daq_channel": label}
        if recorded_signals:
            path = file_path or _default_path("pattern")
            played = [(f"{s.label}_set", [row[j] for row in rows]) for j, s in enumerate(signals)]
            await self._call(self._write_digital_csv, path, rate,
                             played + [(s.label, column)
                                       for s, column in zip(recorded_signals, data)])
            logger.info(f"  応答（{', '.join(s.label for s in recorded_signals)}）を保存: {path}")
            result["daq_channel"] = label + ">" + "+".join(s.label for s in recorded_signals)
            result["data_path"] = path
        return result

    async def _wait_digital_change(self, lines, edge="both", timeout_s=60.0) -> dict:
        signals = self._lines(lines)
        names = check_timed_lines([s.channel for s in signals], self.profile)
        states = await self._call(self.backend.wait_digital_change, names, edge, float(timeout_s))
        label = "+".join(s.label for s in signals)
        shown = ", ".join(f"{s.label}={'High' if v else 'Low'}" for s, v in zip(signals, states))
        logger.info(f"✓ {self._tag}DAQ 変化を検出（{edge}）: {shown}"
                    + ("（Mock: 変化があったものとして進みます）" if self.mock else ""))
        result = {"daq_channel": label}
        if len(states) == 1:
            result["digital_state"] = bool(states[0])
        return result

    # ------------------------------------------------------------------
    # counters: time measurements and the encoder
    # ------------------------------------------------------------------
    async def _measure_pulse_width(self, terminal, min_s=1e-6, max_s=1.0, level="high",
                                   timeout_s=5.0, counter=None) -> dict:
        sig = self.signals.resolve(terminal, "terminal")
        width = await self._call(self.backend.measure_pulse_width, sig.channel, float(min_s),
                                 float(max_s), level, float(timeout_s), self._counter(counter))
        logger.info(f"✓ {self._tag}DAQ {sig.label}: {level} パルス幅 {width:.6g} s")
        return {"daq_channel": sig.label, "daq_value": float(width), "daq_unit": "s"}

    async def _measure_period(self, terminal, min_s=1e-6, max_s=1.0, edge="rising",
                              timeout_s=5.0, counter=None) -> dict:
        sig = self.signals.resolve(terminal, "terminal")
        period = await self._call(self.backend.measure_period, sig.channel, float(min_s),
                                  float(max_s), edge, float(timeout_s), self._counter(counter))
        logger.info(f"✓ {self._tag}DAQ {sig.label}: 周期 {period:.6g} s（{1.0 / period:.6g} Hz）")
        return {"daq_channel": sig.label, "daq_value": float(period), "daq_unit": "s"}

    async def _measure_edge_separation(self, first, second, min_s=1e-6, max_s=1.0,
                                       first_edge="rising", second_edge="rising", timeout_s=5.0,
                                       counter=None) -> dict:
        a = self.signals.resolve(first, "terminal")
        b = self.signals.resolve(second, "terminal")
        gap = await self._call(self.backend.measure_edge_separation, a.channel, b.channel,
                               float(min_s), float(max_s), first_edge, second_edge,
                               float(timeout_s), self._counter(counter))
        logger.info(f"✓ {self._tag}DAQ {a.label}（{first_edge}）-> {b.label}（{second_edge}）: "
                    f"{gap:.6g} s")
        return {"daq_channel": f"{a.label}>{b.label}", "daq_value": float(gap), "daq_unit": "s"}

    async def _start_encoder(self, a, b, pulses_per_rev, decoding="x4", counter=None) -> dict:
        sig_a = self.signals.resolve(a, "terminal")
        sig_b = self.signals.resolve(b, "terminal")
        used = await self._call(self.backend.start_encoder, sig_a.channel, sig_b.channel,
                                int(pulses_per_rev), decoding, self._counter(counter))
        self._encoder_label = f"{sig_a.label}/{sig_b.label}"
        logger.info(f"✓ {self._tag}DAQ エンコーダの追従を開始: A = {sig_a.label}, B = {sig_b.label}, "
                    f"{int(pulses_per_rev)} パルス/回転, {decoding}（{used}、0° から）")
        return {"daq_channel": self._encoder_label, "daq_value": 0.0, "daq_unit": "deg"}

    async def _read_encoder(self) -> dict:
        angle = await self._call(self.backend.read_encoder)
        logger.info(f"✓ {self._tag}DAQ エンコーダ {self._encoder_label}: {angle:+.3f}°"
                    f"（{angle / 360.0:+.4f} 回転）")
        return {"daq_channel": self._encoder_label, "daq_value": float(angle), "daq_unit": "deg"}

    async def _stop_encoder(self) -> dict:
        angle = await self._call(self.backend.read_encoder)     # raises if none is followed
        await self._call(self.backend.stop_encoder)
        logger.info(f"✓ {self._tag}DAQ エンコーダの追従を終了: {angle:+.3f}°")
        return {"daq_channel": self._encoder_label, "daq_value": float(angle), "daq_unit": "deg"}

    # ------------------------------------------------------------------
    # shutdown
    # ------------------------------------------------------------------
    def close(self) -> List[str]:
        """Stop logging, return every output to its safe state, release the device.

        Returns the outputs that could NOT be returned to the safe state.
        """
        # Outputs first, and from here no output call is accepted any more: nothing stays
        # or becomes driven while the recording is being wound up, and a failure or a
        # second Ctrl+C there cannot keep the device from being closed.
        try:
            try:
                self.backend.shutdown_outputs()     # close() below tries again
            except Exception as exc:
                logger.error(f"DAQ 出力のリセットに失敗しました（close で再試行します）: {exc}")
            try:
                session = self._finish_logging()
                if session is not None:
                    logger.warning(f"DAQ 連続記録は daq_stop_logging なしで終了しました: "
                                   f"{session.samples} 点 -> {session.path}")
                    if session.error is not None:
                        logger.error(f"DAQ 連続記録は途中で失敗していました（データは不完全です）: "
                                     f"{session.error}")
            except Exception as exc:
                logger.error(f"DAQ 連続記録の終了処理に失敗しました: {exc}")
        finally:
            failed = self.backend.close()
        return failed
