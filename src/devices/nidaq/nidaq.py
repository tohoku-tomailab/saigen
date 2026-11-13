"""Driver for a National Instruments multifunction DAQ device (written for the USB-6351).

Basic, software-timed I/O through NI-DAQmx:

- ``read_analog``   mean of N samples of one analog input (hardware-clocked when N > 1)
- ``write_analog``  set one analog output to a DC voltage
- ``read_digital``  read one digital line
- ``write_digital`` drive one digital line high or low

Requirements: the NI-DAQmx driver (from ni.com, Windows or Linux) and the ``nidaqmx``
Python package. ``nidaqmx`` is imported inside the methods, so importing this module,
the mock path and the tests need neither.

The driver knows nothing about flows. Which outputs a run may drive is decided by
``DaqOutputPolicy`` in the safety wrapper (``SharedDevices``), not here.

Outputs keep their task open until ``close()``, which first returns every output this
object drove to 0 V / low. An X Series device otherwise holds its last output value
after the program has gone.

    from src.devices.nidaq import NIDaq

    daq = NIDaq(device="Dev1")
    if daq.connect():
        print(daq.read_analog("ai0", samples=100))
        daq.close()
"""

import logging
from typing import Dict, List, Optional

from .policy import (DEVICE_AO_MAX_V, DEVICE_AO_MIN_V, TERMINALS, normalize_ai, normalize_ao,
                     normalize_line)

logger = logging.getLogger(__name__)


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
        self._do_tasks: Dict[str, object] = {}
        self._do_states: Dict[str, bool] = {}
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
        self._connected = True
        return True

    def _physical(self, channel: str) -> str:
        return f"{self.device}/{channel}"

    # ------------------------------------------------------------------
    # analog input
    # ------------------------------------------------------------------
    def read_analog(self, channel: str, samples: int = 1, rate_hz: float = 1000.0,
                    v_range: float = 10.0, terminal: str = "default") -> float:
        """Mean voltage of ``samples`` readings of one analog input.

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
        nidaqmx = _load_nidaqmx()
        constants = nidaqmx.constants
        terminal_config = getattr(constants.TerminalConfiguration, terminal.upper())
        with nidaqmx.Task() as task:
            task.ai_channels.add_ai_voltage_chan(
                self._physical(name), terminal_config=terminal_config,
                min_val=-float(v_range), max_val=float(v_range))
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

    # ------------------------------------------------------------------
    # analog output
    # ------------------------------------------------------------------
    def write_analog(self, channel: str, voltage: float) -> float:
        """Set one analog output to a DC voltage and hold it. Returns the value set."""
        name = normalize_ao(channel)
        volts = float(voltage)
        if not (DEVICE_AO_MIN_V <= volts <= DEVICE_AO_MAX_V):
            raise ValueError(f"{name}: {volts:g} V is outside the device span "
                             f"{DEVICE_AO_MIN_V:g} .. {DEVICE_AO_MAX_V:g} V")
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
        return volts

    # ------------------------------------------------------------------
    # digital I/O
    # ------------------------------------------------------------------
    def write_digital(self, line: str, state: bool) -> bool:
        """Drive one digital line high (True) or low (False) and hold it."""
        name = normalize_line(line)
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
        return bool(state)

    def read_digital(self, line: str) -> bool:
        """Read one digital line as an input.

        A line this object is driving is refused: configuring it as an input would
        release the output.
        """
        name = normalize_line(line)
        if name in self._do_tasks:
            raise RuntimeError(f"{name} is being driven as an output in this session; "
                               "reading it as an input would release it")
        nidaqmx = _load_nidaqmx()
        with nidaqmx.Task() as task:
            task.di_channels.add_di_chan(
                self._physical(name),
                line_grouping=nidaqmx.constants.LineGrouping.CHAN_PER_LINE)
            return bool(task.read())

    # ------------------------------------------------------------------
    # safe state / shutdown
    # ------------------------------------------------------------------
    def reset_outputs(self) -> List[str]:
        """Return every output driven by this object to 0 V / low.

        Every channel is attempted even if one fails. Returns the channels that could
        NOT be reset (empty when all went to the safe state).
        """
        failed = []
        for name, task in self._ao_tasks.items():
            try:
                task.write(0.0, auto_start=True)
            except Exception as exc:
                logger.error(f"could not return {name} to 0 V: {exc}")
                failed.append(name)
        for name, task in self._do_tasks.items():
            try:
                task.write(False, auto_start=True)
                self._do_states[name] = False
            except Exception as exc:
                logger.error(f"could not return {name} to low: {exc}")
                failed.append(name)
        return failed

    def close(self) -> List[str]:
        """Reset the outputs, then release every task. Returns the channels not reset."""
        failed = self.reset_outputs()
        for tasks in (self._ao_tasks, self._do_tasks):
            for name, task in list(tasks.items()):
                try:
                    task.close()
                except Exception as exc:
                    logger.warning(f"closing the task of {name} failed: {exc}")
            tasks.clear()
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
