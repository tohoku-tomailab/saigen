"""Channel names and the output policy of the NI DAQ device. No hardware, no nidaqmx.

A flow names a channel without the device ("ai0", "ao1", "port0/line3"); the device
name ("Dev1") is lab-specific and comes from ``config.yaml``. Inputs are always
allowed. Outputs can switch whatever is wired to the terminal block, so they are
refused unless the channel is listed in ``config.yaml`` (``daq_ao_channels`` /
``daq_do_lines``), and an analog output must stay inside ``daq_ao_min_v`` ..
``daq_ao_max_v``. The real device and the mock use this same class, so a flow that
passes ``--mock`` is not refused for these reasons on the bench.
"""

import re
from typing import Iterable

AI_PATTERN = r"^ai\d{1,2}$"
AO_PATTERN = r"^ao\d$"
LINE_PATTERN = r"^port\d/line\d{1,2}$"

#: Voltage span of the analog outputs of the X Series devices (USB-6351: +/-10 V).
DEVICE_AO_MIN_V = -10.0
DEVICE_AO_MAX_V = 10.0

TERMINALS = ("default", "rse", "nrse", "diff")


class DaqPolicyError(ValueError):
    """An output the configuration does not allow, or a malformed channel name."""


def _normalize(name, pattern: str, kind: str) -> str:
    text = str(name).strip().lower()
    if not re.match(pattern, text):
        raise DaqPolicyError(f"{kind} channel name {name!r} is not valid "
                             f"(expected e.g. {_EXAMPLES[kind]}, without the device name)")
    return text


_EXAMPLES = {"analog input": "'ai0'", "analog output": "'ao0'", "digital line": "'port0/line0'"}


def normalize_ai(name) -> str:
    return _normalize(name, AI_PATTERN, "analog input")


def normalize_ao(name) -> str:
    return _normalize(name, AO_PATTERN, "analog output")


def normalize_line(name) -> str:
    return _normalize(name, LINE_PATTERN, "digital line")


class DaqOutputPolicy:
    """Which outputs a run may drive, and the allowed analog output span."""

    def __init__(self, ao_channels: Iterable = (), do_lines: Iterable = (),
                 ao_min_v: float = 0.0, ao_max_v: float = 5.0):
        self.ao_channels = frozenset(normalize_ao(c) for c in (ao_channels or ()))
        self.do_lines = frozenset(normalize_line(c) for c in (do_lines or ()))
        self.ao_min_v = float(ao_min_v)
        self.ao_max_v = float(ao_max_v)
        if not (DEVICE_AO_MIN_V <= self.ao_min_v <= self.ao_max_v <= DEVICE_AO_MAX_V):
            raise DaqPolicyError(
                f"daq_ao_min_v / daq_ao_max_v must satisfy {DEVICE_AO_MIN_V:g} <= min <= max <= "
                f"{DEVICE_AO_MAX_V:g} V, got {self.ao_min_v:g} .. {self.ao_max_v:g}")

    @classmethod
    def from_config(cls, config: dict) -> "DaqOutputPolicy":
        config = config or {}
        for key in ("daq_ao_min_v", "daq_ao_max_v"):
            if key in config and config[key] is None:
                # a key with no value is never read as "no limit"
                raise DaqPolicyError(f"shared_devices.{key} has no value (null); give a number")
        return cls(ao_channels=config.get("daq_ao_channels") or (),
                   do_lines=config.get("daq_do_lines") or (),
                   ao_min_v=config.get("daq_ao_min_v", 0.0),
                   ao_max_v=config.get("daq_ao_max_v", 5.0))

    def check_analog_output(self, channel, voltage) -> tuple:
        """Return (channel, voltage) normalised, or raise DaqPolicyError."""
        name = normalize_ao(channel)
        if name not in self.ao_channels:
            raise DaqPolicyError(
                f"analog output {name} is not enabled: list it under shared_devices.daq_ao_channels "
                f"in config.yaml after checking what is wired to it "
                f"(enabled: {sorted(self.ao_channels) or 'none'})")
        volts = float(voltage)
        if volts != volts or not (self.ao_min_v <= volts <= self.ao_max_v):
            raise DaqPolicyError(
                f"analog output {name} = {volts:g} V is outside the configured span "
                f"{self.ao_min_v:g} .. {self.ao_max_v:g} V (daq_ao_min_v / daq_ao_max_v)")
        return name, volts

    def check_digital_output(self, line) -> str:
        """Return the normalised line name, or raise DaqPolicyError."""
        name = normalize_line(line)
        if name not in self.do_lines:
            raise DaqPolicyError(
                f"digital output {name} is not enabled: list it under shared_devices.daq_do_lines "
                f"in config.yaml after checking what is wired to it "
                f"(enabled: {sorted(self.do_lines) or 'none'})")
        return name
