"""Ask the NI-DAQmx devices of this PC what they have. Reads properties only.

    python -m src.devices.nidaq.probe          # every device
    python -m src.devices.nidaq.probe Dev2     # one device

No channel is opened, no task is created and nothing is driven: these are the
figures the driver itself reports for the hardware. For a model that has a profile
(``profiles.py``) the report is compared with it; for a new model it is the starting
point of its ``DeviceProfile``. What DAQmx lists is not the last word: the USB-6351
lists bridge channels and then refuses to create one, so a profile claims a feature
only after the bench check in the README.
"""

import sys
from typing import List

from .profiles import PROFILES, DaqProfileError, get_profile


def _get(fn, default=None):
    try:
        return fn()
    except Exception:
        return default


def _names(items) -> List[str]:
    return [str(x).split(".")[-1] for x in (items or [])]


def describe(dev) -> dict:
    """The properties of one nidaqmx device, as plain values."""
    name = dev.name
    lines = [l.name.split("/", 1)[1] for l in _get(lambda: list(dev.di_lines), [])]
    ports = {}
    for line in lines:
        port = line.split("/")[0]
        ports[port] = ports.get(port, 0) + 1
    terminals = [t.replace(f"/{name}/", "") for t in _get(lambda: list(dev.terminals), [])]
    ranges = _get(lambda: list(dev.ai_voltage_rngs), [])
    ao_ranges = _get(lambda: list(dev.ao_voltage_rngs), [])
    return {
        "name": name,
        "model": str(_get(lambda: dev.product_type, "?")),
        "simulated": bool(_get(lambda: dev.is_simulated, False)),
        "ai_channels": len(_get(lambda: list(dev.ai_physical_chans), [])),
        "v_ranges": sorted({abs(v) for v in ranges}),
        "ai_max_rate_single_hz": _get(lambda: float(dev.ai_max_single_chan_rate)),
        "ai_max_rate_aggregate_hz": _get(lambda: float(dev.ai_max_multi_chan_rate)),
        "ai_triggers": _names(_get(lambda: dev.ai_trig_usage)),
        "ai_measurements": _names(_get(lambda: dev.ai_meas_types)),
        "ao_channels": len(_get(lambda: list(dev.ao_physical_chans), [])),
        "ao_span_v": (min(ao_ranges), max(ao_ranges)) if ao_ranges else None,
        "ao_max_rate_hz": _get(lambda: float(dev.ao_max_rate)),
        "port_lines": tuple(ports[p] for p in sorted(ports)),
        "timed_ports": [p.name.split("/", 1)[1] for p in _get(lambda: list(dev.di_ports), [])
                        if _get(lambda p=p: p.di_samp_clk_supported, False)],
        "change_detection_ports": [
            p.name.split("/", 1)[1] for p in _get(lambda: list(dev.di_ports), [])
            if _get(lambda p=p: p.di_change_detect_supported, False)],
        "dio_max_rate_hz": _get(lambda: float(dev.do_max_rate)),
        "counters": len(_get(lambda: list(dev.ci_physical_chans), [])),
        "counter_measurements": _names(_get(lambda: dev.ci_meas_types)),
        "counter_outputs": [c.name.split("/", 1)[1]
                            for c in _get(lambda: list(dev.co_physical_chans), [])],
        "pfi": len([t for t in terminals if t.upper().startswith("PFI")]),
        "terminals": terminals,
    }


def compare(info: dict) -> List[str]:
    """Where the device's own report and the profile of its model disagree."""
    try:
        p = get_profile(info["model"])
    except DaqProfileError:
        return [f"no profile for {info['model']} (supported: {', '.join(sorted(PROFILES))}); "
                "the figures above are the start of one"]
    notes = []

    def check(label, reported, expected, ok=None):
        if reported is None:
            return
        if not (ok(reported, expected) if ok else reported == expected):
            notes.append(f"{label}: device reports {reported}, profile says {expected}")

    check("analog inputs", info["ai_channels"], p.ai_channels)
    check("input ranges", tuple(info["v_ranges"]), tuple(p.v_ranges))
    check("AI rate, one channel", info["ai_max_rate_single_hz"], p.ai_max_rate_single_hz,
          lambda r, e: e <= r)
    check("AI rate, aggregate", info["ai_max_rate_aggregate_hz"], p.ai_max_rate_aggregate_hz,
          lambda r, e: e <= r)
    check("analog outputs", info["ao_channels"], p.ao_channels)
    check("AO rate", info["ao_max_rate_hz"], p.ao_max_rate_hz, lambda r, e: e <= r)
    check("digital lines per port", info["port_lines"], p.port_lines)
    check("counters", info["counters"], p.counters)
    check("PFI terminals", info["pfi"], p.pfi)
    check("hardware-timed port", (info["timed_ports"] or [None])[0], p.timed_port)
    # Not compared: the output span, what the counters can measure or output, triggers.
    return notes or [f"the compared figures agree with the {p.model} profile "
                     "(channel counts, ranges, rates, ports, counters, PFI)"]


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    try:
        import nidaqmx.system
        devices = list(nidaqmx.system.System.local().devices)
    except Exception as exc:  # driver not installed, service stopped, ...
        print(f"NI-DAQmx is not available: {exc}")
        return 1
    if argv:
        devices = [d for d in devices if d.name in argv]
    if not devices:
        print("no device found")
        return 1
    for dev in devices:
        info = describe(dev)
        print(f"== {info['name']}: {info['model']}" + (" (simulated)" if info["simulated"] else ""))
        for key, value in info.items():
            if key not in ("name", "model", "simulated"):
                print(f"  {key}: {value}")
        for note in compare(info):
            print(f"  -> {note}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
