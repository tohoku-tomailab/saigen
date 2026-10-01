# main.py -- device launcher of the SAIGEN MaxArm command server (MicroPython, ESP32).
#
# Installed on the MaxArm by src/devices/maxarm/deploy_firmware.py (a human-run
# tool) next to saigen_maxarm.py. It replaces Hiwonder's factory main.py, which
# ALWAYS moves the arm to its home pose at boot; a backup of the factory program
# is taken first and can be restored (see ../README.md).
#
# Boot behaviour: this launcher never moves the arm and never changes servo torque.
# It builds the Hiwonder driver objects (no motion), switches the pump and valve
# PWM outputs off (MicroPython's PWM(Pin) starts at duty 512), prints
#     EVT READY fw=saigen-maxarm proto=1 ver=<semver>*XX
# once and serves the line protocol on the USB REPL port (115200 8N1).
#
# If a pump/valve PWM output is missing or cannot be set to 0, start-up fails:
# no READY, "EVT FAULT ..." instead (and a best-effort zeroing of the outputs
# that exist). The nozzle object is allocated with SuctionNozzle.__new__ and
# stored before its __init__ runs, so outputs created by a constructor that then
# raises are still reachable by that zeroing (bench check: README "Bench checks").
#
# If anything raises -- including Ctrl-C (KeyboardInterrupt) from the host -- the
# servos get SERVO_MOVE_STOP (best effort), "EVT FAULT ..." is printed and the
# exception is re-raised so the MicroPython REPL becomes reachable (that is how
# deploy_firmware.py gets in). Ctrl-C is never masked. After READY the fault path
# never touches the pump (an object held by suction is not dropped).
#
# Not importable on CPython without the Hiwonder libraries; the tests run it with
# stub modules (tests/test_maxarm_firmware.py).

import sys

try:
    import utime as time
except ImportError:
    import time

try:
    import uselect as select
except ImportError:
    import select

from saigen_maxarm import Server, frame, token_safe, FIRMWARE_VERSION

READ_BURST = 64      # bytes handled per loop pass before the watchdog tick
POLL_MS = 20


def _write(text):
    sys.stdout.write(text)


PWM_OUTPUTS = ("pump_f", "pump_b", "valve_f", "valve_b")


def _zero_pwm(nozzle):
    """Set every pump/valve PWM output to duty 0; raise if one is missing or fails."""
    bad = []
    for name in PWM_OUTPUTS:
        pwm = getattr(nozzle, name, None)
        if pwm is None:
            bad.append(name + ":missing")
            continue
        try:
            pwm.duty(0)
        except Exception as e:
            bad.append("%s:%s" % (name, type(e).__name__))
    if bad:
        raise RuntimeError("pump/valve PWM not zeroed: " + ",".join(bad))


def _zero_pwm_best_effort(nozzle):
    """Zero whatever outputs exist (a half-constructed nozzle may lack some)."""
    for name in PWM_OUTPUTS:
        try:
            pwm = getattr(nozzle, name, None)
            if pwm is not None:
                pwm.duty(0)
        except Exception:
            pass


def _build(hw):
    """Construct the Hiwonder objects into ``hw`` one by one (kept for cleanup)."""
    from BusServo import BusServo
    from espmax import ESPMax
    from SuctionNozzle import SuctionNozzle
    hw["bus"] = BusServo()
    hw["arm"] = ESPMax(hw["bus"])
    # allocate first and keep the reference, then initialise: if __init__ raises
    # after creating some PWM outputs (duty 512), cleanup can still zero them.
    # Older MicroPython builds have no Class.__new__ attribute; object.__new__
    # is the documented form there (deploy_firmware --check reports which works).
    try:
        noz = SuctionNozzle.__new__(SuctionNozzle)
    except AttributeError:
        noz = object.__new__(SuctionNozzle)
    hw["nozzle"] = noz
    SuctionNozzle.__init__(noz)
    _zero_pwm(noz)


def _serve(server):
    rd = getattr(sys.stdin, "buffer", sys.stdin)
    poller = select.poll()
    poller.register(sys.stdin, select.POLLIN)
    while True:
        n = 0
        while n < READ_BURST and poller.poll(0 if n else POLL_MS):
            c = rd.read(1)
            if c:
                server.feed(c if isinstance(c, int) else (ord(c) if isinstance(c, str) else c[0]))
            n += 1
        server.tick()


def run():
    server = None
    hw = {}
    ready = False
    try:
        _build(hw)
        server = Server(hw["arm"], hw["bus"], hw["nozzle"], _write, time.ticks_ms,
                        time.ticks_diff, version=FIRMWARE_VERSION, sys_version=sys.version,
                        ticks_add=time.ticks_add)
        server.announce()
        ready = True
        _serve(server)
    except BaseException as e:           # includes KeyboardInterrupt: report, then re-raise
        try:
            if server is not None:
                server.safe_stop()
            elif hw.get("bus") is not None:
                for i in (1, 2, 3):
                    try:
                        hw["bus"].stop(i)
                    except Exception:
                        pass
        except Exception:
            pass
        if not ready and hw.get("nozzle") is not None:
            _zero_pwm_best_effort(hw["nozzle"])     # start-up failed: outputs may be at 512
        try:
            _write(frame("EVT FAULT type=%s msg=%s"
                         % (token_safe(type(e).__name__, 40), token_safe(e, 100))))
        except Exception:
            pass
        raise


run()
