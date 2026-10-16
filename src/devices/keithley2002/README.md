# Keithley 2002 digital multimeter (GPIB)

An 8.5-digit DMM. Its only interface is GPIB (IEEE-488); SCPI commands are sent with PyVISA
through a GPIB adapter on the PC and a VISA library (NI-488.2 / NI-VISA, Keysight IO
Libraries, etc.). Setup is in `docs/setup.md` §10.

## Wiring and settings

- Connect the PC's GPIB adapter with a GPIB cable. The address is under *MENU > GPIB* on the
  front panel (factory default 16)
- `config.yaml`:

  ```yaml
  shared_devices:
    dmm_resource: "GPIB0::16::INSTR"   # VISA resource string
    dmm_timeout_ms: 10000              # upper limit for one I/O
  ```

## Measurement functions

| `function` | SCPI | Unit | Maximum range |
|---|---|---|---|
| `dc_voltage` | `VOLTage:DC` | V | 1100 |
| `ac_voltage` | `VOLTage:AC` | V | 775 |
| `dc_current` | `CURRent:DC` | A | 2.1 |
| `ac_current` | `CURRent:AC` | A | 2.1 |
| `resistance_2w` | `RESistance` | Ohm | 1.05e9 |
| `resistance_4w` | `FRESistance` | Ohm | 2.1e6 |

NPLC is 0.01–50 for every function (AC included). The maximum ranges and the NPLC range
are from the user's manual (2002-900-01 Rev. E, Table 3-10).

## Command sequence (`configure()` → `read()`)

```
:ABOR
:CONF:<SCPI>                    known state for a single reading (three trigger layers IMM / count 1 / delay 0,
                                math, buffer and scanning disabled, the function's settings at their *RST values)
:SENS:<SCPI>:RANG:AUTO ON       or :SENS:<SCPI>:RANG <value>
:SENS:<SCPI>:NPLC <n>
:UNIT:VOLT:AC V                 (ac_voltage only; left in dB display the unit would differ)
:FORM:DATA ASC
:FORM:ELEM READ                 reply is the number only
:FORM:EXP HPR                   do not round to the range's engineering unit; limits loss of digits on output
:SYST:ERR?                      read until 0. Any error raises Keithley2002Error
:READ?                          = :ABOR + :INIT + :FETCh? (one conversion). samples times
:SENS:<SCPI>:RANG? / :NPLC?     after the last sample, read back and record the actual settings
                                (I/O errors are not swallowed: the replies would get out of step)
```

Of the instrument-wide settings that `:CONF` does not change, line synchronisation
(`:SYST:LSYN:STAT?`) is read at initialisation and recorded in run.log.

`*RST` is not sent. `:CONF` returns the acquisition method, autozero and the function's
settings to their defaults, but leaves everything outside that (GPIB address, display, line
synchronisation, etc.) unchanged. An overflow is returned as `+9.9E37`, which `read()` turns
into None. On exit, GTL (a GPIB bus command, `control_ren`) returns the instrument to
front-panel operation. The settings are not restored.

## Usage

```python
from src.devices.keithley2002 import Keithley2002

dmm = Keithley2002("GPIB0::16::INSTR")
try:
    dmm.clear_status()
    print(dmm.identify())
    dmm.configure("dc_voltage", nplc=10)
    print(dmm.read())
    print(dmm.read_settings("dc_voltage"))   # actual range and NPLC
finally:
    try:
        dmm.local()
    finally:
        dmm.close()                          # close even if GTL fails
```

Connection check (run by a human; it talks to the instrument but moves no robot):

```bash
python -m src.devices.keithley2002.keithley2002 GPIB0::16::INSTR
```

This prints a runpy `RuntimeWarning` (because the package `__init__` imports the module
first); it is harmless.

From a flow, use the `measure_dmm` action (`SharedDevices.measure_dmm`). I/O runs on a
worker thread and is serialised with a lock. On an interrupt (Ctrl+C / Stop in the GUI) the
step ends immediately (it does not hold up the robot's emergency stop), and the worker
abandons the remaining readings once the one I/O in progress has finished. The handle is
always closed by whoever holds the lock; if cleanup() cannot wait long enough, the worker
sends GTL and disconnects as soon as it has finished communicating (it is never closed in
the middle of a transfer).

What commissioning on the instrument taught (order of work, the commands firmware A09
accepted, the unresolved observation about line synchronisation, what not to do) is in
[`docs/keithley2002-commissioning.md`](../../../docs/keithley2002-commissioning.md).
