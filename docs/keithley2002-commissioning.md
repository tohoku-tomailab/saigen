# Commissioning a Keithley 2002 over GPIB: what the bench taught us

Field notes from bringing a Keithley 2002 digital multimeter into the SAIGEN
cell (October 2026). They complement the reference material in
[`src/devices/keithley2002/README.md`](../src/devices/keithley2002/README.md)
(command sequence, driver) and the procedure in
[`setup.md` §10](setup.md#10-keithley-2002-digital-multimeter-optional).
The general implementation rules that came out of this work are in
[`adding-an-instrument.md`](adding-an-instrument.md) (§2 and §6).

Configuration checked: one MODEL 2002 (firmware A09 /A02), one NI GPIB-USB-HS+
adapter, Windows 11, NI-488.2 with NI-VISA, PyVISA 1.16, on 2026-10-15. Only
**DC voltage** was exercised on the instrument; the other five functions are
covered by the tests against simulated instruments only. Another unit, adapter
or PC may differ in details, so treat what follows as expectations to re-check,
not as specifications.

The intended first use is reading a DC voltage (for example the output of a
power supply), and that is the path these notes cover.

## 1. Order of work that worked

The steps that talk to the instrument (2 to 5) are the human's: run them
yourself, or have an agent run them only after you ask for it in the session
(CLAUDE.md rule 1). A DMM-only flow moves nothing, but that does not by itself
authorise an agent to open the hardware.

1. **Install the adapter's driver first.** On the PC we used, before NI-488.2
   was installed, the adapter enumerated (USB VID 3923) but Device Manager
   listed "GPIB-USB-HS+" and its "Analyzer" interface with status *Error*, no
   VISA library was present, and PyVISA had no backend to load. Installing
   NI-488.2 with NI-VISA (administrator rights, a reboot) fixed that: class
   "GPIB", status OK. Other vendors and the PyVISA-py route behave
   differently; see `setup.md` §10.
2. **List GPIB resources only:**
   `pyvisa.ResourceManager().list_resources("GPIB?*::INSTR")`. The default query
   also returns serial (ASRL) resources. Listing is not opening, but on this
   cell some serial devices react when they *are* opened (the MaxArm's ESP32
   resets, the UM22 microscope resets on RTS; see
   [`maxarm-commissioning.md`](maxarm-commissioning.md) and
   [`src/devices/microscope/README.md`](../src/devices/microscope/README.md)),
   so keeping serial resources out of the list removes any temptation to
   probe them.
3. **Read the address from the list, do not assume it.** Our unit was not at
   the factory address 16. The address belongs in `config.yaml`
   (`dmm_resource`), never in tracked files.
4. **`*IDN?`, then the driver's self-check**
   (`python -m src.devices.keithley2002.keithley2002 <resource>`: `*IDN?`, line
   sync, one DC-voltage reading, read-back of range and NPLC, back to front
   panel).
5. **`--validate-only`, `--mock`, then a DMM-only real run** of
   `examples/keithley2002/measure_dc_voltage.json` with `--no-record`.
   `--no-record` switches off only the sensor CSV and video recording; the run
   folder (`run.log`, `measurements.csv`, `summary.md`, `metadata.json`) is
   still written. The flow opened no robot (`"robots_used": []` in
   `metadata.json`).

What was verified: communication, the command sequence, and the flow and
logging path. What was **not** verified: accuracy. The input terminals were
not connected to a known reference, so the values read in that run are
observations from a communication check; they cannot be used to judge the
true input or the measurement accuracy, and they are not listed here.

## 2. What firmware A09 accepted

On the DC-voltage path, every command the driver sends was accepted: the
error queue read `0,"No error"` at each point where the driver checks it
(after connecting and after configuring, not after every query). That
includes the two commands we were least sure of, `:FORM:EXP HPR` and
`:SYST:LSYN:STAT?`.

Step durations observed in that run (`duration_s` in `measurements.csv`, which
covers configure, the readings and the read-back; `wait_after_dmm` was 0;
line sync read ON at initialisation; mains frequency not recorded):

| Step | Samples | NPLC | Range | Duration |
|---|---|---|---|---|
| 1 | 3 | 1 | auto (settled on 0.2 V) | 0.44 s |
| 2 | 5 | 10 | fixed 20 V | 8.3 s |
| 3 | 3 | 1 | auto (settled on 0.2 V) | 0.88 s |

Step 2 took longer than the integration time alone suggests (5 × 10 power-line
cycles is about 1 s); the reason was not investigated. Raise `dmm_timeout_ms`
before using high NPLC values.

## 3. Manual facts that shaped the driver

From the [Model 2002 User's Manual, 2002-900-01 Rev. E](https://download.tek.com/manual/2002-900-01%28E-Feb2009%29%28User%29.pdf). The full
command sequence is in the driver README; this section records why.

- **`:CONFigure:<function>` instead of `*RST`** (p. 3-42). It selects the
  function and puts the instrument into a known single-shot state: all three
  trigger-model layers on Immediate with count 1 and delay 0, continuous
  initiation off, math and the buffer disabled, scanning disabled, acquisition
  method normal, autozero controls and the function's own controls (filter,
  REL, digits, ...) at their `*RST` values (the filter is off after `*RST`,
  Table 3-10 notes 4 and 5). It is therefore **not** "hands off" the
  instrument; what it leaves alone are the settings outside that list, such as
  the GPIB address, the display and line synchronization.
- **NPLC exists for the AC functions too** (Table 3-10: 0.01 to 50 for ACV and
  ACI as for the DC functions). The first draft of the plan assumed otherwise.
- **ACV units can be dB or dBm** (`:UNIT:VOLTage:AC`), so the driver forces V.
- **`:FORMat:EXPonent NORMal` formats the reading in the engineering units of
  the present range; `HPRecision` returns more digits** (p. 3-83). This reduces
  digits lost in the transfer of the reading; it says nothing about
  calibration accuracy.
- **Overflow reads `+9.9E37`** (p. 3-81); the driver treats it as no reading.
  A step whose samples are all invalid fails.
- **GTL is a GPIB bus command** (`control_ren`), not an SCPI string. It returns
  the front panel to the user; it does not restore any setting.
- **Line synchronization** (`:SYSTem:LSYNc:STATe`, p. 3-143) is instrument-wide
  and not in the `:CONFigure` list; its default is OFF (Table 3-13, p. 3-62;
  Table 2-42, p. 2-93).

What `measurements.csv` records for a `measure_dmm` step: the median of the
valid samples (overflow and unparsable replies excluded), full precision; the
unit and function; the range and NPLC read back **after the last sample** (on
autorange, the range the last sample used); and the number of valid samples.

## 4. Open observation: line synchronization read differently in two sessions

On 2026-10-15 the self-check read line sync **OFF**; the next session (the
flow run, a few minutes later) read **ON** at initialisation, and two later
read-only queries read ON. Neither session sends a command that sets it, and
no front-panel use is known. Not explained.

What this means for the records: the line-sync state in `run.log` is a
snapshot taken when the DMM is opened; it does not prove the state during each
measurement. Recording it with every measurement would make the record
stronger (planned, not done); it would still not explain the change.

## 5. Interruption and clean-up: rules that came from review

Each item below was a real defect found by review during this work and is now
covered by a regression test in `tests/test_keithley2002.py`. They apply to any
instrument whose I/O blocks; the general form is in
[`adding-an-instrument.md`](adding-an-instrument.md) §2.

- **Ctrl+C stops the robots first.** If the DMM is still initialising (a GPIB
  reply pending), `ExperimentSession.add_shared()` does not wait for it: it
  registers the device and re-raises, `run()` emergency-stops the robots, and
  `cleanup()` closes the DMM afterwards.
- **An interruption during clean-up stays an interruption.** Before the fix, a
  Ctrl+C while a half-initialised device was being closed turned into an
  ordinary error, and an ordinary error sends the Dobot arms home (MaxArm arms
  are never moved automatically after an error).
- **A VISA handle is never closed under an I/O in flight.** Only the holder of
  the lock closes it. If `cleanup()` gives up waiting (`dmm_close_wait_s`), the
  worker tries to close the handle once its current I/O has ended. That relies
  on the I/O ending: each VISA read and write has the `dmm_timeout_ms` timeout,
  but a query is a write and a read, so this is not a bound on the whole
  operation.
- **The abort request is checked before every SCPI write and query**, not only
  before each reading: a cancel during configuration or during the last reading
  sends nothing more. Device clear, GTL and close are not gated by it.
  Cancelling sets a flag; the worker thread is not killed.
- **A failed close is logged and the handle kept**, so the next `cleanup()` can
  retry. Nothing retries automatically.
- **I/O errors on read-back queries are not swallowed.** A timed-out query can
  leave its reply on the bus, and the next query could then receive it (a
  range taken for a measurement, for instance); the step fails instead.
- **A failed initialisation closes what it opened.** Before the fix, a balance
  opened before a failing DMM stayed open.

## 6. Things not to do

- Do not send `*RST` from a flow step; `:CONFigure` sets what a measurement
  needs and leaves the rest of the instrument alone.
- Do not list or probe all VISA resources on this cell; list GPIB only.
- Do not assume address 16.
- Do not use the generic `wait` action in a DMM-only flow: its `robot_id`
  defaults to 1, so a real run would open robot 1. Use `wait_after_dmm` in
  `config.yaml` for a pause after each DMM step.
- Do not measure current or resistance on an unknown connection: the
  resistance functions source a test current, and the current ranges go
  through the instrument's fuse.
- Do not use readings taken on an unknown input to judge the input or the
  instrument's accuracy; check against a known reference first.

## 7. Small things

- `python -m src.devices.keithley2002.keithley2002` prints a runpy
  `RuntimeWarning` (the package `__init__` imports the module first). It is
  harmless.
- In the GUI, changing a `measure_dmm` step's function drops its fixed range
  (a 20 V range must not become 20 A) and returns the step to autorange.
