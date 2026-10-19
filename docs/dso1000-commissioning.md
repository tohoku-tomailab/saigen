# Commissioning an Agilent DSO1014A over USB: what the bench taught us

Field notes from bringing an Agilent DSO1014A oscilloscope into the SAIGEN cell
(October 2026). They complement the reference material in
[`src/devices/dso1000/README.md`](../src/devices/dso1000/README.md) (commands,
items, units, what is discarded) and the procedure in
[`setup.md` §12](setup.md#12-agilent-1000-series-oscilloscope-optional-read-only).
The general rules for new instruments are in
[`adding-an-instrument.md`](adding-an-instrument.md).

Configuration checked: one DSO1014A (firmware 00.04.04) on the rear USB DEVICE
port, Windows 11, NI-VISA, PyVISA 1.16, on 2026-10-16 and 2026-10-19. A signal
of about 300 Hz and 0.9 Vpp reached CH1 only at the end of the second day; it
was not a known reference, so the readings in §3 check communication and reply
forms, not accuracy. The `<` / `>` bound replies are covered by the tests
against a simulated instrument only. Another unit or firmware may differ, so treat what follows as
expectations to re-check, not as specifications.

## 1. Scope chosen: read-only, automatic measurements

The 1000 series can be fully programmed (vertical, time base, trigger,
acquisition, waveform transfer). It was decided that the human sets the
instrument up on the front panel and SAIGEN only reads the automatic
measurements (`:MEASure:<item>? CHANnel<n>`) together with the settings that
give them their meaning. SAIGEN sends no command that changes a setting; its
only side effects are a device clear when it opens and closes the instrument
and `*CLS` when it opens it (which also clears errors that front-panel use left
in the queue). Waveform transfer was left out: it would need `:WAVeform`
settings to be written.

States whose meaning the code cannot check are refused rather than recorded:
an X-Y time base, the DELAYED (zoom) time base, any trigger other than edge
(its settings are not read, so a change could not be detected), a hidden
channel, and a channel the model does not have.

## 2. Order of work that worked

The steps that talk to the instrument (2 to 4) are the human's: run them
yourself, or have an agent run them only after you ask for it in the session
(CLAUDE.md rule 1).

1. **Read the programmer's guide first** ("Agilent 1000 Series Oscilloscopes
   Programmer's Guide", April 2010). Its text can be extracted. Page numbers in
   the driver comments refer to it.
2. **List USB resources only**
   (`pyvisa.ResourceManager().list_resources("USB?*::INSTR")`). The resource
   string contains the serial number; it belongs in `config.yaml`
   (`scope_resource`), never in tracked files.
3. **The driver's self-check**
   (`python -m src.devices.dso1000.dso1000 <resource> [channel]`): device
   clear, `*CLS`, `*IDN?`, the error queue, a settings snapshot, three items,
   device clear and close. Run it with channels shown and hidden, and on
   another channel than 1.
4. **`--validate-only`, `--mock`, then a scope-only real run** of
   `examples/dso1000/read_scope.json` with `--no-record`.

What was verified: communication, every query the driver sends (about 30 per
snapshot), all 18 items on a signal, the reply forms below, the refusal of a
hidden channel before any measurement query, a scope-only flow run end to end
(two steps, three reads each), and a clean close. What was **not** verified:
the accuracy of the readings (no known reference was used).

## 3. What firmware 00.04.04 answered

| Query | Guide | Instrument |
|---|---|---|
| `*IDN?` | | `Agilent Technologies,DSO1014A,<serial>,00.04.04` |
| `:TIMebase:FORMat?` | `Y-T`, `X-Y`, `ROLL` (p.214) | `Y-T` |
| `:TIMebase:SCALe?`, levels | | `1.000e-06`, `3.00e-01`, `-6.00e-03` (digits vary by query) |
| `:CHANnel<n>:UNITs?` | | `VOLTs` |
| `:CHANnel<n>:PROBe?` | | `1X` |
| `:MEASure?` | `UnLocked` / `Locked` | `UnLocked` |
| `:KEY:LOCK?` | `ENAB` / `DIS` (p.135) | `ENABLE` |
| `:MEASure:SOURce?` | | `CHANnel1` |
| `:MEASure:<item>? CHANnel<n>` on a hidden channel, or with no edges in a stopped trace | 9.9E+37 is documented only for the counter (p.103) | `9.9E+37` |
| `:MEASure:VPP? CHANnel1` on a flat, stopped trace at 0.5 V/div | | `4.000e-002` |
| `:TRIGger:STATus?` while triggering | `RUN`, `STOP`, `T'D`, `WAIT`, `SCAN`, `AUTO` | `T'D` |
| `:MEASure:<item>? CHANnel1` on a signal (0.2 V/div, 2 ms/div) | | `8.880e-001` (Vpp), `3.048e002` (frequency), `1.080e-003` (rise time) |
| `:MEASure:PDUTycycle?`, `NDUTycycle?` | trailing `%` (p.168, 172) | `5.000e001`, `5.121e001`: **no `%`** |
| `:MEASure:OVERshoot?`, `PREShoot?` | trailing `%` (p.174, 178) | `0.000e000`, `9.090e-003`: no `%` |

- `:KEY:LOCK?` = `ENABLE` means the front panel is **enabled**: with that reply
  the human changed V/div between two sessions and the next session read the
  new value. The driver accepts both spellings and records `ENAB` / `DIS`.
- A query with a source (`:MEASure:VPP? CHANnel2`) did **not** change
  `:MEASure:SOURce?` and added no measurement to the screen
  (`:MEASure:TOTal?` stayed 0).
- The self-check (open, snapshot, three items, close) took about 0.5 s with
  acquisition stopped and no waveform. With a signal, each step of the example
  flow (snapshot, three reads 0.5 s apart, snapshot, error queue) took about
  1.3 s.
- The driver strips a trailing `%` only if one is there, so both the guide's
  form and the instrument's form are read.
- No `<` / `>` reply appeared, also not for rise and fall time.

## 4. Defects found on the bench

- **GTL hangs over USB.** The first driver sent GTL (`control_ren`, go to local)
  before closing, as the GPIB drivers do. Over USB with NI-VISA it blocked for
  120 s and then failed with `VI_ERROR_SYSTEM_ERROR`. GTL was removed; closing
  is now device clear, then close. A test fails if REN / GTL is ever used.
- **`ENABLE` instead of `ENAB`.** The first parser accepted only the guide's
  spelling and would have failed every step.
- **The front panel was once found locked** after the session in which GTL
  hung. `:KEY:LOCK DISable` (three spellings) did not change the reply; the
  human released the panel at the instrument (whether with the Force key or by
  switching off and on was not recorded). It did not happen again in later
  sessions without GTL. The cause is not known.

## 5. Defects found by review before the bench

Each was found by review (Astra, through Codex) of the plan or of the code, and
each now has a regression test in `tests/test_dso1000.py` that fails when the
fix is reverted.

- **Settings must be checked against the guide's ranges, not only for being
  numbers.** A hold-off of -1 s, a trigger sensitivity of 99 div or 100 s/div
  were accepted; a units reply such as `VOLTS_GARBAGE` was read as volts by
  prefix matching. Ranges now follow the guide and units are an exact table.
- **Excluded from the comparison is not excluded from the record.** Status,
  sample rate and `:MEASure:SOURce?` may change during a step and are not
  compared, but the snapshot after the reads is now kept on success too.
- **Diagnostics of a failed step.** An I/O error or an instrument error lost
  what had been read. The step now keeps the settings, the replies and the
  error-queue entries read so far, also when the failure happens in the middle
  of a snapshot or of the error queue, without any further I/O.
- **Tests that passed on broken code.** The simulated instrument answered
  unknown measurement headers, so a misspelled header went unnoticed; it now
  refuses them. The `wait_after_scope` test only checked that a wait happened,
  not that it came after the last query. The test of a Ctrl+C during the last
  query let `cleanup()` hide a missing stop; it now checks that the worker
  stops on its own before `cleanup()` runs.
- **"Queries only" was not exact**: the docs now name the device clears and
  `*CLS`.

## 6. Open points

- **A known reference.** Accuracy was not checked. Feed a signal of known
  amplitude and frequency into CH1 and compare.
- **Bound replies** (`<` / `>`, rise time near the limit of the time base)
  and the unit of overshoot / preshoot (the guide says %; `9.090e-003` was read
  on a signal whose overshoot was not known).
- **The front-panel lock** described in §4: its cause and the way to release
  it.
- **Not yet seen on the instrument:** a front-panel measurement lock
  (`:MEASure?` = `Locked`), the ROLL time base (whether `:TIMebase:OFFSet?`
  answers), a current probe (`UNITs` other than volts), a 2-channel model.

## 7. Things not to do

- Do not send GTL over USB (§4).
- Do not send `*RST`, `:AUToscale`, `:RUN`, `:STOP` or `:SINGLE`, or the
  setting form of any `:CHANnel`, `:TIMebase`, `:TRIGger`, `:ACQuire` or
  `:MEASure` command: the set-up belongs to the human at the front panel.
- Do not read `samples` as a number of acquisitions: the oscilloscope does not
  acquire while it answers, and in STOP every read returns the same trace (the
  step logs a warning).
- Do not judge readings of a channel without its settings: V/div, probe ratio
  and units are recorded next to every step for that reason.
- Do not take `9.9E+37` for a value; it means "no measurement" and is counted
  as `no_measurement`.

## 8. Small things

- `python -m src.devices.dso1000.dso1000` prints a runpy `RuntimeWarning` (the
  package `__init__` imports the module first). It is harmless.
- A step that reads several items does not read them at the same instant;
  they are consecutive queries.
