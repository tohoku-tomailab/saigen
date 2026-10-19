# Agilent 1000 series digital oscilloscope (USB, read-only)

DSO1002A / 1004A / 1012A / 1014A / 1022A / 1024A (2 or 4 channels). SAIGEN talks to it
through the rear USB DEVICE port (USBTMC) with PyVISA over a VISA library (NI-VISA etc.).
Setup is in `docs/setup.md` §12. The commands were checked against the "Agilent 1000 Series
Oscilloscopes Programmer's Guide" (edition 011.04.00.04.06, April 2010); "p.n" below are its
pages. What commissioning the first unit taught is in `docs/dso1000-commissioning.md`.

## Policy: set-up on the front panel, SAIGEN only reads automatic measurements

Vertical (V/div, offset, probe ratio, coupling, bandwidth limit), horizontal (s/div),
trigger and acquisition (normal / average / peak) are set on the front panel. SAIGEN sends
queries only. It never sends a command that changes a setting (`*RST`, `:AUToscale`,
`:RUN` / `:STOP` / `:SINGLE`, or the setting form of `:CHANnel` / `:TIMebase` /
`:TRIGger` / `:ACQuire` / `:MEASure`).

Side effects:

- Device clear on connect and on disconnect (clears the I/O buffers)
- `*CLS` on connect (clears the status and the error queue, p.60; errors left there by
  front-panel use are lost too)
- No GTL. Over USB with NI-VISA, `control_ren` blocked for 120 s on a DSO1014A (firmware
  00.04.04) and then failed with `VI_ERROR_SYSTEM_ERROR`. Between query sessions the front
  panel stayed usable (`:KEY:LOCK?` = `ENABLE` means "front panel enabled", p.135). Once,
  after the session in which GTL hung, the front panel was found locked; the cause is not
  known (it was released at the instrument; whether with the Force key or by switching off
  and on was not recorded)

## Wiring and settings

- Connect the rear USB DEVICE port (square connector) to the PC (p.21)
- The resource string is `USB0::0x0957::<product ID>::<serial>::INSTR`. It contains the
  serial number, so it goes in `config.yaml` only:

  ```yaml
  shared_devices:
    scope_resource: "USB0::0x0957::<product ID>::<serial>::INSTR"
    scope_timeout_ms: 5000
    wait_after_scope: 0.0      # wait after each measure_scope
  ```

## Command sequence

```
connect:     device clear, *CLS, *IDN? (model must be in the table; remembers the channel count),
             :SYSTem:ERRor? (until 0, at most 31 reads)
each step:   :TIMebase:FORMat? (X-Y fails; replies Y-T / X-Y / ROLL, p.214)
             settings snapshot (queries only)
             :MEASure:<item>? CHANnel<n> for each item, `samples` times (`interval` s apart)
             settings snapshot again, :SYSTem:ERRor? (any entry fails the step)
disconnect:  device clear, then close (close even if the clear fails; no GTL)
```

Snapshot: time base FORMat / MODE / SCALe / OFFSet; the channel's DISPlay / SCALe / OFFSet /
PROBe / COUPling / BWLimit / FILTer / INVert / VERNier / UNITs; ACQuire TYPE / AVERages;
TRIGger MODE / COUPling / HOLDoff / SENSitivity / HFREject and (for EDGE) EDGE SOURce /
LEVel / SLOPe / SWEep; `:MEASure?` (front-panel measurement lock); `:KEY:LOCK?` (the guide
gives `ENAB` / `DIS`, the instrument returned `ENABLE`; both are accepted and recorded as
`ENAB` / `DIS`). Values are checked against the guide's ranges (time base 1 ns to 50 s/div,
hold-off 100 ns to 1.5 s, trigger sensitivity 0.1 to 1 div). `:MEASure:SOURce?`,
`:TRIGger:STATus?` and `:ACQuire:SRATe?` are recorded but not compared before and after.

A step fails on: an X-Y or DELAYED (zoom) time base (the measurement window cannot be
derived from the settings, p.217); a trigger other than edge (its settings are not read, so
a change could not be detected); a hidden channel; a channel the model lacks; settings that
differ before and after (a change made and undone within the step is not detected); any
requested item without a single valid reading; an I/O error; an unreadable setting; an entry
in the error queue after the reads. A failed step keeps the settings, replies and error-queue
entries read so far as diagnostics, without further I/O.

## Items and units

| Item | Header | Unit |
|---|---|---|
| `vpp` `vmax` `vmin` `vamplitude` `vtop` `vbase` `vaverage` `vrms` | VPP etc. | the channel's UNITs (V / A / W; empty for UNKNown) |
| `frequency` | FREQuency | Hz |
| `period` `risetime` `falltime` `pwidth` `nwidth` | PERiod etc. | s |
| `pdutycycle` `ndutycycle` `overshoot` `preshoot` | PDUTycycle etc. | % (the guide shows a trailing `%`, p.168, 172, 174, 178; firmware 00.04.04 sent none) |

Discarded replies, counted per item and per reason: bound values starting with `<` or `>`
(rise time etc., p.165, 181), unparsable text, NaN / Inf, and a magnitude of 9.9e37 or more
(the guide documents 9.9E+37 only for the counter, p.103; the instrument returned
`9.9E+37` from `:MEASure` for a hidden channel and for a trace without edges). For the four
% items one trailing `%` is removed if present; the value is not multiplied by 100.

`samples` is the number of reads, not of acquisitions: the oscilloscope does not acquire
while it answers (p.27). The items of one read are not simultaneous either.

## Checked on the instrument (DSO1014A, firmware 00.04.04, 2026-10-16 and 2026-10-19)

- A real signal on CH1 (0.2 V/div, 2 ms/div, status `T'D`): all 18 items returned plain
  numbers such as `8.880e-001`, `3.048e002`, `5.000e001` (duty cycle, **no `%`**),
  `0.000e000` and `9.090e-003` (overshoot / preshoot). No `<` / `>` reply was seen. The
  example flow ran end to end with `--no-record`
- `:MEASure:<item>? CHANnel2` left `:MEASure:SOURce?` at `CHANnel1` and added no
  measurement to the screen (`:MEASure:TOTal?` = 0)
- A hidden channel, or a stopped trace without edges, gives `9.9E+37`
- `:KEY:LOCK?` returned `ENABLE`; with that reply V/div changed at the front panel was read
  by the next session
- GTL hung for 120 s and failed (above)
- The self-check (connect, snapshot, three items, disconnect) took about 0.5 s with
  acquisition stopped

## Not yet checked

- `<` / `>` bound replies, and whether overshoot / preshoot are in percent (the guide says
  so; `9.090e-003` was read on a signal whose overshoot was not known)
- The front-panel lock after the GTL hang: its cause and how to release it
- Whether queries are answered while the front-panel measurement is locked
  (`:MEASure?` = Locked)
- Whether `:TIMebase:OFFSet?` answers in ROLL mode

## Self-check (run by a human; queries only, apart from the device clears and `*CLS`)

```bash
python -m src.devices.dso1000.dso1000 USB0::0x0957::<product ID>::<serial>::INSTR [channel]
```
