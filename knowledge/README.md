# Knowledge base

What the SAIGEN developers learned while bringing instruments into the cell:
the order of work that succeeded, the replies a real unit gave where they
differed from the manual, the defects found on the bench or by review, the
points still open, and the things not to do. The procedures themselves are in
[`docs/`](../docs/); this folder keeps the experience behind them, so that the
next laboratory, and the AI coding agent working with it, does not have to
rediscover it.

Every note records what was observed on one unit, with its firmware and date.
Treat the numbers as expectations to re-check on your own unit, not as
specifications.

## Notes

| Note | Instrument | Connection | What it covers |
|---|---|---|---|
| [`maxarm-commissioning.md`](maxarm-commissioning.md) | Hiwonder MaxArm (suction arm) | USB serial | Order of work, numbers measured on the arm, why the reachable envelope is not the workspace box, recovery |
| [`mg400-commissioning.md`](mg400-commissioning.md) | Dobot MG400 (desktop arm) | TCP/IP | Observation-only scope, replies that differ from the vendor documents, order of work for the bench stages, what an interruption does |
| [`keithley2002-commissioning.md`](keithley2002-commissioning.md) | Keithley 2002 digital multimeter | GPIB | Order of work, what the firmware accepted, manual facts that shaped the driver, interruption and clean-up rules |
| [`li5640-commissioning.md`](li5640-commissioning.md) | NF LI5640 lock-in amplifier | GPIB | Read-only scope, what the firmware answered, defects found by review, the internal-oscillator bench test |
| [`dso1000-commissioning.md`](dso1000-commissioning.md) | Agilent DSO1014A oscilloscope | USB | Read-only scope, reply forms, the hang on return-to-local over USB, defects found on the bench and by review |

Rules for AI coding agents working in this repository are in
[`AGENTS.md`](../AGENTS.md) (mirrored in `CLAUDE.md`).

## Adding a note

Write one note per instrument, named `<instrument>-commissioning.md`, when the
instrument is first brought up (step 8 of
[`docs/adding-an-instrument.md`](../docs/adding-an-instrument.md)), and keep
adding to it as the bench teaches more. Use the sections the existing notes
share:

1. **Configuration checked**: model, firmware, interface, operating system, date.
2. **Scope chosen**: what the driver does and what it deliberately does not.
3. **Order of work that worked**: the steps in the order they succeeded.
4. **What the unit answered**: replies as observed, and where they differ from the manual.
5. **Defects found**: on the bench, and by review before the bench.
6. **Open points**: what is not yet verified.
7. **Things not to do**.

Record only what was observed, and say on which unit. Do not include serial
numbers, addresses, API keys or other laboratory-specific values. Never write a
measured value that is not backed by a record.

The notes are licensed under CC BY 4.0, like the documentation in `docs/`.
