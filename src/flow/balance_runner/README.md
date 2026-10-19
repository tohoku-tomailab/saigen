# `src/flow/balance_runner` — centrifuge-tube balancing with the MaxArm

Both tubes already hold a sample. Fills tube A with a fixed volume, then fills
tube B with the same liquid until both tubes weigh the same within a tolerance (default
0.1 g). The Hiwonder MaxArm (robot 4, `type: maxarm`) carries a Picus 2
pipette in a 3D-printed adapter. Both 50 mL tubes stand open in a rack on the
balance for the whole run; the operator places them, the arm never lifts a
tube.

```bash
cp src/flow/balance_runner/balance.example.csv src/flow/balance_runner/balance.csv   # then edit
python -m src.flow.balance_runner.run_balance --validate-only   # sheet + workspace, opens nothing
python -m src.flow.balance_runner.run_balance --mock            # simulated arm, pipette, balance
python -m src.flow.balance_runner.run_balance --pipette-port COM9   # real run, from a terminal
```

A real run asks the operator three things at the terminal (empty rack, tube B,
tube A) and must be started from a terminal that can send **Ctrl+C**, which
is the emergency stop: the arm halts where it is.

## How it balances

The balance always reads `rack-tared total = A + B`. The runner weighs after
every dispense, with the tip lifted to `travel_z`, and books the change to the
tube it just dispensed into, so it knows the gross mass of each tube without
moving one.

1. Arm to `home_position`.
2. Tare with the empty rack; read tube B, then tube A (each with its sample).
   Before any liquid moves, the runner checks that B will need no more than
   `max_tube_volume_mL` and that B is not already heavier than A will be.
   The samples need not weigh the same: the gross masses are what count.
3. Tube A gets `volume_a_mL` (portions of at most `max_aspirate_mL`). The mass
   that arrived per commanded mL is the run's **effective density** (liquid
   density times the pipette's own bias). It must lie within
   `density_check_pct` of `liquid_density_g_per_mL`, otherwise the run stops
   before B gets anything.
4. Tube B gets the volume that leaves it `top_up_reserve_mL` short of A, then
   weighed top-ups, each converted with the effective density, until
   `|A - B| <= tolerance_g`.

The Picus 2 cannot dispense less than 0.5 mL, and liquid cannot be taken back
out. That is why the bulk stops short: the last top-up is then at least
0.5 mL. If a gap smaller than 0.5 mL remains, or B ends up heavier, the run
stops with `NOT BALANCED` and says by how much, so the operator can correct by
hand. With `allow_extra_in_a=1` the runner instead closes such gaps by adding
liquid to tube A too (A then holds more than `volume_a_mL`; the result says how
much).

## Results

- `results.csv` beside the sheet: balanced or not, gross masses of A and B,
  A − B, liquid put into each tube, effective density, and the parameters.
- `logs/<date>/tube_balancing_<time>/`: `run.log`, `measurements.csv`,
  `metadata.json`, `summary.md`, a copy of the sheet, and `balance_log.csv`
  with one row per dispense (commanded mL, grams that arrived, both masses
  and A − B after it).

Exit codes: 0 balanced, 1 not balanced or run failed, 2 bad sheet or missing
pipette port, 3 a target outside the MaxArm's workspace, 130 interrupted.

## Before the first real run

These depend on the cell and the adapter and cannot be checked from software.

- **Payload.** A Picus 2 with a 10 mL tip, the adapter and the USB cable are
  far heavier than a suction cup. Jog the arm with the pipette mounted
  (`python -m src.devices.maxarm.teach`) and watch the read-back: the
  client stops a move whose read-back is more than `move_tolerance_mm`
  (10 mm) off. Do not run with an arm that sags.
- **Height.** The tip must clear the tube rims (balance pan + rack + 115 mm
  tube) at `travel_z`, and `travel_z` must be reachable at every position
  (the MaxArm reaches only about 196–210 mm near the top of its workspace;
  `--validate-only` names the highest reachable z for a refused target). A
  riser under the arm or a short adapter may be needed.
- **Teach the positions** with `teach.py` (jog, `save`) and copy them into
  `balance.csv`: reservoir centre and aspiration height, the lowest safe
  aspiration height, tube A and tube B openings, dispense height (tip just
  inside the mouth, above the 45 mL mark), travel height. The values in
  `balance.example.csv` are placeholders.
- **Reservoir level.** Balancing two tubes uses up to ~45 mL. If the
  reservoir surface drops noticeably, set `reservoir_drop_mm_per_mL`
  (surface drop per mL taken) so the tip follows it; the run stops before
  going below `reservoir_min_z`.
- **Caps.** If the tubes are centrifuged capped, put each cap on the rack with
  its tube when prompted, so the caps are weighed with their tubes.
- **Vibration.** The arm stands next to the balance; `settle_s` (default 3 s)
  and `stabilization_count` (median of 5 readings) are there for that.
  Increase them if successive readings of a still pan differ.
- **Pipette port.** Give it with `--pipette-port`, the sheet's
  `pipette_port`, or `robots.4.picus2_address` in `config.yaml`.

The tube-volume, reservoir and density checks are guards, not guarantees:
liquid that misses a tube but lands on the rack is still weighed and would be
booked to that tube. Watch the first runs.
