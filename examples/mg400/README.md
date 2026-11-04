# Dobot MG400 example (robot 5, `type: mg400`)

- `z_hop.json`: robot 5 goes to its configured `home_position`, lifts the flange by 5 mm,
  waits one second, lowers it by 5 mm and returns home (which then sends nothing: the arm
  is already within 1 mm of home). Every move is one straight `MovL` inside the robot's own
  workspace box. It is the flow of bench stage C2 (`docs/mg400-commissioning.md` §2).
- `config.mg400.yaml`: a complete `config.yaml` with the MG400 as robot 5. Copy it to the
  repository root as `config.yaml` (or merge its `robots.5` entry into yours) and edit it.
  Every key of the entry is required and nothing is inherited; the numbers are **not**
  measurements of a real cell (the box is a small volume around the fake controller's
  start pose), so measure your arm's rest pose and the clear volume around it and tighten
  them before the first real run. `mg400_address` is empty on purpose: a lab address never
  goes into a tracked file (set it in your own `config.yaml`, or `--robot5-mg400 <address>`
  / `ROBOT5_MG400_ADDRESS`).

```bash
cp examples/mg400/config.mg400.yaml config.yaml      # then edit it
python -m src.flow.run_flow examples/mg400/z_hop.json --validate-only
python -m src.flow.run_flow examples/mg400/z_hop.json --mock
```

The real run is the human's: the runner prints what will happen and waits for Enter on a
terminal before any device is opened (`docs/setup.md` §13 lists what to check first, and
what Ctrl+C does: an orderly stop, not an emergency stop). The CI runs the two commands
above against this configuration.
