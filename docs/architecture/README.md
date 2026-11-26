# Architecture

How ROCKO's subsystems work and why they are built this way. These pages explain; for procedures see
[`../guides/`](../guides/), and for the reasoning behind a specific choice see
[`../decisions/`](../decisions/).

| Page | Covers |
| --- | --- |
| [`control-cascade.md`](control-cascade.md) | The nested velocity → pitch → feedforward control loop |
| [`robot-supervisor.md`](robot-supervisor.md) | The supervisor: tank/balance mode, enable/disable, who starts what at boot, and the rules. Written 2026-11-23, not run on the robot |
| [`sim-plant.md`](sim-plant.md) | `SimPlant`, the simulated robot for Tier 3: its equations, signs, estimated parameters and what it leaves out. A model, not the robot |
| [`sim-to-real-gap.md`](sim-to-real-gap.md) | What the simulated robot leaves out and what each omission would do to a gain (reasoning, not measurement) |
| [`verification-ladder.md`](verification-ladder.md) | The five tiers of checking a change, what each proves and cannot show, and which exist today |

The [root README](../../README.md) holds the end-to-end signal path diagram (USB gamepad on the Pi → joy →
teleop node → controller chain → motors) and the repository folder map. It is the canonical version of both —
these pages link to it rather than repeating it.

## Still to write

Honest list of gaps, so nobody assumes a missing page means a missing subsystem:

- **Hardware wiring** — GPIO pins, I²C bus and address, motor driver, battery. Partly recorded in
  `rocko_env/rocko_env/description/ros2_control/*.xacro`; not yet written up.
- **Sensor stack** — the fitted part is a BNO055 despite the `ICM20948` naming; fusion mode,
  calibration procedure, what the exported interfaces mean. See
  [`../claude-analysis/bno055-improvements.md`](../claude-analysis/bno055-improvements.md) for the
  known issues in the meantime.
- **Dashboard and recording** — `rocko_env/rocko_env/webserver/Dashboard.py` (standard library; the
  Flask app it replaced is gone) and `rocko_env/rocko_env/recording/RecordRun.py`. No architecture page;
  each guide ends with how its tool works: [use-the-dashboard.md](../guides/use-the-dashboard.md),
  [record-a-run.md](../guides/record-a-run.md).
