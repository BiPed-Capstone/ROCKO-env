# Architecture

How ROCKO's subsystems work and why they are built this way. These pages explain; for procedures see
[`../guides/`](../guides/), and for the reasoning behind a specific choice see
[`../decisions/`](../decisions/).

| Page | Covers |
| --- | --- |
| [`control-cascade.md`](control-cascade.md) | The nested velocity → pitch → feedforward control loop |
| [`imu-i2c-read.md`](imu-i2c-read.md) | How the IMU plugin reads the BNO055: one bounded I²C burst per cycle inside the plugin (finding C-1) |
| [`encoders-in-cpp.md`](encoders-in-cpp.md) | How the motor plugin counts its wheel encoder's edges and filters the speed (finding C-1) |
| [`robot-supervisor.md`](robot-supervisor.md) | The supervisor: tank/balance mode, enable/disable, who starts what at boot, and the rules. Runs on the robot since 2026-11-24 |
| [`robot-extras.md`](robot-extras.md) | The optional status screen, buzzer and battery monitor: why none of them can stop the robot, how each reports its health, and how to change the screen and the sounds. Not run on the Pi yet |
| [`sim-plant.md`](sim-plant.md) | `SimPlant`, the simulated robot for Tier 3: its equations, signs, estimated parameters and what it leaves out. A model, not the robot |
| [`sim-to-real-gap.md`](sim-to-real-gap.md) | What the simulated robot leaves out and what each omission would do to a gain (reasoning, not measurement) |
| [`verification-ladder.md`](verification-ladder.md) | The five tiers of checking a change, what each proves and cannot show, and which exist today |

The [root README](../../README.md) holds the end-to-end signal path diagram (USB gamepad on the Pi → joy →
teleop node → controller chain → motors) and the repository folder map. It is the canonical version of both —
these pages link to it rather than repeating it.

## Still to write

Honest list of gaps, so nobody assumes a missing page means a missing subsystem:

- **Hardware wiring** — motor driver, power. The header's pin map (motors, encoders, IMU, and the screen, buzzer and
  battery monitor) is in [Fit the extras](../guides/fit-the-extras.md#1-the-pins-and-the-conflict-to-fix-first); the
  motor driver and the power wiring are not written up.
- **Sensor calibration** — the fitted part is a BNO055, read by the `BNO055` plugin. How it is read and
  what the exported interfaces mean is in [`imu-i2c-read.md`](imu-i2c-read.md); the BNO055's own
  calibration procedure is not written up (see
  [`../claude-analysis/bno055-improvements.md`](../claude-analysis/bno055-improvements.md)).
- **Dashboard and recording** — `rocko_env/rocko_env/webserver/Dashboard.py` (standard library; the
  Flask app it replaced is gone) and `rocko_env/rocko_env/recording/RecordRun.py`. No architecture page;
  each guide ends with how its tool works: [use-the-dashboard.md](../guides/use-the-dashboard.md),
  [record-a-run.md](../guides/record-a-run.md).
