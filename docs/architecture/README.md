# Architecture

How ROCKO's subsystems work and why they are built this way. These pages explain; for procedures see
[`../guides/`](../guides/), and for the reasoning behind a specific choice see
[`../decisions/`](../decisions/).

| Page | Covers |
| --- | --- |
| [`control-cascade.md`](control-cascade.md) | The nested velocity → pitch → feedforward control loop |
| [`verification-ladder.md`](verification-ladder.md) | The five tiers of checking a change, what each proves and cannot show, and which exist today |

The [root README](../../README.md) holds the end-to-end signal path diagram (laptop gamepad → TCP →
Pi → controller chain → motors) and the repository folder map. It is the canonical version of both —
these pages link to it rather than repeating it.

## Still to write

Honest list of gaps, so nobody assumes a missing page means a missing subsystem:

- **Hardware wiring** — GPIO pins, I²C bus and address, motor driver, battery. Partly recorded in
  `rocko_env/rocko_env/description/ros2_control/*.xacro`; not yet written up.
- **Sensor stack** — the fitted part is a BNO055 despite the `ICM20948` naming; fusion mode,
  calibration procedure, what the exported interfaces mean. See
  [`../claude-analysis/bno055-improvements.md`](../claude-analysis/bno055-improvements.md) for the
  known issues in the meantime.
- **Telemetry webserver** — the Flask app in `rocko_env/rocko_env/webserver/`.
