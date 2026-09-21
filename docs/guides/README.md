# Guides

Procedures: do this, get that. For *why* something is built the way it is, see
[`../architecture/`](../architecture/) and [`../decisions/`](../decisions/).

| Guide | For |
| --- | --- |
| [`reproducing-results.md`](reproducing-results.md) | Recording a result so someone else gets the same number |

Building, running, and the Docker dev container are in the [root README](../../README.md) — it is the
canonical copy, deliberately not duplicated here.

## What a guide in here must have

1. **Which machine** — your laptop, the Pi, or inside the container. They differ, and the gamepad
   script runs on a *different machine* from the robot code.
2. **What to source first** — `source /opt/ros/jazzy/setup.bash`, then the workspace overlay, in every
   new terminal.
3. **Which directory** — workspace root vs. `src/ROCKO-env`. `colcon` cares.
4. **What success looks like** — the expected output, not just the command.
5. **What commonly goes wrong**, and why.

## Still to write

- Bringing up the robot from cold: power-on order, network, finding the Pi's IP.
- IMU calibration, start to finish.
- Reading the telemetry webserver's plots.
