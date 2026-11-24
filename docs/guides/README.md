# Guides

Procedures: do this, get that. For *why* something is built the way it is, see
[`../architecture/`](../architecture/) and [`../decisions/`](../decisions/).

| Guide | For |
| --- | --- |
| [`reproducing-results.md`](reproducing-results.md) | Recording a result so someone else gets the same number |
| [`verify-in-container.md`](verify-in-container.md) | Building and checking a change on your laptop with no robot: build, static checks, the mock, and the simulated robot (`verify_sim.sh`) |
| [`drive-with-usb-controller.md`](drive-with-usb-controller.md) | Driving with a game controller (8BitDo) plugged into the Pi: enable with LB+RB, split arcade, switching tank/balance, starting at boot, what stops the robot. The only way to drive since 2026-11-22 |
| [`run-on-the-pi.md`](run-on-the-pi.md) | Running anything on the Pi: why every `ros2` command there runs as root, and the scripts in `pi/` that do it (`launch.sh`, `rocko_shell.sh`, `rocko_run.sh`); the boot service (`rocko.service`); the workspace is `~/ROCKO-env` |
| [`bench-test-tank-mode.md`](bench-test-tank-mode.md) | First run of tank mode with the wheels off the ground: do both wheels turn, at what command do they start, and why small commands only make the motors hum |
| [`record-a-run.md`](record-a-run.md) | Recording a run on the robot with `record:=true`: what is recorded, the `run.yaml` that describes it, copying runs to a laptop |
| [`use-the-dashboard.md`](use-the-dashboard.md) | Watching the robot's state and values in a browser on another machine, enabling/disabling and switching mode from the page (no login), and recording from it |
| [`measure-inertia.md`](measure-inertia.md) | Measuring the robot (mass, centre of mass, inertia, wheel radius, motor constants, IMU delay, encoder scale) to replace the simulator's estimates |

Building and running the robot code, and the Docker dev container, are in the
[root README](../../README.md) — the canonical copy, deliberately not duplicated here.
[`verify-in-container.md`](verify-in-container.md) is the narrower walk-through for checking a change
without the robot; it repeats only the commands and what to expect.

## What a guide in here must have

1. **Which machine** — your laptop, the Pi, or inside the container. They differ, and the robot code runs on the Pi.
2. **What to source first** — `source /opt/ros/jazzy/setup.bash`, then the workspace overlay, in every
   new terminal. On the Pi, `pi/rocko_shell.sh` does this (as root, which the robot needs); say so
   instead of listing the `source` lines ([Run ROCKO on the Pi](run-on-the-pi.md)).
3. **Which directory** — workspace root vs. `src/ROCKO-env`. `colcon` cares.
4. **What success looks like** — the expected output, not just the command.
5. **What commonly goes wrong**, and why.

## Still to write

- Bringing up the robot from cold: power-on order, network, finding the Pi's IP.
- IMU calibration, start to finish.
