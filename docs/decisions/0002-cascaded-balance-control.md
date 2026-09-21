# ADR 0002 — Balance using a nested velocity → pitch PID cascade

- **Status:** Accepted
- **Date:** 2026-09-20 (recorded; the design predates this record)
- **Deciders:** ROCKO capstone team

## Context

ROCKO is an inverted pendulum on two wheels. Its only actuator is the wheel pair, and that one
actuator has to serve two competing goals: stay upright, and go where it was told.

The coupling is awkward in a specific way: to accelerate forward the robot must first lean forward,
and to start leaning forward it must briefly drive the wheels *backward*. A controller that maps
"go forward" directly to "spin wheels forward" therefore fights balance rather than serving it.

This is also a teaching platform, so the control structure has to be explainable to a student who
has taken one controls course.

## Decision

Two PID loops in a cascade, implemented as **chained `ros2_control` controllers** so the whole chain
runs in one thread at one rate:

1. `velocity_pid_controller` (outer) — measures wheel velocity, commands a **target pitch angle**
   (`command_interface: pitch`).
2. `pitch_pid_controller` (inner) — measures pitch from the IMU, commands **wheel velocity**
   (`command_interface: velocity`).
3. `left/right_add_feedforward_controller` — chainable; adds a feedforward term and drives the
   `Motor12VoltQuadEncoder` hardware plugin.

The inner loop owns stability, the outer loop owns performance. The entire chain runs at
`update_rate: 100` Hz in the `controller_manager` thread.

Full explanation, with the diagram, in
[`../architecture/control-cascade.md`](../architecture/control-cascade.md).

## Alternatives considered

- **One PID from velocity error to wheel command.** Rejected: it cannot express the lean-to-accelerate
  inversion, so it has no way to trade tracking against staying upright.
- **Full state-space / LQR on the linearized pendulum.** A legitimate and arguably better fit for the
  plant. Not chosen: it needs a physical model (masses, inertias, wheel radius) the project does not
  yet have measured, and it is markedly harder for a student to reason about than two PIDs whose terms
  map to physical quantities. Worth revisiting once a simulator exists ([ADR 0003](0003-simulation-target.md))
  and the model is identified.
- **Separate ROS nodes per loop, connected by topics.** Rejected: that puts DDS between the loops,
  adding latency and jitter to the inner, stability-critical loop. `ros2_control` chaining keeps the
  whole cascade in one thread at one rate.

## Consequences

- **Good:** the structure matches the physics, and each gain has a meaning a student can name. The
  chain is deterministic — one thread, one rate, no message passing between loops. Tank mode
  (`rocko_tank_mode.launch.py`) can bypass the cascade entirely for non-balancing work.
- **Bad:** tuning is strictly ordered — the outer loop's plant *is* the closed inner loop, so
  retuning pitch invalidates every velocity gain. Two loops means roughly twice the tuning surface,
  and cascades are prone to integral wind-up in the outer loop when the inner loop cannot deliver the
  commanded lean. The current config carries `i: 5.0` on the outer loop with `i_clamp: ±150`, which is
  worth revisiting.
- **Follow-on work:** the cascade is only as good as the feedback reaching it, and today that feedback
  path is unsound — see [`../claude-analysis/balance-loop-findings.md`](../claude-analysis/balance-loop-findings.md)
  and [ADR 0004](0004-device-io-in-cpp.md). Gains tuned before those are fixed will not transfer.

## References

- `rocko_env/rocko_env/bringup/config/rocko_controllers.yaml` — the chain and the gains
- `rocko_env/rocko_env/controllers/add_feedforward/AddFeedforwardController.cpp`
- [ros2_control chained controllers](https://control.ros.org/jazzy/doc/ros2_control/controller_manager/doc/controller_chaining.html)
