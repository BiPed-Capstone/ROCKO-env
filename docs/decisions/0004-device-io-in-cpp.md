# ADR 0004 — Device I/O belongs in C++ hardware plugins, not Python nodes

- **Status:** Proposed — describes the intended end state, not the current code
- **Date:** 2026-09-20
- **Deciders:** ROCKO capstone team

## Context

Sensor reads currently reach the control loop the long way round. The C++ hardware plugins call into
**Python nodes over ROS 2 services**, and `ICM20948.cpp` makes those calls from inside `read()` —
three of them per cycle, with **no timeout**
(`rocko_env/rocko_env/hardware/sensors/icm20948/ICM20948.cpp:72-75`).

The `controller_manager` runs at 100 Hz, a 10 ms budget for the whole `read` → `update` → `write`
chain. A blocking service call has no bounded worst case: it involves DDS discovery, serialization,
a hop through the Python interpreter and the GIL, and a scheduler round trip. So sensor data arrives
**late, and late by a varying amount**.

For an inverted pendulum this is not a performance detail. Feedback delay directly consumes phase
margin, and *variable* delay means the plant the PID sees is not the same plant from one cycle to the
next. Gains tuned against it do not describe the robot.

Recorded as finding **C-1** in
[`../claude-analysis/balance-loop-findings.md`](../claude-analysis/balance-loop-findings.md), which
ranks it as the problem dominating all the others.

## Decision

All device I/O on the control path moves **into the C++ hardware plugin, in-process**. No ROS service
call, no topic round trip, and no Python in the 100 Hz path.

This is what `ros2_control` hardware plugins are *for*. A plugin exists precisely so the loop can talk
to a device directly, in a bounded amount of time, without leaving the process.

Python keeps everything off the critical path: calibration scripts, the telemetry webserver, the
laptop gamepad bridge, offline analysis. That is a good use of Python and none of it should be
rewritten.

## Alternatives considered

- **Keep the service calls but add a timeout.** Bounds the worst case instead of fixing it, and turns
  a latency problem into a dropped-sample problem — which then needs a failure policy anyway. Worth
  doing as a stopgap; not the resolution.
- **Move to topics with a real-time-safe subscriber** (`realtime_tools::RealtimeBuffer`, latest-value
  semantics). Genuinely better than services and a legitimate pattern for *non-critical* inputs.
  Still a DDS hop and still the Python interpreter for a read that could be a direct bus transaction.
- **Rewrite the loop outside ROS.** Rejected. `ros2_control` is a good fit for this robot; the Python
  service layer is the anti-pattern, not the framework around it. Leaving ROS would also throw away
  the teaching value of a standard stack.

## Consequences

- **Good:** bounded, low-latency sensor reads; one coherent sample per cycle instead of three reads at
  three instants; a real chance that tuning results mean something. It also makes the architecture's
  own logic legible to students — "device I/O lives in the plugin" is a rule with a visible reason.
- **Bad:** the I²C/UART driver has to be written in C++, which is more work and less approachable than
  the Python equivalent, on a team where Python is the common language. Losing the Python layer also
  loses a convenient place to inspect raw sensor values, so telemetry has to be re-provided
  deliberately (a throttled `RealtimePublisher`, not a print in the loop).
- **Follow-on work:** the BNO055 read is the first target, and moving it is a natural moment to also
  take the sensor-side improvements in
  [`../claude-analysis/bno055-improvements.md`](../claude-analysis/bno055-improvements.md) — UART
  instead of I²C, one transaction per cycle, gyro rate for the derivative term. Note the ordering:
  a better sensor signal sampled through a blocking call is still a bad signal.

## References

- `rocko_env/rocko_env/hardware/sensors/icm20948/ICM20948.cpp:72-75` — the blocking calls
- [`../claude-analysis/balance-loop-findings.md`](../claude-analysis/balance-loop-findings.md) — C-1, H-3, H-6
- [ros2_control hardware components](https://control.ros.org/jazzy/doc/ros2_control/hardware_interface/doc/hardware_components_userdoc.html)
