---
name: control-loop-change
description: Procedure and real-time rules for changing anything in ROCKO's 100 Hz control path — hardware plugins (Motor12VoltQuadEncoder, ICM20948, QuadEncoder), controllers (AddFeedforwardController), update_rate, or the interface wiring in rocko_controllers.yaml. Use before editing any of those, and when asked about loop latency, jitter, blocking calls, real-time safety, or why the balance loop is unreliable.
---

# Changing the 100 Hz control path

## Stop and teach first

This is the most educationally valuable code in the repo and the easiest to break invisibly. Before
any edit, produce for the human:

1. **Which cycle stage** you are touching — `read()`, `update()`, or `write()` — and what else runs
   in that same 10 ms.
2. **The concept.** Why does this matter for a balancing robot? Loop delay eats phase margin: an
   inverted pendulum with a 20 ms feedback delay is a genuinely harder plant than the same robot at
   2 ms, and no gain set recovers the difference. Say that in your own words, for this change.
3. **A measurement or an admission.** Either the number that justifies the change, or a clear
   statement that you are reasoning without one.
4. **Your open questions**, then wait.

## The budget

`update_rate: 100` in `rocko_env/rocko_env/bringup/config/rocko_controllers.yaml` means **10 ms**
for the whole chain, on Pi-class hardware, with Linux's default scheduler. Every hardware plugin's
`read()`, every controller's `update()`, and every `write()` shares that window.

## Hard rules inside read/update/write

Never, in the loop:

- **Block.** No service calls, no `spin_until_future_complete`, no futures, no sleeps, no unbounded
  waits. Device I/O goes *inside* the hardware plugin, in C++, in-process. A Python node reached over
  DDS is not a driver — that round trip is unbounded and jittery by construction.
- **Allocate.** No `new`/`malloc`, no container growth, no `std::string` building, no `std::function`
  that captures by value. Size everything in `on_init`/`on_configure` and reuse it.
- **Log unconditionally.** `RCLCPP_INFO` takes a lock and can hit a syscall. Use
  `RCLCPP_INFO_THROTTLE`, or push telemetry through `realtime_tools::RealtimePublisher`.
- **Touch the filesystem**, or take a mutex that a non-RT thread also holds. Cross the boundary with
  `realtime_tools::RealtimeBuffer` (single producer, single consumer) or `RealtimeBox`.
- **Fail into a safe-looking value.** This one is worth its own line: a failed IMU read reported as
  pitch = 0.0 tells the controller the robot is perfectly upright at the exact moment it has no idea
  where it is. Return `hardware_interface::return_type::ERROR`, or hold the last sample *and* set a
  fault flag something acts on. Never invent data.

## Rules specific to this robot

- **One IMU transaction per cycle.** Reading roll, pitch, and yaw as three separate I²C
  transactions produces an attitude from three different instants — an orientation the robot was
  never in. Read once, use that snapshot.
- **Derivative comes from the gyro**, not from differentiating fused pitch. The fusion filter adds
  delay, and differentiating a delayed signal gives a D term that fights you.
- **Clamp at the hardware boundary.** Every value written to PWM is range-checked in `write()`, not
  trusted from upstream.
- **Software PWM period must be well above the control period.** Equal periods (both 100 Hz today)
  alias, and one lost write latches the previous duty cycle.
- **Near-zero is still a command.** Skipping the write for small values leaves the last PWM latched
  on the motor. Write it.
- **Determinism beats average speed.** A bounded 3 ms is better than a usually-0.5 ms that sometimes
  takes 30. Optimize the worst case.

## Files in the critical path

- `rocko_env/rocko_env/hardware/motors/Motor12VoltQuadEncoder.cpp`, `include/rocko_env/wheel.hpp`
- `rocko_env/rocko_env/hardware/sensors/icm20948/ICM20948.cpp`
- `rocko_env/rocko_env/hardware/sensors/quadEncoder/QuadEncoder.cpp`
- `rocko_env/rocko_env/controllers/add_feedforward/AddFeedforwardController.cpp`
- `rocko_env/rocko_env/bringup/config/rocko_controllers.yaml`

## After implementing

Run the review loop from `CLAUDE.md` §5. `realtime-auditor` is **mandatory** here. Iterate until it
returns clean, cap at three rounds, then report.

## Known findings — read before proposing anything

[`docs/claude-analysis/balance-loop-findings.md`](../../../docs/claude-analysis/balance-loop-findings.md)
already catalogues the live problems (blocking service calls in the loop, IMU failure reporting
upright, open-loop motor command, no tipover cutoff, no watchdog) with severities and locations.
[`bno055-improvements.md`](../../../docs/claude-analysis/bno055-improvements.md) covers the sensor side.

Check whether your idea is already written up there. If it is, reference the finding ID (C-1, H-3, …)
instead of rediscovering it, and **respect the stated ordering** — C-1 dominates the sensor work, so
sensor improvements landed first will be measured against a still-broken feedback path.
