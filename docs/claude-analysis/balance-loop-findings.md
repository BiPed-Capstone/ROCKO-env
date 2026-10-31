# Balance Loop — Findings and Required Changes

**Date:** 2026-09-20
**Scope:** The 100 Hz `controller_manager` loop and everything in its critical path — IMU read, encoder read, motor write, and the failsafe behaviour around them.
**Status of the robot at time of review:** tank mode works; balancing is partially tuned and not reliable.
**See also:** [`bno055-improvements.md`](bno055-improvements.md) — sensor-side changes available
without replacing the IMU.

---

## Summary

The balance controller is being tuned against a feedback path that is neither prompt nor
trustworthy. Three separate problems compound:

1. Every control cycle makes **three blocking ROS 2 service calls into Python nodes**, with no
   timeout. Sensor data reaches the controller late and with variable latency.
2. When the IMU read fails, the robot is told it is **perfectly upright**.
3. The motor command path is **open loop** — commanded wheel velocity is converted directly to a
   PWM duty cycle with no feedback — so the plant gain the PID sees changes with battery voltage
   and wheel speed.

No set of PID gains fixes any of these. Tuning effort spent before they are addressed is likely
to be wasted, because the plant will change character once they are.

The good news: the fixes mostly consist of *deleting* indirection. ROS 2 and `ros2_control` are
appropriate for this robot and nothing here argues for changing framework. Hardware plugins exist
precisely so that device I/O happens in-process in C++; the Python service layer is the
anti-pattern, not the architecture around it.

---

## Findings

Severity reflects impact on the robot staying upright, not code quality.

| ID | Severity | Finding |
| --- | --- | --- |
| C-1 | Critical | Unbounded blocking service calls inside the real-time loop |
| C-2 | Critical | IMU read failure reports pitch = 0.0 ("upright") — **fixed on `ai-forward`, awaiting robot** (2026-10-31); a frozen sensor is still undetected (H-9) |
| C-3 | Critical | No teleop watchdog, and no defined comms-loss failsafe |
| H-1 | High | Motor command is open loop; no wheel velocity feedback |
| H-2 | High | Near-zero commands never write, latching the previous PWM value — **code fixed on `ai-forward`, compile-checked only, awaiting robot** (2026-10-31; pulled forward of the planned order) |
| H-3 | High | IMU sample is three separate I²C reads at three different instants |
| H-4 | High | No tipover cutoff |
| H-5 | High | No real-time scheduling or CPU isolation for `controller_manager` |
| H-6 | High | Gyro angular rate is never read; the D term differentiates a delayed fused angle |
| H-7 | Critical (raised from Medium/High by the `realtime-auditor`; unconfirmed on hardware) | The motors may keep their last command when the cascade stops or the IMU plugin returns ERROR (found 2026-10-31) — **partly mitigated on `ai-forward`** (controller stop logic and the motor `write()` fix, none run on a robot) |
| H-8 | High | An intermittent IMU fault that alternates good and bad reads never tripped the consecutive-only counter (found 2026-10-31) — **fixed in the tracker on `ai-forward`, awaiting robot** |
| H-9 | High (not fixed) | A frozen or reset BNO055 that still reads valid-looking numbers is undetectable (found 2026-10-31); one variant, an all-equal Euler triple, is now rejected in the node |
| M-1 | Medium | `RCLCPP_INFO` called every cycle from inside `write()` — **removed on `ai-forward`, compile-checked only** (2026-10-31) |
| M-2 | Medium | Software PWM period equals the control period (both 100 Hz) |
| M-3 | Medium | PWM value is never clamped to its valid range — **clamped on `ai-forward`, compile-checked only, awaiting robot** (2026-10-31) |
| M-4 | Medium | `use_mock_hardware` is dead; the project cannot be built or tested off the Pi — **partly fixed on `ai-forward`** (wiring mock and off-Pi build exist; no physics) |
| M-5 | Medium | `AddFeedforwardController` ignores the result of `set_value()`, so a failed command write is silently dropped — **fixed on `ai-forward`, awaiting robot** (2026-10-31) |
| M-6 | Medium | `AddFeedforwardController`'s feedforward value was uninitialised and shared between two threads without synchronisation (found 2026-10-31) — **fixed on `ai-forward`** |
| L-1 | Low | `joint_name_` is at namespace scope and shared between both motor instances |
| L-2 | Low | IMU class is named for a part that is no longer used; dead imports |
| L-3 | Low | Deprecated header include and invalid escape sequences, seen as warnings in the first off-Pi build — **fixed on `ai-forward`** (2026-10-31) |
| L-4 | Low | The RViz configs are never installed, so three launch files cannot find them — **fixed on `ai-forward`** (2026-10-31); RViz opening with them not checked |
| L-5 | Low | A D-term spike after cycles in which the IMU angle was held (found 2026-10-31, not fixed) |
| L-6 | Low now, Critical once C-1 adds a timeout | `Motor12VoltQuadEncoder::read` and `QuadEncoder::read` return OK and keep the old position and velocity when the encoder call fails (found 2026-10-31, not fixed) |
| L-7 | Low | `MAX_RAD_PER_SEC` is computed with integer division (found 2026-10-31, not fixed; a human confirms the change) |

---

### C-1 — Unbounded blocking service calls inside the real-time loop

**Where**
- `rocko_env/rocko_env/hardware/sensors/icm20948/ICM20948.cpp:72-74`
- `rocko_env/rocko_env/hardware/motors/Motor12VoltQuadEncoder.cpp:183-185` (runs twice per cycle,
  once per motor instance)

*Line numbers are at audit time (before the 2026-10-31 edits).*

**What**

Each `read()` issues `async_send_request()` and then calls `spin_until_future_complete()` with no
timeout argument. Per 10 ms control cycle the loop therefore performs three synchronous
round-trips of the form:

```
C++ plugin -> DDS serialize -> wake Python process -> I2C / GPIO
           <- DDS deserialize <- Python response    <-
```

**Why it matters**

- The round-trip cost is large relative to a 10 ms budget and, more importantly, *variable*.
  A Python garbage-collection pause or an I²C retry lands directly on the control period.
- With no timeout, `spin_until_future_complete` blocks indefinitely. If a Python node dies or
  stalls, `controller_manager` stops updating and the robot falls over with the motors still
  latched at their last commanded value.
- Variable feedback latency is indistinguishable from phase lag in the plant. It directly
  destabilises a derivative term, which is why the pitch loop is hard to tune.

**Fix**

Move the device I/O into the C++ plugin. Read the IMU over I²C and the encoders over GPIO
directly in `read()`. Delete `Icm20948Data.srv`, `QuadEncoderData.srv`, the Python nodes that
serve them, and the client members in both plugins.

A single burst read of the BNO055 Euler register block at 400 kHz I²C is on the order of 200 µs
against a 10 ms budget, so there is ample headroom.

If any service-based plugin must be kept for some reason, the minimum acceptable form is
`spin_until_future_complete(_node, result, std::chrono::milliseconds(2))` with an explicit
handler for the timeout branch — but the right answer is to remove the indirection entirely.

**Note for whoever adds that timeout** (from review round 1 of the C-2 change, 2026-10-31; not tried):
the timeout branch must also remove the pending request. A client keeps each request it has sent
until a reply arrives, so abandoning a request without removing it makes that list grow every time
the timeout fires.

---

### C-2 — IMU read failure reports pitch = 0.0

**Where** `rocko_env/rocko_env/hardware/sensors/icm20948/ICM20948.py:36-41`

**What**

```python
try:
    response.yaw   = self.icm.euler[0]
    response.roll  = self.icm.euler[1]
    response.pitch = self.icm.euler[2] + self.pitch_offset
except Exception as e:
    self.get_logger().warn("Unable to read gyro data this cycle: " + str(e))
return response
```

On exception the assignments do not happen and `response` is returned with its default-initialised
fields, i.e. **pitch = 0.0**.

**Why it matters**

Pitch 0.0 is the balanced setpoint. A transient I²C error therefore tells the pitch controller the
robot is level at the exact moment it has no idea what the robot is doing. If the robot is actually
leaning, the controller relaxes instead of correcting.

This is not a rare edge case. The part fitted is a BNO055, and **the BNO055's I²C clock stretching
is a documented incompatibility with the Raspberry Pi's I²C hardware** — Adafruit's own guide states
that "the BNO055's use of I2C clock stretching will cause problems with a hardware I2C clock
stretching bug in the Raspberry Pi" and recommends UART instead. **How often this actually fires on ROCKO is unmeasured and must be determined on the real
hardware** — the bug's status on the Pi 4 (BCM2711) is contested, see
[`bno055-improvements.md`](bno055-improvements.md) § 2.1.

Note that the severity of this finding does not depend on that question. *Any* cause of a failed
read — clock stretching, a loose connector, electrical noise from the motors, a brownout — lands in
this handler and produces the same "perfectly upright" lie. The handler is wrong regardless of how
often it runs.

The standard I²C mitigation — dropping the bus to 10 kHz via `dtparam=i2c_arm_baudrate=10000` — is
not usable here: at that rate a single block read costs roughly 9 ms against a 10 ms control period.
Moving the sensor to UART is the viable fix. See
[`bno055-improvements.md`](bno055-improvements.md) § 2 for the wiring and protocol details and the
supporting references.

**Fix**

Never synthesise a plausible-looking value on failure. In the C++ plugin after C-1 is fixed:

- hold the previous good pitch,
- increment a consecutive-failure counter,
- return `hardware_interface::return_type::ERROR` after N consecutive failures (N ≈ 5 at 100 Hz,
  i.e. 50 ms of blindness), which deactivates the controllers,
- surface the counter as a diagnostic so a failing sensor is visible before the ceremony rather
  than during it.

**Status on `ai-forward` (2026-10-31):** the fix is written and checked as far as the
[ladder](../architecture/verification-ladder.md) allows; it has not been run on a robot. Landed
(next-steps row 0.1): the `Icm20948Data.srv` response has a `bool success`; `ICM20948.py` answers
`success = false` with NaN angles when a read fails; `ICM20948.cpp` counts a read as good only if the
call worked, `success` is true, all three angles are finite, and they are plausible (`|pitch|` and
`|roll|` at most 180 deg, `|yaw|` at most 360 deg; a garbled value counts as a failed read). Otherwise
it holds the last good angles, counts the failure, logs a throttled error, and returns `ERROR` while
the new `ImuFaultTracker` is tripped. The tracker trips on **either** `kMaxConsecutiveImuFailures = 5`
failed reads in a row (50 ms at 100 Hz) **or** `kMaxWindowImuFailures = 10` failed reads among the last
20 (200 ms), which is [H-8](#h-8--an-intermittent-imu-fault-never-tripped-the-consecutive-counter).
Both limits are **proposed** values that a human confirms. The trip is not latched: a good read clears
the consecutive run at once, and the windowed count falls as good reads push old failures out of the
window. The angles start as NaN, so a failure on the first cycle cannot read as 0.0.

**Added after review round 2 (2026-10-31):** `ICM20948.py` also answers `success = false` when all three
Euler values are exactly equal. Real data has three independent angles at 1/16 deg resolution, and an
all-zero block is a plausible bus-fault or reset output that `(0, 0, 0) + pitch_offset` would turn into
a believable pitch. That is a guess about what a fault returns; it has not been seen on a sensor and no
tier runs the node. It also would reject a genuine reading with three identical values.

Checked: the Tier 1 build compiles it with no compiler warnings; the tracker's counting logic has a
host unit test (`tools/imu_fault_tracker_test.cpp`, not a ladder tier) that passes and was seen to fail
on three deliberate breaks. The mock replaces this plugin, so Tier 2 passes but cannot see it.
Not done: the counter is not exported as a diagnostic (only the throttled log shows it); **C-1 is
still open** (`read()` still makes a blocking service call, so a dead `icm20948` node blocks it
forever and no failure is ever counted); **[H-9](#h-9--a-frozen-or-reset-bno055-is-undetectable)
is open** (a frozen or reset sensor is not detected). Also open: whether `ERROR` stops the wheels,
which is [H-7](#h-7--the-motors-keep-their-last-command-when-the-cascade-stops), and what
`controller_manager` does after `ERROR`, which depends on the ros2_control version and has not been
observed. Robot procedure: [journal](../journal/2026-10-31-sim-plant-and-critical-fixes.md).

---

### C-3 — No teleop watchdog, and no defined comms-loss failsafe

**Where** No implementation found. `Joystick.py` accepts a TCP connection and republishes; nothing
ages the command out.

**What**

If the operator's link drops, the last received velocity command remains in effect indefinitely.

**Why it matters**

This is the most likely wedding-day failure mode, and the correct failsafe for a balancing robot
is **the opposite of the usual one**. For a statically stable robot, comms loss means stop the
motors. For an inverted pendulum, stopping the motors *is* falling over.

**Fix**

Timestamp every incoming command. If the newest command is older than ~200 ms:

- zero the **velocity reference**, and
- keep the balance loop running so the robot stands still, upright, until the link returns.

Do not deactivate controllers, and do not zero the motor output. Make this explicit in code with a
comment, because the intuitive "safe" behaviour is the dangerous one here.

---

### H-1 — Motor command is open loop

**Where** `rocko_env/rocko_env/hardware/motors/Motor12VoltQuadEncoder.cpp:205-206`

*Line numbers are at audit time (before the 2026-10-31 edits).* The snippet below is the audit-time code. It no longer exists: `write()` was rewritten on 2026-10-31 (see H-2 and M-3). The open-loop mapping it shows is still how a command becomes a duty cycle.

```cpp
double radPerSec = _wheel.cmd;
int pwmVal = (std::abs((radPerSec / MAX_RAD_PER_SEC)) + 0.02) * 100;
```

**What**

The commanded wheel velocity is converted to a duty cycle by a fixed linear map. The encoder
velocity is read, but it feeds the *outer* robot-velocity loop only — there is no loop closing
commanded wheel velocity against measured wheel velocity.

**Why it matters**

Duty cycle is approximately voltage, not velocity and not torque. The relationship between them
varies with:

- **battery state of charge** — the plant gain drifts down over the course of an afternoon,
  so gains tuned on a full pack are wrong on a half-empty one;
- **wheel speed** — back-EMF opposes applied voltage, so the same duty cycle produces
  progressively less torque as the wheel spins up;
- **stiction and load** — worst exactly at low speed, which is where a balancing robot lives.

A PID controller assumes a roughly linear, roughly time-invariant plant. This plant is neither.
Much of the tuning difficulty is likely traceable here.

**Fix**

Close a wheel-velocity loop with encoder feedback. This can be a small PI loop inside the plugin,
or delegated to a motor controller that does it in hardware (see *Related hardware work* below).
Also calibrate `MAX_RAD_PER_SEC` against measured no-load speed rather than a datasheet figure.

---

### H-2 — Near-zero commands never write, latching the previous PWM value

**Where (line numbers at audit time, before the 2026-10-31 edit)** `rocko_env/rocko_env/hardware/motors/Motor12VoltQuadEncoder.cpp:206, 219-220`

**What**

`pwmVal` bottoms out at exactly `2` because of the `+ 0.02` offset, and the write is guarded by
`if (pwmVal > 2)`. At zero command `pwmVal == 2`, the guard is false, and `softPwmWrite` is never
called — so the motor **holds whatever duty cycle was last written**.

Integer truncation widens the dead band: any command where `|radPerSec| / MAX_RAD_PER_SEC < 0.01`
produces `pwmVal == 2` and is silently dropped.

**Why it matters**

Small corrections near equilibrium are precisely what a balance controller spends its time issuing,
and they are exactly the commands being discarded. The motor instead continues executing a stale,
larger command. This produces hunting and limit-cycle oscillation around the setpoint.

**Fix**

Always write. Remove the `+ 0.02` fudge and the `> 2` guard; if a minimum duty cycle is needed to
overcome stiction, apply it as an explicit, commented stiction-compensation term with a true
zero case:

```cpp
if (std::abs(radPerSec) < kDeadbandRadPerSec) {
  softPwmWrite(_speedPin, 0);
} else {
  softPwmWrite(_speedPin, std::clamp(pwmVal, 0, 100));
}
```

**Status on `ai-forward` (2026-10-31): code written, pulled forward; compile-checked only (type and
syntax), awaiting robot.** The audit's order of work puts this at step 5 and the person's stage order
put it in stage 4; review round 2 showed that the stop paths of C-2, M-5 and H-4 are meaningless while
this guard exists, so it landed earlier. `Motor12VoltQuadEncoder::write()` now always calls
`softPwmWrite()`. A non-finite command, or one with `|cmd|` below `kDeadbandRadPerSec = 0.05` rad/s,
writes 0 % and leaves the direction pin untouched. Otherwise duty is
`clamp(|cmd| / MAX_RAD_PER_SEC * 100 + kStictionDutyPercent, 0, 100)` with `kStictionDutyPercent = 2.0`,
truncated to a whole percent. Both constants are **proposed**. One effect to know about, by arithmetic
on the old code: the old guard dropped every command under about 1.19 rad/s, and commands from 0.05 up
to about 1.19 rad/s now get a 2 % duty. Checked: `check_motor_plugin_compiles.sh` passes on the edited
file (syntax and types against hand-copied stubs). Not checked: a build against real WiringPi, that it
runs, that the wheel stops at a zero command, that the direction pin does not chatter near zero, the
deadband and offset values. Robot procedure: the
[journal](../journal/2026-10-31-sim-plant-and-critical-fixes.md). The rest of the file's findings
(C-1 for the encoders, L-1) are not touched.

---

### H-3 — IMU sample is three separate I²C reads

**Where** `rocko_env/rocko_env/hardware/sensors/icm20948/ICM20948.py:37-39`

*Line numbers are at audit time (before the 2026-10-31 edits).*

**What**

`self.icm.euler` is accessed three times. Each property access on the Adafruit driver triggers a
fresh I²C register read, so yaw, roll and pitch are sampled at three different instants and do not
form a coherent orientation.

Pitch — the only axis the balance loop depends on — is the *last* of the three, so it is the most
delayed.

**Fix**

Read the Euler register block (0x1A–0x1F) once as a single burst and decode all three axes from
that one sample. This falls out naturally when C-1 moves the read into C++.

**Status on `ai-forward` (2026-10-31):** partly done. `ICM20948.py` now reads `self.icm.euler` once
and decodes all three axes from that tuple, so the three angles come from one access (next-steps row
0.1, which added this alongside C-2). It is still the Adafruit driver's own read, not a single burst
of registers `0x1A`–`0x1F` in C++, and nobody has checked on a robot what it does to the timing. The
rest waits for C-1.

---

### H-4 — No tipover cutoff

**What**

Nothing detects that balance has already been lost.

**Why it matters**

Once the robot is past recovery, continuing to drive the motors makes it thrash on the floor. A
robot that goes limp reads to onlookers as "it stopped"; a robot that thrashes reads as "it
malfunctioned." At a wedding, that distinction is most of the outcome.

**Fix**

If `|pitch|` exceeds ~35°, zero the motor output and latch a disarmed state until explicitly
re-armed. Re-arming should be a deliberate operator action, not automatic.

---

### H-5 — No real-time scheduling or CPU isolation

**What**

`controller_manager` runs as an ordinary `SCHED_OTHER` process, competing with the Python nodes,
the two WiringPi software-PWM spinner threads, and — once the Pi is broadcasting its own access
point — the wifi driver's softirq work.

**Fix**

1. Add `isolcpus=3` to the kernel command line and pin `controller_manager` to that core.
2. Set `controller_manager`'s `thread_priority` parameter for SCHED_FIFO, and add a matching
   `rtprio` entry to `/etc/security/limits.conf`.
3. Set the CPU governor to `performance`.
4. Confirm the Pi is not thermally throttling; a reception hall is warmer than a workbench.

Verify with a histogram of actual loop periods, not an average. The tail is what tips the robot.

---

### H-6 — Gyro angular rate is never read

**Where** `rocko_env/rocko_env/hardware/sensors/icm20948/ICM20948.cpp:44-61` — the plugin exports
`yaw`, `pitch` and `roll` and nothing else.

**What**

Only fused Euler angles are available to the controllers. The pitch PID's derivative term therefore
has no choice but to numerically differentiate a fused angle.

**Why it matters**

That angle carries the BNO055 fusion filter's group delay — tens of milliseconds. Differentiating a
delayed signal yields a more delayed one, and a derivative term fed by sufficiently delayed feedback
stops damping the system and starts destabilising it.

The sensor already measures angular rate directly, at registers `0x14`–`0x19`, with essentially no
filter delay. That signal is exactly what the D term wants and it is currently being discarded.

**Fix**

Export pitch rate as an additional state interface and configure the pitch controller to consume it
as the measured derivative instead of differentiating the angle. Determine empirically which gyro
axis corresponds to robot pitch, and verify the sign — an inverted D term is worse than none.

This is likely the largest single control improvement available without changing any hardware. See
[`bno055-improvements.md`](bno055-improvements.md) § 1.

---

### H-7 — The motors keep their last command when the cascade stops

**Found** 2026-10-31, while writing the C-2 fix. **Read-only analysis; not confirmed on hardware.**
Severity was Medium/High when found; raised to Critical in the table above after the 2026-10-31 audit. Nobody has watched it on a stand.

**Where** `rocko_env/rocko_env/hardware/motors/Motor12VoltQuadEncoder.cpp` (`write()`, `_wheel.cmd`);
`rocko_env/rocko_env/controllers/add_feedforward/AddFeedforwardController.cpp` (`on_deactivate`)

**What**

The IMU and the motors are separate hardware components. When the IMU plugin returns
`hardware_interface::return_type::ERROR` (C-2), or the controller chain drops out for any other
reason, nothing deactivates the motor plugin. Its `write()` keeps writing the stale `_wheel.cmd`.
As found, `AddFeedforwardController::on_deactivate` only reset its buffer; it did not command zero.
With the `pwmVal > 2` guard from H-2 (a zero command is never written, so the previous duty cycle
stays latched) the wheels may keep turning at the last command. (The mitigations since added are in
the status paragraph below.)

**Why it matters**

C-2 stops the controllers being fed a believable lie. It does not, by itself, make the robot stop,
and a robot that carries on driving with nobody commanding it is the thrashing failure that H-4
describes. This is the other half of "what happens after the error". It has not been observed.

**Fix direction**

A defined, commented stop-or-hold behaviour on cascade loss: say which one (zero the output, or hold
and flag), where it lives, and test it. It is related to H-4 (the tipover cutoff) and H-2 (the latch),
and to the failsafe question in C-3. *Adding* a safeguard is allowed, but a choice of limits goes in a
proposal. Best check once built: Tier 3, then a person on a stand. Queue:
[next-steps row 1.7](../next-steps.md).

**Severity raised** in review round 1 (2026-10-31): the `realtime-auditor` rates this Critical. After
an IMU `ERROR` both motors may still be driven by the last command, and the gamepad still steers a
robot that can no longer sense its tilt. Still read-only analysis; nobody has watched it.

**Status on `ai-forward` (2026-10-31, after review round 2): partly mitigated, still unconfirmed on
hardware, row 1.7 stays open.** In code, none of it run on a robot:

1. `AddFeedforwardController`, in chained mode, sets the reference back to NaN after reading it each
   cycle, so a dead upstream controller leaves NaN and not a repeated old value.
2. A NaN reference repeats the last finite command for at most `kMaxStaleReferenceCycles = 3` cycles
   (30 ms nominal at 100 Hz, **proposed**) and then writes `0.0`. The short hold stops one late
   upstream write from producing a one-cycle zero pulse (a review round 2 finding). The cost is that the
   wheels can keep driving for about that long after the upstream controller stops (not measured).
3. A non-finite (inf) command is refused and written as `0.0`.
4. `on_deactivate` writes `0.0` to every command interface.
5. The motor plugin's `write()` now writes the speed pin every cycle (see
   [H-2](#h-2--near-zero-commands-never-write-latching-the-previous-pwm-value)), so that `0.0` reaches
   the PWM as 0 % duty. Review round 2's High finding was that, before this, writing `0.0` on a fault
   path flipped the direction pin (the old code treated `0.0` as forward) while the old duty stayed
   latched.

Checked: the Tier 1 build with no compiler warnings, Tier 2 (the controller loads and activates), and
the compile-only check of the motor plugin. Not checked: the NaN path, the hold, `on_deactivate` or the
motor `write()` on any tier, and anything on a robot; no automated test of the controller's stop logic
exists (a gtest was declined in round 2; the Tier 3 fault scenarios are meant to cover it, and Tier 3 is
not built). What `controller_manager` does when a hardware component's `read()` or a controller's
`update()` returns `ERROR` depends on the ros2_control version; what the author remembers of Jazzy 4.x
is from memory, not from a test, so it is not recorded here as fact. The H-4 tipover cutoff is not
written.

---

### H-8 — An intermittent IMU fault never tripped the consecutive counter

**Found** 2026-10-31, in review round 1 of the C-2 change (the first version of the fix).

**Where** `rocko_env/rocko_env/hardware/sensors/icm20948/include/rocko_env/ImuFaultTracker.hpp`;
`ICM20948.cpp` `read()`

**What**

The first version of the C-2 fix counted only failed reads *in a row* and returned `ERROR` at five.
A sensor that alternates, for example four bad reads and one good one, repeating, resets that run
each time and never reaches five, yet is bad on 80 % of its reads. The robot would balance on an
angle that is mostly a held old value, with no `ERROR` ever raised.

**Fix**

Count failures in a sliding window as well. `ImuFaultTracker` now trips on either limit:
`kMaxWindowFailures = 10` failed reads among the last `kWindowReads = 20` (200 ms at 100 Hz), or the
consecutive limit. The window limit is a **proposed** value that a human confirms.

**Status on `ai-forward` (2026-10-31): fixed in the tracker, awaiting robot** (part of next-steps row
0.1). Checked: `tools/imu_fault_tracker_test.cpp` feeds 4 bad plus 1 good reads ten times, asserts that
the longest run was 4 (the consecutive limit was never reached), that the tracker still tripped, and
that 16 of the last 20 reads were failures. It went red when the window trip was disabled, when `>=`
was changed to `>`, and when a good read stopped clearing the run. Not checked: the plugin, the sensor
or a robot.

---

### H-9 — A frozen or reset BNO055 is undetectable

**Found** 2026-10-31, in review round 1. **Not fixed.**

**Where** `ICM20948.py` and `ICM20948.cpp` `read()`; the whole read path.

**What**

The fault detector sees failed reads and out-of-range values. It cannot see a sensor that answers
every read with a valid-looking number that is wrong. Two cases: a BNO055 that has frozen and keeps
returning its last orientation, and one that has reset (for example in a brownout) and come back in
configuration mode, where the output can read as a constant `pitch_offset` lean. Either looks like a
healthy, steady robot to the code.

**Why it matters**

This is the C-2 failure again by another route: the controller is told something believable that is
not true. Nobody has seen it happen on ROCKO.

**Fix direction**

Needs the read in C++ (C-1), so the plugin can see the BNO055's own status and calibration registers
and can check that successive samples change at all. Queue: [next-steps row 1.8](../next-steps.md).

**Partly covered, one variant (review round 2, 2026-10-31):** `ICM20948.py` now rejects a reply whose
three Euler values are all equal, which would catch an all-zero block from a reset or a bus fault. A
frozen sensor that keeps returning its last real orientation, and any garbage whose three values
differ, are still not detected. Not seen on a sensor; no tier runs the node.

---

### M-1 — Logging from inside the real-time loop

**Where (line numbers at audit time, before the 2026-10-31 edit)** `rocko_env/rocko_env/hardware/motors/Motor12VoltQuadEncoder.cpp:223-224`

`RCLCPP_INFO` is called unconditionally for the right wheel on every `write()` — string formatting
plus I/O, 100 times per second, in the hot path. Remove it, or replace it with
`RCLCPP_INFO_THROTTLE`. For real telemetry, publish from a `realtime_tools::RealtimePublisher` or
record with `ros2 bag`.

**Status on `ai-forward` (2026-10-31): the per-cycle `RCLCPP_INFO` in `write()` is removed** (pulled
forward with H-2). Compile-checked only (the compile-only check passes; Tier 1 leaves this file out).
Nothing was added in its place, so there is no telemetry of the duty; read it with an oscilloscope or
logic analyser on the speed pin.

---

### M-2 — PWM period equals the control period

**Where** `rocko_env/rocko_env/hardware/motors/Motor12VoltQuadEncoder.cpp:162`

*Line numbers are at audit time (before the 2026-10-31 edits).*

`softPwmCreate(_speedPin, 0, 100)` — WiringPi software PWM uses 100 µs ticks, so a range of 100
gives a 10 ms period, i.e. 100 Hz, identical to the control rate. Consequences: beat frequencies
between the two, ~1% torque resolution, audible whine, and two userspace threads spinning in
competition with the loop.

Note that GPIO 14 (left speed pin) is not a hardware-PWM-capable pin on the Pi, so moving to
hardware PWM requires moving a wire. GPIO 13 (right speed pin) is capable.

*Correction (2026-10-31): the pin numbers in the audit are WiringPi numbers, not BCM GPIO numbers
(`Motor12VoltQuadEncoder.cpp` calls `wiringPiSetup()`, not `wiringPiSetupGpio()`). Whether pin 13 is
hardware-PWM capable must be re-checked against `gpio readall` on the Pi.*

---

### M-3 — PWM value is never clamped

`pwmVal` can exceed 100 whenever `|radPerSec| > MAX_RAD_PER_SEC`. Clamp explicitly rather than
relying on WiringPi's internal handling.

**Status on `ai-forward` (2026-10-31): clamped in code, compile-checked only, awaiting robot.** Duty is
`std::clamp`ed to [0, 100] percent before the cast to `int`. Not checked: on a robot, or on a scope.
A note outside this finding: `MAX_RAD_PER_SEC = 1150 / 60 * M_PI * 2` divides two `int` literals, so it
evaluates to 19 x 2 pi = 119.38 rad/s and not 120.43 rad/s (about 0.9 % lower; by arithmetic, not
checked on a robot). Not fixed; now finding [L-7](#l-7--max_rad_per_sec-divides-integers).

---

### M-4 — `use_mock_hardware` was dead (partly fixed on `ai-forward`)

**Where (at audit time, 2026-09-20)** `rocko_env/rocko_env/description/ros2_control/rocko.ros2_control.xacro:4`

At audit time the flag was plumbed from all three launch files through the xacro args and into the
macro signature, and the macro body never referenced it. Combined with the unconditional WiringPi
link in `rocko_env/CMakeLists.txt:75-78`, `rocko_env` could not be built or run anywhere except on
the Pi. See the status paragraph below for what has changed on `ai-forward`.

**Why it matters**

At audit time every iteration required booting the robot. This is the single largest tax on
development speed, and it means none of the failsafes above can be tested without risking the
hardware.

**Fix**

1. Guard the Pi-only sources behind a CMake conditional so the package builds off-Pi.
2. Make `use_mock_hardware` select an alternative plugin in the xacro.
3. Rather than `mock_components/GenericSystem` (which just echoes commands back as state), write a
   `rocko_env/SimPlant` hardware plugin that integrates inverted-pendulum dynamics and exports the
   same `pitch` / `velocity` / `position` state interfaces as the real hardware.

A cart-pole plant model is on the order of 150–200 lines of C++ and is far more useful here than a
full Gazebo setup: the gains will not transfer exactly, but it exercises the entire real control
stack — chained controllers, sign conventions, units, the watchdog, arm/disarm, the tipover cutoff
— on a laptop, with no hardware. That is where the wedding-day risk actually lives.

**Status on `ai-forward` (2026-10-27):** fix items 1 and 2 are done; item 3 is not.

- Item 1: `rocko_env` now builds in the container with `-DROCKO_SKIP_WIRINGPI=ON`. This leaves out
  `Motor12VoltQuadEncoder.cpp`, so that file is still **not compiled** off a Pi. The Pi default is
  unchanged and still requires WiringPi.
- Item 2: `use_mock_hardware:=true` selects `mock_components/GenericSystem`. As this finding
  predicted, it only echoes commands and holds pitch at 0.0, so it checks controller *wiring* and
  nothing about balance. Verified by `verify_mock.sh`.
- Item 3 (`SimPlant`) is the missing piece and is the recommended Tier 3. It conflicts with
  [ADR 0003](../decisions/0003-simulation-target.md), which chose Gazebo; see the `bootstrap-sim`
  skill for the trade.

---

### M-5 — A failed command write is silently dropped

**Where** `rocko_env/rocko_env/controllers/add_feedforward/AddFeedforwardController.cpp:150`

*Line numbers are at audit time (before the 2026-10-31 edits).*

`command_interfaces_[i].set_value(...)` returns `bool`, and in Jazzy the method is `[[nodiscard]]`.
The controller discards it, so if the write fails the cycle carries on as if the command went out.
The compiler said so during the first off-Pi build:
`warning: ignoring return value of ‘bool hardware_interface::LoanedCommandInterface::set_value…’`.

**Why it matters**

A dropped command leaves the previous command latched on the motor — the same family of failure as
H-2, and a silent fallback of the kind the critical-path rules forbid. How often `set_value` returns
`false` in practice is **not measured**; this is a latent defect found by reading a warning, not an
observed one.

**Fix**

Check the result. On `false`, return `controller_interface::return_type::ERROR` (or hold the last
good command *and* raise a fault flag something acts on) rather than continuing as if it worked.
Count failures and report them with a throttled log, never one per cycle.

**Status on `ai-forward` (2026-10-31):** fixed in code, awaiting a robot (next-steps row 0.3).
`update_and_write_commands` checks the `bool` from `set_value()`, counts failures in
`failed_command_writes_`, still tries every interface so one failure does not leave the other wheel's
command stale, logs a throttled error (every 1000 ms at most), and returns `ERROR` if any write
failed. Checked: the Tier 1 build no longer prints the `ignoring return value` warning (the build log
was read), and Tier 2 passes. Not checked: the failure path itself, which the mock cannot provoke.
What the robot does after that `ERROR` is the open question in H-7.

**Changed after review round 1 (2026-10-31):** the controller no longer returns `ERROR` on the first
failed write. A single transient failure would otherwise permanently stop one wheel, so it returns
`ERROR` only after `kMaxConsecutiveWriteFailures = 5` consecutive failed cycles (a **proposed** value
that a human confirms). Every interface is still tried each cycle, and a comment that described this
wrongly was corrected. What `controller_manager` does after an `update()` returns `ERROR` depends on
the ros2_control version and was not observed.

**Changed after review round 2 (2026-10-31):** the controller now always writes a value on every
interface (a NaN reference no longer means "skip"); a non-finite command is refused and written as
`0.0`; and `on_deactivate` writes `0.0`. Details and the unchecked parts are under
[H-7](#h-7--the-motors-keep-their-last-command-when-the-cascade-stops). The Tier 1 build has no
compiler warnings. The failure path is still not exercised by any tier.

---

### M-6 — The feedforward value was uninitialised and unsynchronised

**Found** 2026-10-31, by the `realtime-auditor` in review round 1 of the M-5 change.

**Where** `rocko_env/rocko_env/controllers/add_feedforward/include/rocko_env/AddFeedforwardController.hpp`
(`_feedforward`), `AddFeedforwardController.cpp` (`update_and_write_commands`)

**What**

`_feedforward` was a plain `double` with no initial value. The topic callback wrote it from the
executor thread and the 100 Hz update read it, with no synchronisation. Before the first message
arrived the update read an indeterminate value, and the two threads touched it at the same time
afterwards. Both are undefined behaviour in C++, and the first could add garbage to every wheel
command.

**Fix**

Make it `std::atomic<double>` initialised to `0.0`, with a `static_assert` that it is lock-free (the
update thread must not take a lock a non-real-time thread also takes). A feedforward message that is
NaN or infinite is ignored and the previous finite value stays in effect.

**Status on `ai-forward` (2026-10-31): fixed.** Checked: the Tier 1 build compiles it, including the
`static_assert`, with no compiler warnings; Tier 2 loads and activates the controller. Not checked:
the lock-free property on a Pi (only the arm64 container was tried), and any behaviour on a robot.

---

### L-1 — `joint_name_` at namespace scope

**Where** `rocko_env/rocko_env/hardware/motors/Motor12VoltQuadEncoder.cpp:21, 99`

*Line numbers are at audit time (before the 2026-10-31 edits).*

Declared at namespace scope rather than as a class member, so both motor instances write to the
same variable. It is currently unused beyond the assignment (`_prefix` is the member actually
used), so there is no live bug — but it is a latent aliasing trap. Delete it or make it a member.

---

### L-2 — IMU class is named for a part that is no longer used

`ICM20948.py` imports `adafruit_icm20x`, `numpy`, `ahrs.filters.Madgwick`, `acc2q` and `Quaternion`,
and uses none of them — it instantiates `adafruit_bno055.BNO055_I2C` (git history: "changed to
bno055"). The C++ plugin, the pluginlib export, the service type and the xacro all still say
ICM20948.

Worth renaming while the file is being rewritten for C-1 anyway, so the next person reading
`rocko.ros2_control.xacro` is not misled about which sensor is on the robot.

Note also that the BNO055's onboard fusion runs at 100 Hz, so sampling faster than 100 Hz yields no
new information. Raising the control rate meaningfully would require switching to raw gyro and
accelerometer output with a filter running on the Pi.

---

### L-3 — Warnings from the first off-Pi build

*Line numbers are at audit time (before the 2026-10-31 edits).*

Seen in the container build on 2026-10-27; none affects behavior today.

- `AddFeedforwardController.hpp:25` includes `realtime_tools/realtime_buffer.h`, deprecated in
  Jazzy in favor of `realtime_buffer.hpp`. Rename the include.
- `rocko_env/rocko_env/controllers/utils/RateLimiter.py` has two docstrings (starting at lines 7
  and 39) containing `\param` tags. Python 3.12 reports each as
  `SyntaxWarning: invalid escape sequence '\p'`, and a later Python will make that an error. Make
  them raw strings (`r"""`).

**Status on `ai-forward` (2026-10-31):** fixed (next-steps row 0.4). The include is now
`realtime_buffer.hpp` and both docstrings are `r"""`. Checked: the Tier 1 build has no warnings from
either, and `python3 -W error -m py_compile` on the host raises no `SyntaxWarning`.

---

### L-4 — The RViz configs are never installed

**Where** `rocko_env/CMakeLists.txt` (the `install(DIRECTORY ...)` blocks);
`rocko_env/rocko_env/bringup/launch/rocko_tank_mode.launch.py:82`, `rocko_gui.launch.py:78`,
`rocko_env/rocko_env/description/launch/view_robot.launch.py:82`

`rocko_env/rocko_env/description/rviz/` holds `rocko.rviz` and `rocko_view.rviz`, but no
`install(DIRECTORY ...)` block lists it, so `install/rocko_env/share/rocko_env/rviz/` does not exist
(checked in the container build, 2026-10-27). The three launch files above look for their config
there. The README presents `ros2 launch rocko_env view_robot.launch.py description_package:=rocko_env`
as working; it cannot find `rocko_view.rviz`. Found by the ros2-control review of the `ai-forward`
change, not caused by it. Not fixed there, because a fix deserves its own goal and verification.

**Fix**

Add `rocko_env/description/rviz` to the description `install(DIRECTORY ...)` block, then confirm
the three launch files start RViz with their configs (needs a display, so a person or a GUI-capable
container).

**Status on `ai-forward` (2026-10-31):** fixed (next-steps row 0.5). `rocko_env/rocko_env/description/rviz`
is in the description `install(DIRECTORY ...)` block. Checked: after the build, `ls
install/rocko_env/share/rocko_env/rviz/` shows `rocko.rviz` and `rocko_view.rviz`. Not checked: that
RViz starts with them, which needs a display. The `view_robot.launch.py` default argument
([#11](https://github.com/BiPed-Capstone/ROCKO-env/issues/11)) was fixed at the same time (row 0.6):
`description_package` now defaults to `"rocko_env"`. That launch file has not been run.

---

### L-5 — D-term spike after held cycles

**Found** 2026-10-31, in review round 1 of the C-2 change. **Not fixed.** Read-only analysis,
unmeasured.

**Where** `ICM20948.cpp` `read()` (the hold) and the pitch PID's D term (H-6)

**What**

After a failed read the plugin holds the last good angle. The pitch controller differentiates that
angle (H-6), so during the held cycles the derivative sees no change, and on the first good read the
whole accumulated change arrives in one step, which can show up as a spike in the D term.

**Why it matters**

A short, large kick to the wheels just after a sensor glitch, at the moment the robot is least
well-known. How large depends on the gains, which are not touched here.

**Fix direction**

Taking the D term from the gyro rate (H-6) removes the differentiation of the held angle. Until then,
treat a glitch followed by a kick as this. Queue: [next-steps row 1.9](../next-steps.md).

---

### L-6 — A failed encoder read is reported as a good one

**Found** 2026-10-31. **Not fixed.** Read-only analysis; nobody has watched it fail.

**Where** `Motor12VoltQuadEncoder::read` (`rocko_env/rocko_env/hardware/motors/Motor12VoltQuadEncoder.cpp`)
and `QuadEncoder::read` (`rocko_env/rocko_env/hardware/sensors/quadEncoder/QuadEncoder.cpp`)

**What**

`Motor12VoltQuadEncoder::read` and `QuadEncoder::read` return OK and keep the old position and
velocity when the encoder call fails. There is no fault flag, and the error log line is unthrottled.

**Why it matters**

It is hidden today because the blocking call (C-1) has no timeout: a stalled encoder node stalls the
whole loop instead of failing. It becomes a C-2-class bug (a failure that looks like a good reading)
the moment C-1 adds a timeout, because the timeout branch will then land in this silent path.

**Fix direction**

Use the same pattern as `ICM20948::read` (a fault tracker, return `ERROR`, a throttled log) in the C-1
encoder change. Queue: [next-steps row 1.10](../next-steps.md).

---

### L-7 — `MAX_RAD_PER_SEC` divides integers

**Found** 2026-10-31 (first noted under M-3). **Not fixed.**

**Where** `rocko_env/rocko_env/hardware/motors/Motor12VoltQuadEncoder.cpp` (`MAX_RAD_PER_SEC`)

**What**

`1150 / 60 * M_PI * 2` divides two `int` literals, so it evaluates to 19 x 2 pi = 119.38 rad/s and
not 120.43 rad/s. This is by arithmetic; it has not been checked on a robot.

**Why it matters**

Fixing it lowers every duty cycle by about 0.9 % (the full-scale speed it divides by gets larger), so
it changes the scaling the existing tuning was done against. That is a hardware-limit change a human
confirms, so it is a proposal and not an agent edit. Queue: [next-steps row 1.11](../next-steps.md).

---

## Recommended order of work

Ordered by risk removed per unit of effort.

1. **C-2** — stop reporting 0.0 on IMU failure. Smallest change on this list, removes a direct
   fall-over cause.
2. **C-1 (IMU) + UART migration** — move the sensor read into `ICM20948.cpp` and move the sensor to
   UART at the same time; delete the service and the Python node. Doing both together avoids writing
   the I²C read twice.
3. **H-6** — export gyro pitch rate and switch the D term onto it. Cheap, and large. Do it while the
   plugin is already open.
4. **C-1 (encoders)** — same treatment for the encoder reads.
5. **H-2, M-1, M-3** — fix the `write()` path. Small, mechanical, low risk. (Code written ahead of
   this position on `ai-forward`, 2026-10-31, compile-checked only; see H-2.)
6. **M-4** — CMake guard (done on `ai-forward`) plus the `SimPlant` plugin (not done), so the rest can be developed and tested on a
   laptop.
7. **C-3, H-4** — watchdog and tipover cutoff, developed against `SimPlant`.
8. **H-5** — real-time scheduling and core isolation; measure the period histogram before and
   after.
9. **H-1** — close the wheel velocity loop.
10. **Only now**, retune. Expect materially different gains from the current values; the plant will
   have changed.

The IMUPLUS fusion-mode change and the quaternion read from
[`bno055-improvements.md`](bno055-improvements.md) §§ 3–4 are near-free and fold naturally into step 2.

Steps 1–5 should be verified to have reduced the spread of loop period and feedback age before any
tuning time is spent.

---

## What is *not* a problem

Recorded so that effort is not spent here:

- **ROS 2 and `ros2_control` are the right choice.** Nothing in these findings is caused by the
  framework. Chained controllers are a good fit for a cascaded balance controller and the existing
  chain structure is sound.
- **Running everything onboard the Pi is fine.** A 100 Hz loop is well within a Pi's capability
  once the blocking I/O is out of the critical path.
- **The split of concerns is good.** Balance-critical work (IMU → pitch PID → motor) lives inside
  the `controller_manager` loop; non-critical work (joystick, diff-drive mixing) lives outside as
  Python nodes. That is the correct boundary. The problem is that C-1 accidentally drags Python
  back across it.
- **100 Hz is an appropriate rate**, matched to the BNO055 fusion output. Chasing a higher rate is
  not indicated.

---

## Related hardware work

A move to brushless motors with ODrive controllers is under consideration. If that happens:

- **H-1 and M-2 are obviated** — the velocity loop and commutation move into the ODrive.
- **C-1 is partially obviated** — the encoder service disappears, but the IMU fix is still
  required and is on the critical path either way.
- **Everything in C-2, C-3, H-4, H-5 and M-4 still applies unchanged.**

The fixes above should not be deferred pending a hardware decision. They are independent of it,
and several of them are prerequisites for evaluating it — without a trustworthy loop period and a
trustworthy pitch signal there is no baseline to compare new hardware against.
