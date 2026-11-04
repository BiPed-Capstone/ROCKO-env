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
| C-2 | Critical | IMU read failure reports pitch = 0.0 ("upright") — **fixed on `ai-forward`, awaiting robot** (2026-10-31); a frozen sensor is still undetected (H-9); the sim's copy of the rule is run end to end by check C2 (2026-11-02) |
| C-3 | Critical | No teleop watchdog, and no defined comms-loss failsafe — **fixed on `ai-forward`, awaiting robot** (2026-11-02; checked in the sim; the 0.2 s timeout is proposed); the response is a step to zero velocity, never a motor stop |
| H-1 | High | Motor command is open loop; no wheel velocity feedback |
| H-2 | High | Near-zero commands never write, latching the previous PWM value — **code fixed on `ai-forward`, compile-checked only, awaiting robot** (2026-10-31; pulled forward of the planned order) |
| H-3 | High | IMU sample is three separate I²C reads at three different instants |
| H-4 | High | No tipover cutoff — **written on `ai-forward`, awaiting robot** (2026-11-02; checked in the sim; the 35 deg limit and the 10 deg re-arm band are proposed); see H-10, C-1 and the known item under H-4 |
| H-5 | High | No real-time scheduling or CPU isolation for `controller_manager` |
| H-6 | High | Gyro angular rate is never read; the D term differentiates a delayed fused angle — **partly fixed on `ai-forward`** (2026-11-02): the rate is exported and wired in the sim only; the real YAML change is gated; axis and sign unverified |
| H-7 | Critical (raised from Medium/High by the `realtime-auditor`; unconfirmed on hardware) | The motors may keep their last command when the cascade stops or the IMU plugin returns ERROR (found 2026-10-31) — **partly mitigated on `ai-forward`** (controller stop logic and the motor `write()` fix, none run on a robot); the sim showed what `controller_manager` does after an `ERROR`, for one component (2026-11-02) |
| H-8 | High | An intermittent IMU fault that alternates good and bad reads never tripped the consecutive-only counter (found 2026-10-31) — **fixed in the tracker on `ai-forward`, awaiting robot** |
| H-9 | High (not fixed) | A frozen or reset BNO055 that still reads valid-looking numbers is undetectable (found 2026-10-31); one variant, an all-equal Euler triple, is now rejected in the node |
| H-10 | High (not fixed; unmeasured) | Upstream PID integrators keep winding while the tipover cutoff is latched, so a re-arm can produce a large command (found 2026-11-02) |
| H-11 | High | Nothing aged out the feedforward: if `DiffDriveController.py` died mid-turn, the wheels kept opposite biases and the gamepad could not stop them (found 2026-11-03; called H-A in the round-3 review and in code comments) — **fixed on `ai-forward`, awaiting robot** (2026-11-03; checked in the sim; the 200 ms limit is proposed) |
| H-12 | High (shown in the model; unknown on the robot) | With every gain positive, the outer velocity loop is positive feedback unless the encoder reads negative when a positive wheel command drives the robot; under the sim's default signs it runs away and tips the model in about 10 s (found 2026-11-03, stage 5) — **not fixed**: the robot sign check is [proposal 0006](../proposals/0006-velocity-loop-sign-check.md) |
| M-1 | Medium | `RCLCPP_INFO` called every cycle from inside `write()` — **removed on `ai-forward`, compile-checked only** (2026-10-31) |
| M-2 | Medium | Software PWM period equals the control period (both 100 Hz) |
| M-3 | Medium | PWM value is never clamped to its valid range — **clamped on `ai-forward`, compile-checked only, awaiting robot** (2026-10-31) |
| M-4 | Medium | `use_mock_hardware` is dead; the project cannot be built or tested off the Pi — **partly fixed on `ai-forward`** (wiring mock and off-Pi build exist; a `SimPlant` physics model exists with estimated parameters; its balance scenarios exist since 2026-11-03; they fail with the hardware gains and pass with sim gains tuned against the model, stage 5c-1) |
| M-5 | Medium | `AddFeedforwardController` ignores the result of `set_value()`, so a failed command write is silently dropped — **fixed on `ai-forward`, awaiting robot** (2026-10-31) |
| M-6 | Medium | `AddFeedforwardController`'s feedforward value was uninitialised and shared between two threads without synchronisation (found 2026-10-31) — **fixed on `ai-forward`** |
| M-7 | Medium | The IMU node read the gyro (a second I2C transaction) inside the blocking service call every cycle although no real controller uses the rate (found 2026-11-03; called M-A in the round-3 review) — **fixed on `ai-forward`, awaiting robot**: `read_pitch_rate`, default false; one gap left |
| M-8 | Medium | Two unthrottled INFO lines per gamepad message in `DiffDriveController.py` (found 2026-11-03; called M-B in the round-3 review) — **fixed on `ai-forward`** (the lines were removed) |
| L-1 | Low | `joint_name_` is at namespace scope and shared between both motor instances |
| L-2 | Low | IMU class is named for a part that is no longer used; dead imports |
| L-3 | Low | Deprecated header include and invalid escape sequences, seen as warnings in the first off-Pi build — **fixed on `ai-forward`** (2026-10-31) |
| L-4 | Low | The RViz configs are never installed, so three launch files cannot find them — **fixed on `ai-forward`** (2026-10-31); RViz opening with them not checked |
| L-5 | Low | A D-term spike after cycles in which the IMU angle was held (found 2026-10-31, not fixed) |
| L-6 | Low now, Critical once C-1 adds a timeout | `Motor12VoltQuadEncoder::read` and `QuadEncoder::read` return OK and keep the old position and velocity when the encoder call fails (found 2026-10-31, not fixed) |
| L-7 | Low | `MAX_RAD_PER_SEC` is computed with integer division (found 2026-10-31, not fixed; a human confirms the change) |
| L-8 | Low (matters when H-6 reaches the real YAML) | `pid_controller` sets the rate reference once, at activation, and never again (found 2026-11-02, not fixed) |

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
tier runs the node. It also would reject a genuine reading with three identical values. (This rule is for
the Euler angles only: the gyro rate added for H-6 has no such rule, because a gyro that is truly still
reads (0, 0, 0), which is a real measurement.)

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

**Downstream, in the sim (check C2, 2026-11-02, `SimPlant`, `controller_manager` 4.48.1).** `SimPlant`
applies the same counting rule to a simulated IMU (a copy of the decision, not `ICM20948.cpp`). Starting
at 2 deg, so a true pitch of exactly 0.0 is impossible: a 0.03 s outage (3 reads) gave no `ERROR`, and no
exported pitch sample was 0.0 or NaN (a held value is a run of 5 identical samples); a 0.2 s outage
(20 reads) made `read()` return `ERROR`, and `controller_manager` logged `Deactivating following
hardware components as their read cycle resulted in an error ... [ SimRocko ]`, deactivated all five
controllers (`joint_state_broadcaster` included), and logged `Not acceptable command interfaces
combination` and `Error while attempting mode switch when deactivating controllers in read cycle!`.
End state: controllers `inactive`, `SimRocko` `unconfigured`; 0 samples of 0.0 or NaN. The check went
red (4 zero samples) when the failed read was made to export 0.0. **This does not show what the robot's
motors do**: there the IMU is its own component (see [H-7](#h-7--the-motors-keep-their-last-command-when-the-cascade-stops)),
so which controllers `controller_manager` deactivates after an IMU `ERROR`, and whether the motor
plugin keeps driving, remains unobserved.

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

**Status on `ai-forward` (2026-11-02): fixed in code, checked in the sim, awaiting robot.** Next-steps row
2.5. `DiffDriveController.py` owns the age check: `CommandWatchdog` measures from the moment a command
**arrives**, on `time.monotonic`. Past `command_timeout_s` (**0.2 s, proposed; a human sets it**; 20 control cycles of 10 ms, which is 4 x the
laptop's `HEARTBEAT_PERIOD_S` of 0.05 s) it zeroes the velocity reference and the feedforwards, **keeps publishing**, and the balance loop keeps running;
nothing is deactivated and no motor output is zeroed. It publishes a `Bool` on
`/diff_drive_controller/command_timed_out` (`transient_local`) and logs `teleop command timed out: zeroing the
velocity reference, balance loop keeps running` (only after a command had been received; at start-up it logs
`no teleop command yet: the velocity reference is zero` instead) and `teleop commands resumed`. Two upstream changes were
needed for the age to mean anything. `Joystick.py` used to re-publish the last packet every 10 ms, which
made an old command look fresh; it now publishes only on a real packet, and survives a clean disconnect
(it used to crash) and a reconnect. And the laptop sender `joystick/computer_joystick.py` sent only on a
change, so a stick held steady would have looked like a dead link; it now resends every 0.05 s and pads or
cuts the gamepad's data to 6 axes and 16 buttons (the 40-byte packet is unchanged for a gamepad of that
size). **The laptop's copy must be updated**; it has no reconnect code, so restart it after a drop.
`Joystick.py` also rejects a message with a non-finite or out-of-range axis (beyond 1 by more than 0.01) or a
button that is not 0 or 1, and drops the connection, so a garbled stream becomes silence and the watchdog
zeroes the reference. `DiffDriveController`'s callback also stored
`msg.twist`, which has no `.twist`, so its first command would have raised `AttributeError`; fixed.

Checked, in the model: check `C3` of `verify_sim.sh` (a fake gamepad at 20 Hz for 2 s, then silence):
`command_timed_out` went `False` 0.014 s after the first command and `True` 0.207 s after the last, inside
the 0.2 to 0.3 s window; the reference was `[0.05, 0.05]` in 195 of 195 messages while driven and exactly
`[0, 0]` in 170 of 170 afterwards; the controllers stayed active; 170 plant samples arrived after it.
It went red with `command_timeout_s=1000`. The check also requires `teleop commands resumed` before the first
`teleop command timed out` in the log. Host tests `tools/teleop_watchdog_test.py` (16/16; red on 4
deliberate breaks when it had 13 checks) and `tools/computer_joystick_heartbeat_test.py` (5/5); whether the
later checks were made to fail was not recorded. Not checked: the TCP path in a
launch, anything on a robot, and what the step to zero does to a leaning robot (the reference goes to 0 in
one cycle with no ramp; `RateLimiter` is unused). Procedure: the
[stage 4 journal](../journal/2026-10-31-sim-plant-and-critical-fixes.md).

**The node's own death (2026-11-03):** this watchdog lives inside `DiffDriveController.py`, so it cannot
act if that node dies or hangs; the last feedforwards would have stayed in force. [H-11](#h-11--nothing-ages-out-the-feedforward)
adds an age limit in `AddFeedforwardController` (200 ms, proposed) that zeroes the feedforward term in that
case. Checked in the model only (check `FF` kills a plain `ros2 topic pub`, not `DiffDriveController.py`
itself).

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
rest waits for C-1. The gyro rate added for H-6 is a second read after the angles, not part of that
snapshot; one 12-byte burst of registers `0x14`–`0x1F` (gyro and Euler) would give both in one sample and
is a note for the C-1 IMU migration (stage 6), not implemented.

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

**Status on `ai-forward` (2026-11-02): written, checked in the sim, awaiting robot.** Next-steps row 2.6.
**Why a cutoff:** the reason is physical. Past the lean from which the robot can still recover, the wheels
cannot get back under the body, so driving them only spins them or shoves a robot that is already down. The
35 deg comes from this finding's write-up above, not from this robot's physics or a measurement. The cutoff
is the **opposite** of C-3's rule on purpose: it zeroes the wheels only **after** the robot has fallen; while
it is upright, losing the gamepad zeroes only the velocity **reference** ([C-3](#c-3--no-teleop-watchdog-and-no-defined-comms-loss-failsafe)).
Each of `left_` and `right_add_feedforward_controller` now claims `<joint>/pitch` and trips when
`|pitch| > tipover_pitch_deg` (**35.0 deg, proposed; a human confirms**; 0 or less disables it with a
WARN; above 90 and NaN are refused; read-only). Past the limit it writes `0.0` every cycle and **does not
deactivate** anything; it never trips on NaN (pitch starts as NaN). It re-arms only by **one** message,
`ros2 topic pub --once -w 2 /tipover_rearm std_msgs/msg/Empty {}`, which both wheels subscribe to (`-w 2`
because plain `--once` re-arms only one wheel: discovery of the second subscriber takes about 3 s), **and**
only while `|pitch|` is at or below `rearm_max_pitch_deg` (**10 deg, proposed**, read-only; hysteresis, so
a robot is re-armed only after a person has stood it up; configure fails unless 0 < it < the trip limit).
Activating the controller also re-arms it, without the 10 deg check. Each wheel's instance latches on its
own; activation fails if the pitch interface is missing. The decision is in `TipoverLatch.hpp`. The limit
applies to the **reported** pitch, which includes the IMU node's `pitch_offset = 7.6` deg (origin
undocumented), so the true lean at the trip is not known to better than that.

Checked, in the model: check `H4` (start at 40 deg): both controllers logged `TIPOVER CUTOFF TRIPPED` in the
first cycle they wrote; all 801 of 801 commands written while `|pitch| > 35` were exactly `0.0` (read from
`controller_manager`'s `/controller_manager/introspection_data/full`, best-effort QoS); the controllers stayed
active (numbers from the first run). The check now sends one `/tipover_rearm` message while the robot is still
past the limit and requires both controllers to refuse it and neither to log `RE-ARMED`; it passed that way
(counts not recorded). Red with the cutoff off (`tipover_pitch_deg` 0): -400 rad/s,
548 of 548 commands non-zero, no trip. Host test `tools/tipover_latch_test.cpp`: 12 test functions, red on 2
deliberate breaks when it had 10 (the re-arm band cases were not shown to fail). **Risks, not fixed:**
[H-10](#h-10--upstream-pid-integrators-wind-up-while-the-cutoff-is-latched);
a frozen IMU ([H-9](#h-9--a-frozen-or-reset-bno055-is-undetectable)) cannot trip it; and
[C-1](#c-1--unbounded-blocking-service-calls-inside-the-real-time-loop) blocks the cutoff entirely, because pitch
reaches it only through the blocking `read()`. Not checked: anything on a Pi; an accepted re-arm in the sim;
the 10 to 35 deg re-arm band in the sim (host test only). Procedure (tethered, wheels off the ground, lean
past 35 deg by hand, then try the re-arm inside and outside the band): the
[stage 4 journal](../journal/2026-10-31-sim-plant-and-critical-fixes.md).

**Known item (2026-11-02): the tipover block is in `AddFeedforwardController.cpp` twice.** The inline block
(about lines 299 to 352) is what runs; an unused duplicate, the helper `tipover_tripped_this_cycle()` (about
line 445), exists because the guard hook denied the edit that would have replaced the block with a call (the
hook counts safety words and cannot tell a move from a removal). A human removes the duplicate; a proposal is
to be written. Separately, the three tipover keys are only code defaults in
`add_feedforward_controller_parameters.yaml`, which the hook does not watch, and `rocko_controllers.yaml` is
untouched; the same proposal asks to add them to the hook's key list and to the real YAML. Next-steps row 1.14.

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

**Status on `ai-forward` (2026-11-02): partly fixed.** Next-steps row 1.2. *Done in code:*
`Icm20948Data.srv` has `pitch_rate_deg_s`; `ICM20948.py` reads `.gyro` once per reply, **only when its `read_pitch_rate` parameter is true (default false since
2026-11-03, finding [M-7](#m-7--the-imu-node-read-the-gyro-every-cycle-for-nothing); with it false the rate is NaN and
the real D term is unaffected)**, and exports
`pitch_rate_sign * degrees(gyro[pitch_rate_axis])`, with `pitch_rate_axis` (0) and `pitch_rate_sign` (+1) as named
parameters that are **unverified guesses**. The unit is a guess too: the BNO055 counts 1 LSB = 1/16 deg/s, the
Adafruit driver is documented to return rad/s, and versions older than 5.0 returned deg/s, which would make
the rate 57 times too large; the robot procedure must check the magnitude. `success` covers the angles
only: a gyro-only failure makes `pitch_rate_deg_s` NaN and never discards a good pitch, and there is no
'identical values' rule for the gyro (all zero is a real reading; that rule stays for the Euler angles).
The plugin exports `pitch_rate` [deg/s] on both joints and judges the rate **on its own**: a rate that is not
finite or beyond 2000 deg/s (**proposed**) is held (the last good one, NaN before the first) and counted in a
separate counter, and never makes `read()` return `ERROR`; only the angles feed the fault tracker. The
sim's copy differs: it ties the rate to the pitch, so a bad rate is a failed read of the whole sample there,
and no sim check exercises the plugin's separate rule. The rate is a second sensor read after the angles,
not one snapshot; one 12-byte burst read (registers `0x14`–`0x1F`) belongs to the C-1 IMU migration (stage
6) and is not implemented. Host test `tools/imu_node_test.py` (6/6) covers the node's reply with a stand-in
sensor; the real, mock and sim `<ros2_control>` blocks all export it (14
interfaces now); `SimPlant` exports `pitch_sign * degrees(psi_dot)` with no IMU delay; and the **sim**
YAML (only) sets `pitch_pid_controller`'s `reference_and_state_interfaces` to `["pitch", "pitch_rate"]`. That
is the one wiring difference `sim/check_sim_config.py` allows. *Not done:* the real `rocko_controllers.yaml`
still says `["pitch"]`, so the robot's D term still differentiates the angle; changing it is a gated
decision (a proposal, not yet written) and has to cover [L-8](#l-8--pid_controller-sets-the-rate-reference-once-at-activation).
It must also cover a **persistent** rate failure: today a bad rate is only held and logged, so once the real
YAML consumes the rate the plugin needs its own consecutive-failure tracker that makes `read()` return `ERROR`,
switched by a hardware parameter such as `pitch_rate_required` (see the comment in `ICM20948.hpp`).

How `pid_controller` 4.42.1 uses a second interface (read from its source at that tag by the integrator,
2026-11-02): it is the measured derivative of the first; the D term is `d * (rate reference - measured
rate)`; the names are arbitrary; and a NaN rate makes it fall back, silently, to differentiating the angle.

Checked, in the model: check `H6`: over 600 pairs from the same cycle `feedback_dot` equalled the exported
`pitch_rate` exactly (largest 46 deg/s) and `error_dot` was `-feedback_dot`; red with `["pitch"]`. That shows
the wiring, not a benefit: the sim's rate has no delay and an assumed sign. **Observation, model only,
unchanged hardware gains, 2 deg start, zero reference:** with H-6 the first 2 s settle faster (pitch 0.43 /
0.31 deg at 1 s, 0.26 / 0.04 deg at 2 s) but the slow drift tips the model sooner (10.1 deg at 8 s, 23.3 deg at
9 s, first at or past 35 deg at 9.42 / 9.34 s) than without it (1.59 deg at 1 s, 5.9 deg at 8 s, 34.4 deg at
10 s, first at or past 35 deg at 10.02 s). Neither holds. It is a slow run-away, consistent with the outer
velocity loop's sign problem flagged in [`sim-plant.md` §2.4](../architecture/sim-plant.md), but **not
shown and not yet explained** (stage 5). It is not evidence that the gyro D term is good or bad. Robot
procedure for the axis and sign: the [stage 4 journal](../journal/2026-10-31-sim-plant-and-critical-fixes.md).

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
exists (a gtest was declined in round 2; the Tier 3 fault scenarios are meant to cover it, and they do not
exist yet: Tier 3 is only partly built). What `controller_manager` does when a hardware component's `read()` or a controller's
`update()` returns `ERROR` depends on the ros2_control version; what the author remembers of Jazzy 4.x
is from memory, not from a test, so it is not recorded here as fact. The H-4 tipover cutoff is not
written.

**Update 2026-11-02 (stage 4).** The H-4 tipover cutoff is now written (it is a separate thing: it
zeroes the wheel command on a lean). And the sim answered one half of "what does `controller_manager` do
after an `ERROR`", for one component: in `SimPlant`, where the IMU and the motors are one component, an
`ERROR` from `read()` made `controller_manager` 4.48.1 deactivate that component and all five controllers
(check C2 long; details under [C-2](#c-2--imu-read-failure-reports-pitch--00)). It does **not** show
what the robot's separate motor component does, so H-7 stays *partly mitigated, unconfirmed on hardware*.

On the robot an IMU `read()` `ERROR` should also deactivate both feedforward controllers, because they claim
`<wheel>/pitch` and `controller_manager` deactivates the controllers that use a deactivated component's
interfaces. **Unverified**, and it differs from the sim, where the plant is one component that owns the wheel
commands as well.

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
three Euler values are all equal, which would catch an all-zero block from a reset or a bus fault (this rule is for the Euler angles, not the gyro rate). A
frozen sensor that keeps returning its last real orientation, and any garbage whose three values
differ, are still not detected. Not seen on a sensor; no tier runs the node.

**Related (2026-11-02):** the H-4 tipover cutoff cannot see a frozen sensor that holds a safe-looking pitch.

---

### H-10 — Upstream PID integrators wind up while the cutoff is latched

**Found** 2026-11-02, while writing the H-4 cutoff. **Not fixed. Not measured**: this is from reading the
cascade, and the sim check `H4` exercises only a refused re-arm.

**Where** `AddFeedforwardController.cpp` (`update_and_write_commands()`, the tipover branch) and the two PIDs
upstream of it in `rocko_controllers.yaml` (`pitch_pid_controller`, `velocity_pid_controller`).

**What**

When the cutoff trips, the last controller in the chain writes `0.0`. Nothing upstream is told:
`pitch_pid_controller` and `velocity_pid_controller` stay active and keep computing, and while the robot
lies on the floor (or in someone's hands) nothing makes their error shrink, so an integral term can keep growing
until it reaches its clamp (`i_clamp_max` / `i_clamp_min`). (The integral gains are currently 5.0 for the
velocity loop and 0.0 for the pitch loop in `rocko_controllers.yaml`, so the wind-up is today the velocity
loop's, and its clamp is +/-150, in degrees of target pitch, past the cutoff angle itself.)

**Why it matters**

A re-arm puts the wheel command back under the loops' control at once. The first command after it can
be whatever the wound-up integrators say, which could be a large lurch at the moment a person has just
set the robot upright. The integrators are not reset by a trip or a re-arm.

**Fix direction**

Reset or hold the upstream integrators on a trip (or on a re-arm), for example by re-activating the
chain before the re-arm message (reported safer by the integrator; not tested), or by conditional
integration while disarmed. Until it is fixed the robot procedure for a re-arm asks for the stick centred and
for the cascade to be deactivated and reactivated (stage 4 journal, H-4 procedure step 9; the exact commands
are not written, and activating an `*_add_feedforward_controller` itself re-arms it without the 10 deg check). Either changes control behaviour, so it needs the sim scenarios of
next-steps row 2.3 to check and a person to check on a stand. Queue: [next-steps row 1.12](../next-steps.md).

---

### H-11 — Nothing ages out the feedforward

**Found** 2026-11-03, by the `realtime-auditor` in review round 3 of stage 4 (named H-A there; the code
comments now say H-11).

**Where** `rocko_env/rocko_env/controllers/add_feedforward/AddFeedforwardController.cpp`
(`update_and_write_commands`), `include/rocko_env/AddFeedforwardController.hpp`,
`include/rocko_env/FeedforwardTimeout.hpp` (new)

**What**

`DiffDriveController.py` publishes each wheel's feedforward [rad/s] every 10 ms and
`AddFeedforwardController` added the LAST value it had received to the wheel command, however old. If that
node died during a turn, nothing replaced the value: the two wheels kept opposite biases and the gamepad
could not stop them. C-3's watchdog cannot help, because it lives inside the node that died. Not seen on a
robot; plausible (a Python node crashing is what H-8-style faults and C-1's blocking calls make likely).

**Fix**

`FeedforwardTimeout` (header only, no ROS types) stores the value and its arrival time [ns] on a steady
(monotonic) clock, both lock-free atomics. Each cycle `update()` returns the value while it is at most
`kMaxFeedforwardAgeNs` old (200 ms, **proposed**, equal to `command_timeout_s`) and 0.0 after that. Only the
**feedforward term** goes to 0.0: the reference from `pitch_pid_controller` still goes through, so the
balance loop keeps holding the robot up (the same rule as C-3; zeroing the whole command would drop the
robot). One `WARN` per wheel when a time-out starts, `Feedforward on '<topic>' is N ms old ... (K
time-outs since start)`, at most one line per second per wheel (`kTimeoutLogMinCycles`, 100 cycles; a
time-out inside that second is counted, not logged); a fresh message ends it at once. A message the callback ignores for being NaN or infinite does not refresh the
arrival time. Nothing arrived yet counts as 0.0 and not as a time-out (the earlier behaviour).

**Status on `ai-forward` (2026-11-03): fixed in code, checked in the sim, awaiting robot.** Next-steps row
1.15. Checked: host test `tools/feedforward_timeout_test.cpp` (7 test functions, red on 2 deliberate
breaks); check `FF` of `verify_sim.sh` (a script publishes on both wheels at 100 Hz for 4 s, then is
killed: both controllers logged the time-out once, 0.1 to 0.5 s later, and stayed active; red with the
limit infinite and with 50 ms); `sim/check_sim_config.py` compares `FF_AGE_S` with `kMaxFeedforwardAgeNs`;
full `verify_sim.sh` PASS (`S1 S2a H4 C3 C2 H6 FF`, 9 launches, 157 s). **Observation, cause not
established:** in the container the right wheel sometimes timed out while its Python publisher was
running, so a message gap of more than 200 ms happened (1 of 1 runs at 20 Hz, 2 of 6 `FF` runs at 100
Hz). Check `FF` prints this as a note and does not fail on it. If the same gaps happen on the robot, the
feedforward would drop out for a moment and come back; nobody has measured that. Not checked: anything on
a robot, that 200 ms is a good limit, DDS delivery on the Pi, and the zeroing step's effect on a leaning
robot (it is a step, no ramp). Related and **unconfirmed**: `pid_controller` may also hold its last
reference forever if the node that sets it dies. Journal: *Stage 4, review round 4*.

---

### H-12 — The outer velocity loop is positive feedback unless the encoder's sign is the "wrong" one

**Found** 2026-11-03, stage 5, by a linear analysis of the simulated cascade
(`sim/cascade_linearisation.py`; [stage 5 journal](../journal/2026-11-03-stage5-balance-in-sim.md)). The
sign question itself was written down in [sim-plant.md §2.4](../architecture/sim-plant.md) on 2026-10-31;
this finding shows it is what tips the model. **Not fixed. Shown in the model only**: whether the robot
has it depends on encoder wiring nobody has checked.

**Where** `velocity_pid_controller` in `rocko_controllers.yaml` (lines 55-56: `p: 2.0, i: 5.0, d: 0.2`,
all positive) together with the sign of the velocity that `QuadEncoders.py` reports, relative to the
direction a positive wheel command drives the robot. No file is wrong on its own; the fault is in how
their signs combine.

**What**

`velocity_pid_controller` computes a target pitch from `reference − measured velocity` with positive
gains, and `pitch_pid_controller` leans the robot to that target. Leaning makes the wheels drive the
robot. For the loop to correct a speed error, the measured velocity has to grow in the direction that
**reduces** the error. sim-plant.md §2.4 works it through: with every gain positive (and a working pitch
loop), that holds only if the encoder reads **negative** when the robot rolls the way a **positive**
command drives it. The authors evidently assumed the opposite (`DiffDriveController.py` sends `linear.x`
as both the velocity reference and a positive wheel feedforward). Under their assumption the P and I terms
both push the robot away from the reference: the loop is positive feedback at DC, for any positive gains.

In the model (estimated parameters, sim default signs `pitch_sign = −1`, `encoder_sign = +1`, the
hardware gains): a real closed-loop pole at **+0.82 /s** (discrete, as the sim runs; +0.85 /s in the
closed-form reduction). Scaling the velocity gains up moves it further right (+2.07 /s at 8x), the
signature of positive feedback. The simulated robot drifts off and falls 9.3 to 10 s after release from a
2 deg start, as stages 3 and 4 saw. With the other encoder sign the run-away at DC goes, but the model
still does not hold with the hardware gains: the pitch loop's ~3.5 Hz oscillation is marginal at the
estimated 20 ms IMU delay (a gain margin of about −1.1 dB) and the velocity loop's D term takes damping
from it. That second part is about gain values and the unmeasured delay, not about a sign.

**Why it matters**

If the robot has the authors' intended encoder sign, its outer loop cannot hold a speed: it drifts and
falls after some seconds whatever the velocity gains, and tuning them only changes how fast. That would
fit "balancing is partially tuned and not reliable" (this page's header), but it is **not evidence** for
it: a robot that falls for any of the other reasons on this page looks the same.

**Fix direction**

First the check, then one sign change. A human runs the two-step check of sim-plant.md §2.4 (tank mode on
a stand, then a hand tilt of the IMU; about 15 minutes, the robot does not need to balance). If the
encoder reads positive for a positive command, flip one sign: negate the three `velocity_pid_controller`
gains in `rocko_controllers.yaml`, **or** negate the velocity `QuadEncoders.py` reports. A human chooses
which ([proposal 0006](../proposals/0006-velocity-loop-sign-check.md) lists the trade-off). Then the gains
themselves are a separate question for the sim scenarios (stage 5) and a tethered test. Until the check is
done, the sim's balance scenarios run with `encoder_sign:=-1` as a labelled assumption, and the scenario
`AB` keeps the as-built signs and is expected to fall, so this finding stays visible on the ladder
(`verify_sim.sh scenarios`). Queue: [next-steps row 2.7](../next-steps.md).

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
- Item 3 (`SimPlant`) was the missing piece. **Update 2026-10-31 (stage 3): the plugin now exists as a
  model, and it is not a finished Tier 3.** `rocko_env/SimPlant` integrates planar inverted-pendulum
  dynamics with a DC-motor model and exports the same 14 interfaces as the real hardware (12 until stage 4 added `pitch_rate`); the real
  cascade loads against it unchanged (`rocko_sim.launch.py`, `rocko_sim_controllers.yaml`), and
  `verify_sim.sh` checks that pitch responds to the wheels and that the robot falls with no torque.
  The recommendation above said "on the order of 150–200 lines"; the model header alone is 484 lines
  and the plugin 349 (`wc -l`, 2026-11-02), because the plant also copies the motor `write()` duty map and the encoder
  node's speed halving, models the IMU delay and injects faults. **Every physical parameter is an
  estimate** (nothing was measured), the upright, impulse and velocity-step scenarios and the
  one-sign-flip check do not exist, and the sim stops the whole plant on an IMU `ERROR` where the
  robot would not (H-7), so M-4 stays *partly done*. See the
  [verification ladder](../architecture/verification-ladder.md) for the tier's status and
  [`sim-plant.md`](../architecture/sim-plant.md) for the model. It still conflicts with
  [ADR 0003](../decisions/0003-simulation-target.md), which chose Gazebo and stays `Proposed`; a
  proposal to amend it is to follow.

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

### M-7 — The IMU node read the gyro every cycle for nothing

**Found** 2026-11-03, by the `realtime-auditor` in review round 3 of stage 4 (M-A there).

**Where** `rocko_env/rocko_env/hardware/sensors/icm20948/ICM20948.py` (`callback()`),
`ICM20948.cpp` (`read()`)

**What**

H-6 added a gyro read to the IMU node. That is a second I2C transaction inside the blocking service call
of every 10 ms cycle (C-1), and the real `rocko_controllers.yaml` does not use `pitch_rate`, so on the robot
it cost loop time for nothing.

**Fix**

A node parameter `read_pitch_rate`, default `false`. While false the gyro is never read and
`pitch_rate_deg_s` is NaN ("no rate"); the plugin holds NaN and does not warn, because that is by design.
The plugin warns about an unusable rate only after it has seen one good rate.

**Status on `ai-forward` (2026-11-03): fixed in code, awaiting robot.** Checked: host test
`tools/imu_node_test.py` (7 of 7, including: started with no parameters, one sensor read per reply, NaN
rate, no warning; and a rate beyond +/-2000 deg/s sent as NaN and logged). The C++ change is checked by the
Tier 1 build only; `verify_sim.sh` uses `SimPlant`, not this node. **Who reports a rate that is bad from
the very first read** (with the rate on): the node, not the plugin. The node logs `Pitch rate unavailable`
(once a second at most) both for a failed gyro read and for a rate beyond +/-2000 deg/s, because it applies
the same limit as the plugin (`MAX_ABS_PITCH_RATE_DEG_S` in `ICM20948.py`, which must equal
`kMaxAbsPitchRateDegS` in `ICM20948.hpp`) and sends NaN for it. The plugin stays silent until it has seen
one good rate. **To use the rate:** BOTH the gated change to
`rocko_controllers.yaml` (finding H-6, with L-8) AND launching `ICM20948.py` with `read_pitch_rate:=true`.
Only the first is a proposal-gated edit; forgetting the second leaves the D term with NaN.

---

### M-8 — Two INFO lines per gamepad message

**Found** 2026-11-03, review round 3 (M-B there). `DiffDriveController.robot_body_vector_updated` logged
`Callback triggered` and the whole twist on every message, 40 to 200 lines a second once the node no longer
crashed on its first command.

**Status on `ai-forward` (2026-11-03): fixed.** Both lines were removed. Checked only by reading the diff and
by `teleop_watchdog_test.py` still passing.

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

**Update 2026-11-02:** the plugin now exports the gyro rate, and the sim YAML uses it; the real YAML does not yet (H-6), so this is still open on the robot.

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

### L-8 — `pid_controller` sets the rate reference once, at activation

**Found** 2026-11-02, reading `pid_controller` 4.42.1 (upstream source at that tag, read by the integrator) for
the H-6 change. **Not fixed.** It is invisible in the sim and unobserved on a robot.

**Where** `pitch_pid_controller` in `rocko_sim_controllers.yaml` (and in `rocko_controllers.yaml`, once H-6 is
wired there).

**What**

With `reference_and_state_interfaces: ["pitch", "pitch_rate"]`, the D term is `d * (rate reference - measured
rate)`. The rate **reference** is set only once, when the controller activates, to the measured rate
(`set_current_state_as_first_setpoint`, on by default), and nothing writes it afterwards. In the sim that
value is exactly 0.0, because `SimPlant` holds the robot still until the controllers command the wheels.
On the robot it would be whatever the gyro read at activation: a constant bias of `d * r0` in the D term for the
rest of the run; and if there was no good IMU read yet, NaN, and then `pid_controller` silently falls back
to differentiating the angle, which is the behaviour H-6 exists to remove.

**Why it matters**

A proposal that wires `pitch_rate` into the real YAML has to say how the reference is set (to zero on
activation, or kept written by something), and show that the first IMU read is good before the controller
activates. Queue: [next-steps row 1.13](../next-steps.md).

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
6. **M-4** — CMake guard (done on `ai-forward`) plus the `SimPlant` plugin (a model with estimated parameters exists, 2026-10-31; balance scenarios since 2026-11-03, failing with the hardware gains and passing with sim gains tuned against the model), so the rest can be developed and tested on a
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
