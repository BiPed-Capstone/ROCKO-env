# `SimPlant`: the plant model for Tier 3

**What this is:** the specification that `rocko_env/SimPlant` implements and that
`tools/sim_plant_model_test.cpp` checks against §7.1. `SimPlant` is a `ros2_control` hardware plugin
(a `SystemInterface`) that simulates ROCKO's physics, so the real controllers can run against
something that falls over.

**Code this page specifies:**

- `rocko_env/rocko_env/hardware/sim/include/rocko_env/SimPlantModel.hpp`: the physics (§3) and the
  parameters (§4, §5), plain C++17 with no ROS types. Tested on the host by
  `tools/sim_plant_model_test.cpp` against the numbers in §7.1.
- `rocko_env/rocko_env/hardware/sim/SimPlant.cpp` and `include/rocko_env/SimPlant.hpp`: the
  `rocko_env/SimPlant` `SystemInterface` plugin that wraps it, registered in `rocko_env_hardware.xml`
  and always built (it needs no WiringPi).
- `use_sim_plant:=true` in `rocko.urdf.xacro` selects it; every parameter of §4 and §5 is an xacro arg
  there. `rocko_sim.launch.py` starts it with `rocko_sim_controllers.yaml`; `verify_sim.sh` and
  `sim/sim_probe.py` check criteria S1 and S2a against it, and `sim/scenario_checks.py` checks the
  stage 4 safeguards (tipover cutoff, teleop watchdog, IMU outage, pitch rate) against it. `sim/check_sim_config.py` (run by
  `verify_urdf.sh`) fails if the sim controller YAML's wiring or the defaults of the simulator
  settings drift apart.

Whether Tier 3 counts as built is recorded only in the [verification ladder](verification-ladder.md),
which (since 2026-11-07) says it is built **with limits**: this plant has ESTIMATED, unmeasured
parameters, `encoder_sign:=-1` is ASSUMED, and a pass says nothing about whether the robot balances.
[ADR 0003](../decisions/0003-simulation-target.md) (still `Proposed`) chose Gazebo; the 2026-10-31 run
([journal](../journal/2026-10-31-sim-plant-and-critical-fixes.md)) builds `SimPlant` first, as finding
**M-4** recommends. The proposal to amend the ADR is
[proposal 0005](../proposals/0005-amend-adr-0003-simulpant-first.md) (Open; row 2.1 of
[next-steps.md](../next-steps.md)).

**Code this page reads:** `bringup/config/rocko_controllers.yaml`, `DiffDriveController.py`,
`Joystick.py`, `Motor12VoltQuadEncoder.cpp`/`.hpp`, `QuadEncoders.py`, `QuadEncoder.py`,
`ICM20948.py`/`.cpp`/`.hpp`, `AddFeedforwardController.cpp`, `rocko.ros2_control.xacro` and
`rocko_description.urdf.xacro`, in the `ai-forward` tree of 2026-10-31, which includes the finding
**C-2** changes to the IMU files and the new `Motor12VoltQuadEncoder::write()` (findings H-2, M-1,
M-3: deadband, stiction offset, clamp). Those files, which the C-2 and `write()` changes edited, are
cited by function or constant name. The `file:line` citations that remain are to files not edited
since (the Python nodes, YAML and xacro); they will drift when those files change.

---

> **Read this first: what this plant is, and what it is not**
>
> - **It is** a planar inverted pendulum on wheels (one body, one wheel pair, forward/back motion and
>   pitch only) driven by a DC-motor model. It exports **the same 14 interfaces as the real hardware**:
>   for each of `left_wheel_joint` and `right_wheel_joint`, one command (`velocity`) and six states
>   (`position`, `velocity`, `yaw`, `pitch`, `roll`, `pitch_rate`). The real controllers load against it unchanged;
>   only the hardware layer is swapped. (`rocko_sim.launch.py` loads `rocko_sim_controllers.yaml`,
>   which starts as a copy of `rocko_controllers.yaml`, gains included, with one named wiring difference: the
>   sim's pitch PID also reads `pitch_rate` (§2.2, finding H-6). Sim gains live only there.)
> - **Every physical number in it is an estimate. Nothing has been measured.** Masses, centre-of-mass
>   height, inertia, motor constants, IMU delay: all invented or inferred, and labelled so in §4.
> - **Gains found against it are candidates, not results.** A gain set that holds the simulated robot
>   up says nothing about the real one. Promoting a gain to the robot is a proposal and a tethered test
>   by a human ([`tune-gains`](../../.claude/skills/tune-gains/SKILL.md)).
> - **It is not** Gazebo, not a contact simulation, and not a model of turning (yaw), of the floor, or
>   of anything in §6.
> - Its most useful job is the one finding M-4 names: exercise the **real** control stack (units, signs,
>   chaining, safeguards) on a laptop. §2 shows that job is already paying off.

---

## 1. Words used on this page

- **Plant:** the thing being controlled, here the robot's body and wheels.
- **Cascade:** the two nested PID loops; see [control-cascade.md](control-cascade.md).
- **Wheel command `u`:** the number `pitch_pid_controller` plus the feedforward write into
  `<wheel>_joint/velocity` (the command interface). Unit in §2.
- **Sim-forward:** **by definition, the direction a positive wheel command drives the robot.** The sim
  is built around this one definition. Whether sim-forward is the robot's front or back is unknown
  (§2.4) and does not matter to the physics.
- **Lean `ψ` (psi):** the body's true tilt from vertical, in radians, **positive when the body leans
  toward sim-forward**. This is the physics variable. It is *not* the exported `pitch` state; §3.7 says
  how one becomes the other.
- **Chassis-relative wheel angle:** the angle the motor shaft turns relative to the robot's frame. That
  is what an encoder on the motor measures, and it differs from the wheel's angle to the floor whenever
  the body tilts.

---

## 2. Units and signs of the real cascade (the audit)

Before modelling anything, this section works out from the code what each number in the real cascade
*means*: its unit, and which way is positive. The sim must hand the controllers numbers with the same
meaning, or it tests a different robot.

### 2.1 Every signal in the cascade

| Signal | Interface or topic | Unit on the real robot | Sign | Where it comes from |
| --- | --- | --- | --- | --- |
| Pitch state | `left_wheel_joint/pitch`, `right_wheel_joint/pitch` (the same number on both) | **degrees** | Unknown from code (§2.4) | `ICM20948.py`'s `callback()` reads the BNO055 `euler` tuple once (heading, roll, pitch), in degrees per the Adafruit library, and stores `euler[2] + self.pitch_offset`; `pitch_offset = 7.6` is added to a degree value, so it is **+7.6°**. `ICM20948.hpp` (comment on `_pitch`) also says degrees. `ICM20948::export_state_interfaces()` exports the one `_pitch` under both joints |
| Pitch "zero" | — | degrees | — | 0.0 is whatever raw BNO055 pitch reads −7.6°. Presumably the team found that to be upright; **unmeasured**. Nothing says 0.0 is "centre of mass over the axle" |
| Wheel velocity state | `<wheel>_joint/velocity` (state) | **m/s**, not rad/s | Unknown from code (§2.4) | `Motor12VoltQuadEncoder::read()` stores the service reply in `_wheel.vel`. The reply is `QuadEncoders.py:102`: `np.average` of the two wheels' filtered velocities (`:80-90`), each `Δposition / 0.01` (`:71`, `:73`), with position in metres (`:70`: counts ÷ `meters_conversion`, `:35`, which uses radius 0.06 m and belt 1.355). **See finding U-2: the value is about half the true speed** |
| Wheel position state | `<wheel>_joint/position` | **m** of **left** wheel travel, on both joints | Same as velocity | `QuadEncoders.py:101` returns `left_position` only; `Motor12VoltQuadEncoder::read()` stores it in `_wheel.pos` |
| Wheel command | `<wheel>_joint/velocity` (command) | **rad/s**, read as a fraction of the gearbox-output no-load speed | Positive = sim-forward, by definition | `Motor12VoltQuadEncoder::write()`: `radPerSec = _wheel.cmd`; non-finite or `abs(radPerSec) < 0.05` (`kDeadbandRadPerSec` in the `.hpp`) writes duty 0; else duty % = `clamp(abs(radPerSec) / MAX_RAD_PER_SEC * 100 + 2.0, 0, 100)` (`kStictionDutyPercent` in the `.hpp`), truncated to `int`. `MAX_RAD_PER_SEC` (the `.hpp`) is "1150 RPM" at the gearbox output (finding U-3) |
| Direction pin | GPIO | — | `cmd >= 0` writes `!invert`, else `invert`; left unchanged for a non-finite or deadband command | `Motor12VoltQuadEncoder::write()`, the two `digitalWrite(_dirPin, ...)` calls. Left `invert=true` (`rocko.ros2_control.xacro`, `LeftDriveMotor` block), right `invert=false` (`RightDriveMotor` block) |
| Velocity reference | `velocity_pid_controller/reference` (topic) | "m/s" by intent | Positive = stick forward, probably | `DiffDriveController.command_controller()` sends `linear_vel` (the `linear.x` it received) for both wheels; `Joystick.py` multiplies the stick by `linear_scale = -10.0` (set in `__init__`, applied where the `TwistStamped` is built), so the range is ±10 "m/s" |
| Feedforward | `left_feedforward`, `right_feedforward` (topics) | **rad/s of the wheel** | Same sign as `linear.x` (± turning) | `DiffDriveController.command_controller()`: `left_vel = linear_vel - angular_vel`, `right_vel = linear_vel + angular_vel`, each divided by `wheel_radius_meters` (0.06). `AddFeedforwardController::update_and_write_commands()` adds it to the pitch loop's output: `u = pitch_pid output + feedforward`, **while the feedforward's last message is at most 200 ms old** (`kMaxFeedforwardAgeNs`, proposed, finding H-11); after that it adds 0.0. The sim's publisher is `DiffDriveController.py` or a script, as on the robot |
| Pitch rate state | `<wheel>_joint/pitch_rate` (the same number on both) | **deg/s**, meant to be positive while pitch increases | Depends on `pitch_rate_axis` and `pitch_rate_sign` in `ICM20948.py`, both **unverified guesses** (finding H-6) | `ICM20948.py`'s `callback()` reads `.gyro` once, **only if its `read_pitch_rate` parameter is true (default false, finding M-7; then `pitch_rate_deg_s` is NaN and no gyro read happens)**, and answers `pitch_rate_sign * degrees(gyro[pitch_rate_axis])` as `pitch_rate_deg_s`; `ICM20948::read()` stores it in `_pitch_rate`, which `export_state_interfaces()` exports under both joints. The comment in `ICM20948.py` says the Adafruit driver returns rad/s and that versions before 5.0 returned deg/s; which one is installed was not checked, so the unit conversion is also unverified. **No controller in `rocko_controllers.yaml` reads it**; the sim's `rocko_sim_controllers.yaml` does (§2.2) |
| Yaw, roll states | `<wheel>_joint/yaw`, `/roll` | degrees | — | Exported (`ICM20948::export_state_interfaces()`); **no controller reads them** (`rocko_controllers.yaml` uses only `pitch` and `velocity`) |

### 2.2 What each PID consumes and produces

Both loops are `pid_controller/PidController` from `ros2_controllers` (4.42.1 in the container, per the
[ladder](verification-ladder.md)). Per that package's implementation (upstream, not re-read for this
page), each cycle it computes `error = reference − state` and
`output = p·error + (integral term) + d·(error − previous error)/dt`. The derivative is of the
**error**, so a jump in the reference also kicks the D term.

| Loop (YAML lines) | Input error | Output | Gains (YAML), with units |
| --- | --- | --- | --- |
| `velocity_pid_controller` (`rocko_controllers.yaml`) | reference "m/s" − measured velocity state (m/s, U-2) | **target pitch, degrees** (it is written into `pitch_pid_controller`'s reference, compared with pitch in degrees) | p = 2.0 °/(m/s); i = 5.0 °/m; d = 0.2 °·s²/m; integral clamp ±150 °, i.e. the integrator alone may ask for 150° of lean, far past the 35° tipover angle of finding H-4 |
| `pitch_pid_controller` (`:20-36`) | target pitch (°) − measured pitch (°) | **wheel command, rad/s** (after the feedforward is added) | p = 10.0 (rad/s)/°, i.e. 573 (rad/s)/rad; i = 0; d = 0.1 (rad/s)/(°/s) |

**The D term with a rate (finding H-6, sim only).** In `rocko_sim_controllers.yaml`,
`pitch_pid_controller` lists `reference_and_state_interfaces: ["pitch", "pitch_rate"]`;
`rocko_controllers.yaml` still lists `["pitch"]`. This is the one wiring difference
`sim/check_sim_config.py` allows. The integrator read `pid_controller` 4.42.1's source at that tag
(2026-11-02; this page's author did not): the second entry is the measured derivative of the first, the
D term becomes `d · (rate reference − measured rate)` instead of a derivative of the error, the
names of the two entries are arbitrary, and if the measured rate is NaN the controller silently falls back
to differentiating the angle. The rate **reference** is set only once, at activation, to the measured rate
(`set_current_state_as_first_setpoint`, on by default), and nothing writes it afterwards: it is 0.0 in
the sim because `SimPlant` holds the robot still until the controllers command the wheels (§3.6), but on
the robot it would be whatever the gyro read at that instant (a constant bias `d · r0` in the D term), or
NaN for good if there was no good IMU read yet (finding [L-8](../claude-analysis/balance-loop-findings.md#l-8--pid_controller-sets-the-rate-reference-once-at-activation)).
Any proposal to change the real YAML has to cover that.

The two loops agree with each other on units (degrees in, degrees out). The pitch loop's output and the
feedforward agree on "rad/s" by name, but not on *which shaft* (U-4).

### 2.3 Unit findings

These are findings about the real code, recorded here because the sim has to reproduce them. None is
fixed by this page. **U-1 to U-6 are defined on this page, not in
[balance-loop-findings.md](../claude-analysis/balance-loop-findings.md)**, whose IDs (C-1, H-3, L-7, ...)
are a different list; where one of these is also a finding there, the entry says so.

- **U-1 · One interface name, two units.** On each wheel joint, the `velocity` **state** is in **m/s**
  (robot speed over the floor, from the encoders) while the `velocity` **command** is in **rad/s**
  (wheel speed). ROS convention for a revolute joint is rad/s for both. The cascade never compares the
  two directly, so this does not break it today; but `joint_state_broadcaster` publishes the state on
  `/joint_states` as if it were rad/s, and anyone who closes a loop between command and state (finding
  H-1's fix) must convert. **What the sim does:** export the velocity state in m/s exactly as the real
  stack does, because the outer loop's reference is in m/s. Changing the unit is a separate fix that
  must change the reference with it.
- **U-2 · Both wheels share one encoder reply, and it reads about half the real speed.** Both motor
  plugins call the same service, `encoder_data` (`Motor12VoltQuadEncoder::on_init()`, which creates `_client`;
  `QuadEncoders.py:58`), once each per 10 ms cycle, one after the other in the same `read()` pass. Each
  call computes `(position − position at the previous call) / 0.01` and moves "previous call" forward
  (`QuadEncoders.py:70-73`, `:98-99`). The first call of a cycle sees about 10 ms of motion, the second
  almost none, but both divide by 0.01 s. The two time gaps add up to one period, so the low-pass
  filter (`alpha = 0.05`, `:39`, `:80-87`) settles at **the average of the two samples, about 0.5 × the
  true speed**. Steady state for a constant true speed `v`: left joint ≈ 0.513 v, right ≈ 0.487 v; time
  constant ≈ 97 ms (the filter runs twice a cycle, so it is faster than the 195 ms a single call would
  give). Derived from the code, **not measured**; it depends only on the two reads landing in the same
  cycle, which they do. Consequence: the velocity loop sees half the speed, so in steady state it
  drives the robot at about **twice** the commanded speed. **What the sim does:** reproduce it by
  default (§3.7), behind a switch for when finding C-1's encoder fix lands.
- **U-3 (= L-7 in the findings doc) · `MAX_RAD_PER_SEC` is 119.38 rad/s, not 120.43.** `MAX_RAD_PER_SEC` in `Motor12VoltQuadEncoder.hpp` writes
  `1150 / 60 * M_PI * 2`. In C++ `1150 / 60` is integer division, = 19, so the constant is
  19 × 2π = 119.381 rad/s rather than 1150 RPM = 120.428 rad/s. A 0.9 % error, harmless by itself.
  **What the sim does:** the write-path replica uses the code's 119.381; the motor physics uses the
  motor's own 120.428 (§4). The 0.9 % mismatch then appears in the sim as it does on the robot.
- **U-4 · The feedforward and the duty map disagree about which shaft "rad/s" is.**
  `DiffDriveController.command_controller()` computes the feedforward as wheel rad/s (`m/s ÷ wheel radius`).
  `write()` divides by the gearbox-output no-load speed. `QuadEncoders.py:35` says a 1.355 belt
  reduction sits between the gearbox output and the wheel. If it does, a feedforward of `x` wheel-rad/s
  actually asks for `x / 1.355` wheel-rad/s. The older `QuadEncoder.py:28` (not launched;
  `rocko.launch.py:128-148` comments it out) has no belt factor, so the repo disagrees with itself
  about whether the belt exists. **What the sim does:** model the belt at 1.355 as an estimate (§4).
- **U-5 · The velocity reference asks for more than the wheels can give.** Full stick is ±10 "m/s"
  (`linear_scale` in `Joystick.py`). The wheels' no-load ground speed is about 5.3 m/s with the belt (7.2 m/s
  without), at 12 V, before any load. A full-stick request therefore saturates the motors and winds up
  the velocity loop's integrator (clamped only at ±150°).
- **U-6 · Geometry disagrees too.** Wheel radius: §4.2. Wheel separation: 0.1 m (`rocko_description.urdf.xacro:49`, `:85`, ±0.05 m), 0.32 m (`rocko_tank_mode_controllers.yaml:15`) and 0.356 m
  (`wheel_distance` in `DiffDriveController.__init__`, declared and never used). The planar sim needs no separation.

### 2.4 Sign conventions: is the cascade negative feedback?

**The question.** Both PIDs have positive gains and use `error = reference − measurement`. For the
cascade to correct errors rather than amplify them, the physical meaning of "positive" has to line up
across three quantities the code never relates to each other: the **wheel command**, the **pitch**,
and the **encoder velocity**.

**Definitions.** Sim-forward is the direction a positive wheel command `u` drives the robot (§1). The
code cannot say which way that is: it depends on the motor driver's wiring (§2.1, direction pin).
Relative to it, two signs are unknown:

- `s_θ` (pitch sign): `+1` if the measured pitch **increases** when the body leans toward sim-forward,
  `−1` if it decreases. Depends on how the BNO055 is mounted and its axis setup; not in the code.
- `s_v` (encoder sign): `+1` if the measured velocity is **positive** when the robot rolls toward
  sim-forward, `−1` if negative. Depends on the encoder wiring; `QuadEncoders.py:27` swaps the right
  encoder's pins so left and right agree *with each other*, but nothing ties them to the command.

**What the physics says** (derived in §3; the signs below hold for any positive masses and lengths).
A positive wheel command pushes the wheels toward sim-forward. The motor's reaction torque on the body
and the base's acceleration both tip the body **away** from sim-forward. A body leaning toward
sim-forward falls further toward sim-forward unless the wheels are driven toward sim-forward under it.

**Inner loop (pitch → wheel command).** With the velocity loop off, `u = Kp·(0 − pitch)`, `Kp > 0`.
Lean the body toward sim-forward by a small angle `ψ > 0`; measured pitch is `s_θ·ψ`, so
`u = −Kp·s_θ·ψ`. Recovering needs `u > 0` (drive the wheels under the lean), so it needs **`s_θ = −1`**.
With `s_θ = +1` the loop drives the wheels away from the lean and the robot falls faster than with no
control at all. A robot-checkable way to say the same thing: **the inner loop is negative feedback if
and only if a positive wheel command makes the measured pitch go up.**

**Outer loop (velocity → target pitch).** This loop works on a slower time scale than the pitch loop.
Assume the inner loop works (`s_θ = −1`, so `u = Kp·θ_ref + Kp'·ψ`), treat it as fast and let the
body's tilt settle quickly (`ψ̈ ≈ 0`, `ψ̇ ≈ 0`); the linearised wheel equations of §3.2 then reduce to
one equation for the wheel angle `φ` (rad, positive rolling toward sim-forward):

```
m_eff · φ̈  =  c · φ̇  −  K_u · Kp · θ_ref            (1)

m_eff = K_u · Kp' · (a11 + a12) / (m_b · g · l)  −  a11     [kg·m²]
```

where `θ_ref` is the target pitch in degrees, `Kp' = Kp·180/π` is the pitch gain per radian, `K_u`
(N·m per rad/s of command) and `c` (N·m·s/rad, motor back-EMF plus friction) are the motor constants of
§3.3, and `a11`, `a12` are the inertia terms of §3.2. For any pitch gain big enough to hold the body up,
`m_eff > 0` (0.078 kg·m² with the §4 estimates). Equation (1) has two consequences:

1. **The pitch loop alone does not hold the robot still.** With `θ_ref = 0`, `φ̇` grows like
   `exp((c/m_eff)·t)`: the robot creeps off and speeds up. The motor's back-EMF, which damps the wheels
   in open loop, appears through the high-gain pitch loop as **negative** damping on robot speed. With
   the §4 estimates, `c/m_eff ≈ 0.31 s⁻¹`, a time constant of about 3 s. The velocity loop has to remove
   this.
2. **What the outer loop needs.** The velocity loop sets `θ_ref = Kpv·(v_ref − v_meas) + (integral) +
   (derivative)` with `v_meas ≈ 0.5·s_v·r·φ̇` (U-2). Put that into (1) with `v_ref = 0` (`Kdv` is left out
   because it only adds a small amount to `m_eff`):

   ```
   m_eff · φ̈  =  (c + s_v·k·Kpv) · φ̇  +  s_v·k·Kiv · φ ,      k = K_u · Kp · 0.5 · r
   ```

   For this to be stable, the `φ̇` term and the `φ` term on the right must both be negative. Both need
   **`s_v = −1`**, and the first also needs `k·Kpv > c`. With `s_v = +1`, the P term and the I term
   **both** push away from the reference: the outer loop is positive feedback, and the robot runs away
   in one direction (a real positive root; about 0.85 s⁻¹ with the §4 estimates and the YAML gains).

**Feedforward.** In (1) the feedforward enters exactly like a target pitch: adding `u_ff` to the wheel
command is the same, on the slow time scale, as adding `u_ff/Kp` degrees to `θ_ref`. Positive
`linear.x` makes both the velocity reference and the feedforward positive, so the **feedforward always
has the same sign as the outer loop**. It is consistent with the outer loop whichever way `s_v` turns
out. (On the fast time scale it first pushes the wheels toward sim-forward, which tips the body the
other way: the "back up to lean forward" move that [control-cascade.md](control-cascade.md) describes.)

**What the code's authors intended, and why it cannot all be true.** `DiffDriveController.command_controller()`
sends `linear.x` as the velocity reference *and*, divided by the radius, as a positive wheel
feedforward. So the authors took sim-forward, positive measured velocity and positive `linear.x` to be
the same direction (`s_v = +1`). The README and control-cascade.md say the velocity loop commands
"lean further forward" to go faster; with `Kpv > 0` that makes positive pitch a lean toward the robot's
front, and with the first assumption, toward sim-forward (`s_θ = +1`). **Under that intent, with all
gains positive, the inner loop is positive feedback.** The robot is reported to balance partly
([findings](../claude-analysis/balance-loop-findings.md), "Status of the robot"), which a
positive-feedback inner loop could not do. So on the robot at least one of the authors' assumptions is
false. The code cannot say which.

> **Verdict: cannot be determined from the code, and the authors' evident intent is inconsistent.**
>
> - Inner loop: negative feedback **only if** a positive wheel command makes the measured pitch go up
>   (`s_θ = −1`). Unknown from code; the partial balancing on the robot suggests it holds there.
> - Outer loop, given a working inner loop: negative feedback **only if** the measured velocity is
>   positive when the robot rolls the way a **negative** command drives it (`s_v = −1`). Unknown from
>   code. The authors' intent (`s_v = +1`) makes the outer loop **positive feedback**: P and I both act
>   the wrong way.
> - Feedforward: always the same sign as the outer loop. Consistent with it in either case.
> - Even with consistent signs, equation (1) says the outer loop's P gain must beat the negative
>   damping the inner loop creates (`k·Kpv > c`). Whether `Kpv = 2.0` does depends on unmeasured
>   numbers; with the §4 estimates it does not.
>
> In one sentence: with every gain positive, the cascade can only work if **positive pitch and positive
> measured velocity both point the way a negative wheel command drives the robot**.

**The motor `invert` parameter and left/right asymmetry.** `invert` (left `true`, right `false`) flips
the direction pin on one side. Two mirror-image motors on a differential drive have to turn opposite
ways (as seen from their shafts) to drive the robot the same way, so opposite `invert` values are what
a correct build needs, given identical driver wiring on both sides. If a side were wrong, the pitch loop
(identical commands to both wheels) would spin the robot in place. Tank mode drives straight, which
suggests the two sides agree with each other; it says nothing about the signs above. Asymmetries that
do exist: the position state is the **left** wheel only on both joints (`QuadEncoders.py:101`), and the
two velocity states differ by about 5 % (U-2). Neither changes a sign. The planar sim drives its one
wheel pair with the sum of the two wheels' torques, so left/right differences in the *commands* only
matter through that sum.

**What the sim picks, and why.** The sim cannot know `s_θ` and `s_v`, so it exposes both as
parameters, `pitch_sign` and `encoder_sign` (§4), and **declares each choice an assumption**:

- `pitch_sign = −1` (default): a positive wheel command makes the exported pitch go up. Basis: it is the
  only value under which the inner loop can work, and the robot reportedly balances partly. If it were
  wrong, nothing else in the sim would be testable.
- `encoder_sign = +1` (default): the encoder reads positive when the robot rolls the way a positive
  command drives it. Basis: the authors' intent read from `DiffDriveController.py`, and the convention
  a builder would expect. **Consequence: with the hardware YAML as it is, the sim's outer loop is
  positive feedback. That is the finding above showing up in the sim, and the sim must show it.**
  Setting `encoder_sign = −1` is allowed only as a labelled assumption ("assumes the encoder reads
  negative when a positive command drives the robot; not checked on the robot"), carried on every
  result obtained that way. Flipping the velocity loop's gains negative in a `*_sim.yaml` encodes the
  same assumption the other way round and needs the same label. A sim that picks whichever sign makes
  the YAML work would pass by construction: finding **C-2**'s mistake, a sensor that always reads
  "fine".

**The robot check that settles both signs.** Neither needs the robot to balance. A human does this;
record it as in [reproducing-results.md](../guides/reproducing-results.md).

1. **`s_v` and sim-forward.** Robot on a stand, both wheels clear of everything. On the Pi:
   `source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash`, then
   `ros2 launch rocko_env rocko_tank_mode.launch.py`. In a second shell (sourced the same way):
   `ros2 topic echo /joint_states --field velocity`. In a third:
   `ros2 topic pub -r 20 /diffbot_base_controller/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.5}}}"`.
   Tank mode turns positive `linear.x` into a positive command on both wheels (about 3.9 rad/s, 5 %
   duty). The direction the wheels turn **is** sim-forward; note it on the robot with tape. Both wheels
   must turn the same way. A positive `velocity` in `/joint_states` means `s_v = +1`, negative means
   `s_v = −1`. Stop with Ctrl-C in the third shell; abort with Ctrl-C in the launch shell if a wheel runs
   away. (Tank mode runs at 50 Hz, where the U-2 halving happens to cancel out; only the sign matters
   here.)
2. **`s_θ`.** Motors off or the launch stopped. On the Pi: `ros2 run rocko_env ICM20948.py`, and in a
   second shell `ros2 service call /icm20948_data rocko_interfaces/srv/Icm20948Data`. Call it with the
   robot held upright, then tilted about 10° toward the taped sim-forward direction. Pitch going **down**
   means `s_θ = −1` (the sim's default), **up** means `s_θ = +1`.

---

## 3. The model

### 3.1 State and frames

The robot is treated as two rigid bodies moving in one vertical plane: the **wheel pair** (both wheels
together, rolling without slipping) and the **body** (everything else: frame, motors, battery, Pi). The
axle is the pivot between them.

| Symbol | Meaning | Unit | Sign |
| --- | --- | --- | --- |
| `φ` (phi) | Wheel angle relative to the **floor** | rad | Positive rolling toward sim-forward |
| `ψ` (psi) | Body lean from vertical | rad | Positive leaning toward sim-forward (the same rotational sense as `φ`) |
| `x = r·φ` | Axle position along the floor | m | Positive toward sim-forward |
| `φ − ψ` | Wheel angle relative to the **body** (what the motor shaft and encoder turn through) | rad | Positive when the wheel turns toward sim-forward relative to the frame |

State vector: `X = [φ, ψ, φ̇, ψ̇]`. Rates in rad/s. `φ` itself never appears in the forces (the floor
looks the same everywhere), so it only matters for the exported position.

### 3.2 Equations of motion

Parameters (values in §4): `r` wheel radius; `M_w` total mass of both wheels; `I_w` total wheel inertia
about the axle; `m_b` body mass; `l` distance from axle to body centre of mass; `I_b` body inertia about
its own centre of mass (pitch axis); `g` gravity.

The body's centre of mass sits at `(x + l·sin ψ, r + l·cos ψ)`. Kinetic energy:

```
T = ½·M_w·ẋ²  +  ½·I_w·φ̇²  +  ½·m_b·(ẋ² + 2·l·cos ψ·ẋ·ψ̇ + l²·ψ̇²)  +  ½·I_b·ψ̇²       with ẋ = r·φ̇
V = m_b·g·l·cos ψ
```

The motor torque `τ_net` acts **between** body and wheel: `+τ_net` on the wheel, `−τ_net` on the body.
Its virtual work is `τ_net·(δφ − δψ)`, so the generalised forces are `Q_φ = +τ_net`, `Q_ψ = −τ_net`.
Lagrange's equations then give:

```
a11·φ̈           + a12·cos ψ·ψ̈  =  τ_net + a12·sin ψ·ψ̇²          (wheel)
a12·cos ψ·φ̈     + a22·ψ̈         = −τ_net + m_b·g·l·sin ψ         (body)

a11 = (M_w + m_b)·r² + I_w        a12 = m_b·r·l        a22 = m_b·l² + I_b        [kg·m²]
```

Solve the 2×2 system for `φ̈` and `ψ̈` each evaluation (determinant `a11·a22 − a12²·cos²ψ`, always
positive). `τ_net` is the total over both wheels (§3.3).

**Sign check a student can do in their head.** Upright and still, with no torque: a small forward lean
`ψ > 0` gives `ψ̈ > 0` (it falls further) and `φ̈ < 0` (the falling body pushes the wheels backward). A
positive torque gives `φ̈ > 0` and `ψ̈ < 0` (wheels forward, body tips back). Both match §2.4.

### 3.3 Motor model and the write path

**The write path, copied from `Motor12VoltQuadEncoder::write()`** (default, `model_write_quirks = true`).
For each wheel, from its command `u` (rad/s):

```
if u is not finite, or abs(u) < 0.05:  pct = 0      deadband: a stop (write(), kDeadbandRadPerSec); dir pin unchanged
else:
  dir   = +1 if u >= 0, else −1                     (write(), the digitalWrite calls)
  pct   = clamp(abs(u) / MAX_CODE · 100 + 2.0, 0, 100), truncated toward zero to an integer
                                                    MAX_CODE = 119.381 rad/s (U-3); +2.0 % stiction
                                                    offset (kStictionDutyPercent); clamp (M-3) and int (write())
duty    = dir · pct / 100                           in [−1, 1]; with pct = 0 the direction does not matter
```

`write()` now writes the speed pin on **every** cycle (finding H-2's hold is gone), so the replica has
no memory either: each command maps to its duty on its own. Note what the map does near zero: any
command from 0.05 rad/s up gets at least 2 % duty (the stiction offset), and 1.0 rad/s still gets only
2 % because of the truncation. In the code `MAX_RAD_PER_SEC` keeps its integer-division bug (U-3), and
so does `kWriteMaxRadPerSec` in `SimPlantModel.hpp`; it is copied, not fixed. When the real `write()`
changes again, this replica changes in the same commit. With `model_write_quirks = false` the sim uses
`duty = clamp(u / MAX_CODE, −1, 1)` (NaN gives 0), continuous, with no deadband, offset or truncation.

**The motor.** A permanent-magnet DC gearmotor with the electrical time constant ignored (it is far
shorter than 1 ms for motors of this size). Per motor, at the **gearbox output**:

```
τ_out = τ_stall · ( duty · V_bat / V_rated  −  ω_out / ω_noload )          [N·m]
```

This is the usual `τ = (k_t/R)·(V − k_e·ω)` written with the two numbers a datasheet gives:
`τ_stall = k_t·V_rated/R` and `ω_noload = V_rated/k_e`. With a full battery and `duty = u/MAX_CODE` it
reads `τ_out ≈ (τ_stall/ω_noload)·(u − ω_out)`: the motor pulls toward the commanded speed, with
stiffness `τ_stall/ω_noload`. That is why the real open-loop write path (H-1) behaves like a soft
velocity servo, and why its strength moves with battery voltage.

Through the belt (reduction `N_b`; the wheel turns slower): `ω_out = N_b·(φ̇ − ψ̇)` and the torque at
the wheel is `N_b·τ_out`. Summing both motors and the friction on the wheel–body joint:

```
τ_net = Σ over left, right of N_b·τ_out(duty_i)  −  b·(φ̇ − ψ̇)  −  τ_c·tanh((φ̇ − ψ̇)/ω_ε)
```

- `b` is viscous friction for both wheels together (N·m·s/rad), estimated from the no-load current
  (§4). It is about 37 times smaller than the back-EMF damping, so it barely matters.
- The Coulomb term `τ_c` is off by default (`τ_c = 0`). If turned on, `tanh` with `ω_ε = 0.1 rad/s`
  keeps it smooth enough for the integrator.
- `duty = 0` leaves the back-EMF term in place: the model assumes the driver shorts the motor during
  PWM off-time (braking). A driver that lets the motor coast instead gives less damping (§6).
- `disable_torque = true` sets the motor part of `τ_net` to zero (no drive and no back-EMF: an
  open-circuit motor). Friction stays.

### 3.4 Linearised about upright

About `ψ = 0`, `φ̇ = ψ̇ = 0`, with `model_write_quirks = false`, both wheels commanded the same `u`, and
no Coulomb friction, `τ_net = K_u·u − c·(φ̇ − ψ̇)`, where

```
K_u = 2 · N_b · τ_stall · V_bat / (V_rated · MAX_CODE)      [N·m per rad/s of command]
K_ω = 2 · N_b² · τ_stall / ω_noload                          [N·m·s/rad]   (back-EMF, both motors)
c   = K_ω + b                                                [N·m·s/rad]
D   = a11·a22 − a12²                                         [kg²·m⁴]
```

Then `Ẋ = A·X + B·u` with `X = [φ, ψ, φ̇, ψ̇]`:

```
      | 0          0                1                  0             |         | 0                   |
A  =  | 0          0                0                  1             |    B =  | 0                   |
      | 0   −a12·m_b·g·l / D   −(a22+a12)·c / D   +(a22+a12)·c / D    |         | +(a22+a12)·K_u / D  |
      | 0   +a11·m_b·g·l / D   +(a11+a12)·c / D   −(a11+a12)·c / D    |         | −(a11+a12)·K_u / D  |
```

With the §4 estimates:

```
      | 0     0        1        0      |          | 0       |
A  =  | 0     0        0        1      |     B =  | 0       |      (A in s⁻¹ and s⁻²; B in rad/s² per rad/s)
      | 0  −169.69  −5.7856   5.7856   |          | 4.1933  |
      | 0   73.814   1.5057  −1.5057   |          | −1.0913 |
```

### 3.5 Integration

**Classical 4th-order Runge–Kutta (RK4), fixed sub-step `h = 1 ms`, duties held constant over each
step.** Why:

- The fastest dynamics in the model are around 10–25 s⁻¹ (§7: the braked pole at −12.2 s⁻¹; the closed
  pitch loop's oscillation at about 23 rad/s with the YAML gains). At `h = 1 ms`, `|λ·h| ≈ 0.025`, deep
  inside RK4's stability limit (about 2.8). Higher gains found while tuning have room to spare.
- It is accurate enough that the integrator can be ruled out when the sim disagrees with anything: the
  2°→35° fall time changes by less than 0.01 ms between `h = 1 ms` and `h = 0.01 ms`. Semi-implicit
  Euler at 1 ms is off by 0.5 ms; at 10 ms (one step per control period) it is off by 5 ms. RK4 costs
  four evaluations of a 2×2 solve per sub-step, about 6 µs typical and 38 µs worst per cycle at the 50-sub-step cap, timed natively on a laptop host (realtime-auditor, 2026-11-02); not measured in the container or on a Pi.
- A 1 ms sub-step also sets the IMU delay's resolution to 1 ms (§3.7).

Once released, each `read()` adds the `period` that `controller_manager` passes to a carried
remainder `unintegrated_s_`, takes `n = floor(remainder / 1 ms)` sub-steps (with a 10⁻⁶ sub-step slack
against rounding error) and subtracts them, so a period that is not a whole number of milliseconds
loses no time: the remainder, always under 1 ms, is carried to the next `read()`. The physics clock
therefore keeps up with the sum of the controllers' `dt`. `n` is clamped to `[0, 50]`. A cycle that
wants more than 50 steps (the container stalled for more than 50 ms) increments a counter and zeroes the
remainder, so the skipped time is dropped rather than integrated in one burst; the counter is logged
at most once a second and printed in total when the plugin deactivates; it never logs per cycle, and no
interface exports it (the 14 interfaces are fixed). Before release, and in `on_activate`, the remainder
is zero. Sim time `t` (seconds since **release**, §3.6) is
the sum of sub-steps, not wall-clock time.

### 3.6 What happens in each 10 ms cycle

`controller_manager` runs `read()` → every controller's `update()` (upstream controllers first) →
`write()`, at `update_rate: 100` (`rocko_controllers.yaml`'s `update_rate`).

**`read(time, period)`**

1. Only once released (below): for each of the `n` sub-steps, if the impulse (§5) is due
   (`t ≥ impulse_time_s`, not yet applied), add it to `ψ̇`. Take one RK4 step with the duties from the
   last `write()`. Add `h` to `t`. Push the true `ψ` into the IMU ring buffer. If
   `abs(ψ) ≥ fallen_pitch_deg`, latch "fallen" (below).
2. Compute the encoder outputs (§3.7).
3. Compute the pitch output, applying the IMU delay and the fault policy (§3.7, §5).
4. If any state is not finite, return `ERROR`. If the fault policy trips, return `ERROR`. Else `OK`.

**`write(time, period)`**: if not yet released and both wheel commands are finite, release. Then for
each wheel, run the write-path replica of §3.3 on its command and store the signed duty. No physics
happens here. A command written at cycle `k` therefore acts from `read()` at `k+1` onward, over the
whole interval in between: a zero-order hold, as on the robot, where the PWM value set in `write()` runs
until the next `write()`.

**Release.** `controller_manager` activates the hardware before any controller is spawned, and spawning
takes seconds. A plant that started falling at `on_activate` would be lying on the floor before the
cascade, or anything watching it, were running. So the plugin **holds the robot still** at its initial
state (no sub-steps, `t = 0`) until the first `write()` in which both wheel commands are finite, which
is when `AddFeedforwardController` first writes after its activation. That stands in for a person
holding the robot until the controllers are up. Physics starts at the next `read()`; every scenario time
in §5 counts from that moment. The plugin logs the release once.

**The open-loop gap after release (size not measured).** The release happens when the feedforward
controllers start writing, and `AddFeedforwardController` writes a finite `0.0` when its reference is
still NaN. So the robot is possibly released before `velocity_pid_controller` has a reference, and for
some control cycles after release the wheel command may be that `0.0` and nothing from the loops: the
robot would then fall open-loop. How long such a gap lasts was **not measured**: the source of
`pid_controller` (when it first computes an output after activation) has not been read for this page.
It matters for any scenario timed "from release", because the open-loop body falls with a time
constant of about 116 ms (§7.1, row 2), so even a few control cycles change the lean the loops
inherit. Until the gap is measured, read a result such as "recovered from a 2° start" as "recovered
from a 2° start plus whatever the gap added".

**On the floor.** Past `fallen_pitch_deg` the pendulum model means nothing (the body would be lying on
the floor). The plugin then freezes `ψ` and sets `ψ̇ = 0`; the wheels keep integrating. Scenarios judge
failure at the 35° tipover angle of finding H-4 long before this.

**`on_activate`** resets everything: `ψ` from `initial_pitch_deg`, all rates 0, `t = 0`, the ring buffer
filled with the initial `ψ`, both held duties 0, the encoder filter at 0, the fault tracker new, the
impulse un-applied, both commands NaN, not released. No allocation happens in `read()` or `write()`;
the ring buffer is sized in `on_init` from `imu_delay_max_ms`.

### 3.7 What the plugin exports

| Interface (both joints unless noted) | Value | Unit |
| --- | --- | --- |
| `pitch` | `pitch_sign · degrees(ψ delayed by imu_delay_ms) + imu_pitch_bias_deg`, held on a failed read (§5) | degrees |
| `pitch_rate` | `pitch_sign · degrees(ψ̇)`, the current rate with no IMU delay; held on a failed read (§5) | deg/s |
| `yaw`, `roll` | **0.0, always.** The planar model has neither. No controller reads them (§2.1); if one ever does, this row must change | degrees |
| `position` | `encoder_sign · r · (φ − ψ)` (chassis-relative travel; the real stack reports the left wheel on both joints, and the planar sim has one value) | m |
| `velocity` (state) | see below | m/s |
| `velocity` (command) | input to the write-path replica | rad/s |

**Pitch.** The ring buffer holds one `ψ` per 1 ms sub-step; the exported value is the one
`imu_delay_ms` sub-steps old (an integer number of milliseconds). It is the plain delay of §4, standing
in for the BNO055's fusion lag plus the service call. The sim does **not** add the +7.6° of
`pitch_offset` in `ICM20948.py`: that offset turns the raw sensor angle into the robot's "upright = 0" frame, and the
sim produces that frame directly. `imu_pitch_bias_deg` models an upright reference that is wrong by a
fixed amount (default 0.0: assumes the 7.6° calibration is exact, which nobody has checked).

**Encoder velocity, `model_encoder_node = true` (default).** Reproduces finding U-2 exactly. Each
`read()`, with `p = encoder_sign · r · (φ − ψ)`:

```
s1      = (p − p_last) / 0.01            first service call this cycle: ~10 ms of motion over a fixed 0.01 s
y       = 0.95 · y + 0.05 · s1           → exported as left_wheel_joint/velocity
y       = 0.95 · y + 0.05 · 0            second call: no motion since the first → right_wheel_joint/velocity
p_last  = p
```

`0.01` and `0.05` are copied from `QuadEncoders.py:71`, `:39`; they are code, not estimates. With
`model_encoder_node = false` (for after finding C-1's encoder fix) both joints export
`encoder_sign · r · (φ̇ − ψ̇)` with no filter. Encoder counts are not quantised in either mode (§6).

**Pitch rate (finding H-6, built).** Both joints export `pitch_rate` = `pitch_sign · degrees(ψ̇)` in deg/s
(`SimPlantModel::imu_pitch_rate_deg_s()`), the **current** lean rate with **no** fusion delay (the gyro
rate skips the filter the delay stands in for) and no bias term (a constant angle error has no rate).
It shares the pitch's fault policy (§5): a failed read holds both the last good pitch and the last good
rate, and a non-finite rate counts as a failed read of the whole sample. **This differs from the real
plugin**, which judges the rate on its own: there a bad rate (not finite, or beyond 2000 deg/s) is held and
counted separately and never makes `read()` return `ERROR`, and a good pitch is kept. The sim has no
gyro-only fault, so no sim check exercises the plugin's rule. The mock block exports it too and zero-fills it, as it
does `pitch`. Check H6 of `verify_sim.sh` shows that `pitch_pid_controller`'s `feedback_dot` equals this
value in the same control cycle (600 pairs, largest 46 deg/s). That shows the wiring. It does not show
that the gyro's real rate is as clean as this one, which is exact and undelayed.

---

## 4. Parameters

### 4.1 Physical parameters: `ESTIMATE, NOT MEASURED`

> **`ESTIMATE, NOT MEASURED`.** Every number in this table is invented or inferred. None was weighed,
> measured or read off a part on ROCKO. Each is a named plugin parameter with the default below, so a
> measured value replaces it without a code change (the body's inertia is the exception: it has no
> parameter yet). [measure-inertia.md](../guides/measure-inertia.md) says how to measure them, and
> [proposal 0007](../proposals/0007-measure-robot-parameters.md) asks a human to.

| Parameter | Symbol | Unit | Estimate | Basis |
| --- | --- | --- | --- | --- |
| `wheel_radius_m` | `r` | m | 0.06 | The value the balancing stack uses today; see §4.2. **Issue #16 stays open** |
| `belt_ratio` | `N_b` | – | 1.355 | `QuadEncoders.py:35` comment ("1.355 reduction from belts"); the older `QuadEncoder.py:28` has none (U-4) |
| `wheel_mass_kg` (each) | `m_w` | kg | 0.3 | `rocko_description.urdf.xacro:12`, marked "arbitrary" there. Plausible for a 120 mm wheel with hub and pulley |
| wheel inertia (derived) | `I_w` | kg·m² | 1.08 × 10⁻³ (both) | Solid disc, `½·m_w·r²` each. Motor rotor inertia reflected through the gears is left out |
| `body_mass_kg` | `m_b` | kg | 2.0 | **Invented** from a guessed parts list: two gearmotors ≈ 0.5 kg, 12 V pack ≈ 0.4 kg, Pi and boards ≈ 0.15 kg, frame ≈ 0.9 kg. The URDF's 0.3 kg (`:8`) is marked "arbitrary" and is lighter than the motors alone, so it is not used |
| `com_height_m` | `l` | m | 0.20 | **Invented.** Axle to body centre of mass, for a body about 0.4 m tall |
| `body_height_m`, `body_depth_m` | `h_b`, `d_b` | m | 0.40, 0.10 | **Invented.** Used only for `I_b` |
| body inertia (derived) | `I_b` | kg·m² | 0.02833 | Uniform box about its centre: `m_b·(h_b² + d_b²)/12` |
| `gravity_m_s2` | `g` | m/s² | 9.81 | Standard |
| `motor_stall_torque_Nm` | `τ_stall` | N·m | 0.774725 (each, gearbox output, 12 V; 7.9 kgf·cm × 0.0980665, the value the derived numbers below use) | **Inferred.** The code's 1150 RPM (`MAX_RAD_PER_SEC` in `Motor12VoltQuadEncoder.hpp`) and "145 PPR at gearbox output shaft" (`QuadEncoders.py:35`) match the published figures for a goBILDA 5203-series 5.2:1 gearmotor, whose datasheet stall torque is 7.9 kgf·cm. The repo never names the motor, and the datasheet figure is quoted from memory, not re-checked |
| `motor_no_load_speed_rad_s` | `ω_noload` | rad/s | 120.43 (gearbox output, 12 V) | 1150 RPM from the `MAX_RAD_PER_SEC` comment in `Motor12VoltQuadEncoder.hpp`, computed without the integer-division bug (U-3) |
| `motor_stall_current_A`, `motor_no_load_current_A` | `I_stall`, `I_nl` | A | 9.2, 0.25 | Same inferred datasheet. Used only to estimate `b` |
| viscous friction (derived) | `b` | N·m·s/rad | 6.42 × 10⁻⁴ (both wheels) | Per motor: `N_b²·k_t·I_nl/ω_noload`, with `k_t = τ_stall/I_stall`. The URDF's `damping="0.2"` (`:51`) has no stated basis and is not used |
| `coulomb_friction_Nm` | `τ_c` | N·m | 0.0 (off) | Unknown. Off until measured |
| `battery_voltage_V` | `V_bat` | V | 12.0 | Nominal 12 V pack (the motor class is `Motor12Volt…`). Constant: no sag (§6) |
| `motor_rated_voltage_V` | `V_rated` | V | 12.0 | The voltage `τ_stall` and `ω_noload` are quoted at |
| `imu_delay_ms` | – | ms | 20 | Finding H-6: the BNO055 fusion output lags by "tens of milliseconds"; plus the blocking service call (C-1). Integer, ≤ `imu_delay_max_ms` |
| `imu_pitch_bias_deg` | – | ° | 0.0 | Assumes the 7.6° offset makes 0.0 exactly upright (§3.7) |
| `fallen_pitch_deg` | – | ° | 60 | Invented; only stops meaningless physics (§3.6) |

Derived with these numbers: `a11 = 0.01044`, `a12 = 0.024`, `a22 = 0.10833` kg·m²;
`D = 5.55 × 10⁻⁴`; `K_u = 0.017587` N·m per rad/s; `K_ω = 0.023623` and `c = 0.024265` N·m·s/rad.

**Copied from code, not estimates** (they must equal the real code's values; change them only when the
code changes): `MAX_CODE = 119.381` rad/s, integer-division bug included (U-3); write deadband
`0.05` rad/s and stiction offset `2.0` % (`kDeadbandRadPerSec` and `kStictionDutyPercent` in `Motor12VoltQuadEncoder.hpp`); duty clamp to
`[0, 100]` and truncation to `int` (`write()`, the `softPwmWrite` call); PWM range 100 (`softPwmCreate` in `on_activate()`); encoder filter `alpha = 0.05` and
assumed period `0.01` s (`QuadEncoders.py:39`, `:71`).

**Conventions and switches** (not physical estimates; §2.4 and §3):

| Parameter | Default | Meaning |
| --- | --- | --- |
| `pitch_sign` | −1 | Exported pitch = `pitch_sign × lean toward sim-forward`. Assumption, §2.4 |
| `encoder_sign` | +1 | Exported position and velocity = `encoder_sign × motion toward sim-forward`. Assumption, §2.4 |
| `model_write_quirks` | true | Replicate today's `write()` exactly (§3.3) |
| `model_encoder_node` | true | Replicate today's `QuadEncoders.py` (U-2, §3.7) |
| `imu_delay_max_ms` | 100 | Ring-buffer size, allocated in `on_init` |
| sub-step `h`, max sub-steps | 1 ms, 50 | Fixed in code (§3.5), not parameters |

### 4.2 The wheel radius: four values, none measured

The repo holds four different wheel radii. This page does **not** pick the right one, and
[issue #16](https://github.com/BiPed-Capstone/ROCKO-env/issues/16) stays open
([next-steps](../next-steps.md) row 0.7).

| Value | Where | Used by |
| --- | --- | --- |
| 0.06 m | `wheel_radius_meters` in `DiffDriveController.__init__`; `QuadEncoders.py:34` ("60 mm radius"); `QuadEncoder.py:28` (as "120 mm wheel diameter") | Feedforward; encoder → metres in the launched node |
| 0.0813 m (effective) | `QuadEncoder.py:28` vs `QuadEncoders.py:35` | The older node converts counts to metres without the 1.355 belt factor. If the belt is real, its numbers are what a 0.06 × 1.355 = 0.0813 m wheel would give |
| 0.127 m | `rocko_tank_mode_controllers.yaml:17` | Tank mode's `diff_drive_controller` (issue #16 suspects it is left over from a demo) |
| 0.015 m | `rocko_description.urdf.xacro:14` | The URDF; what a Gazebo plant would use |

**The sim uses 0.06 m, as an estimate.** It is the value the balancing stack actually uses (feedforward
and the launched encoder node), and with it the sim's encoder metres match what the real node computes
for the same wheel rotation. It is the one value with a comment claiming a physical size. None of that
makes it correct.

### 4.3 The robustness sweep (±30 %)

The [journal](../journal/2026-10-31-sim-plant-and-critical-fixes.md)'s criterion S3 reruns every
scenario at ±30 % of four estimates, all combinations (16 corners, plus nominal):

| Swept | Parameter | Range |
| --- | --- | --- |
| Mass | `body_mass_kg` | 1.4 – 2.6 kg |
| Centre-of-mass height | `com_height_m` | 0.14 – 0.26 m |
| Motor gain | `motor_stall_torque_Nm` (scales `K_u` and `K_ω` together) | 0.542 – 1.007 N·m |
| IMU delay | `imu_delay_ms` | 14 – 26 ms |

`imu_delay_ms` is a whole number of milliseconds (sub-steps are 1 ms), so 14 and 26 ms are exact,
not rounded to a control period. The body's inertia follows `body_mass_kg` (a uniform box) but not
`com_height_m`.

**The sweep is a script:** [`sim/parameter_sweep.py`](../../sim/parameter_sweep.py) (stage 5c-2). It
runs U and I once and V three times per case (the nominal model, each parameter alone at −30 % and
+30 %, and the 16 corners), through `verify_sim.sh scenarios` with its committed thresholds and the
model changed by `SIM_PLANT_OVERRIDES` (the four parameters above are launch arguments of
`rocko_sim.launch.py`, defaulting to the table in §4.1). It writes a CSV and a markdown table with
the linear model's prediction for each case. The last results and what they mean:
[stage 5c-2 journal](../journal/2026-11-03-stage5c2-sweep.md). S2b and AB are not part of the sweep.

The [`tune-gains`](../../.claude/skills/tune-gains/SKILL.md) held-out set (different initial pitch,
±20 % on mass and wheel friction, one extra control period of sensor delay) still applies on top.

---

## 5. Fault injection and scenario parameters

All default to **off**. Times are sim time since release (§3.6), and an IMU outage or impulse can only
happen after release. Angles and rates are in the exported
pitch convention (sign applied, no bias), so `initial_pitch_deg: 2.0` means the controller reads +2.0 at
`t = 0`.

| Parameter | Unit | Default | Effect |
| --- | --- | --- | --- |
| `initial_pitch_deg` | ° | 0.0 | Starting lean: `ψ₀ = radians(initial_pitch_deg) · pitch_sign` |
| `imu_fault_start_s` | s | 0.0 | Start of the IMU outage |
| `imu_fault_duration_s` | s | 0.0 (off) | Length of the outage. Every `read()` with `imu_fault_start_s ≤ t < start + duration` is a failed IMU read |
| `impulse_time_s` | s | −1.0 (off) | When the push happens; negative means never |
| `impulse_angular_velocity_deg_per_s` | °/s | 0.0 | Added once to the pitch rate at `impulse_time_s`: `ψ̇ += radians(value) · pitch_sign`. `φ̇` is left unchanged, as if the push's reaction on the wheels were cancelled at the floor. A modelling convenience, not a real force |
| `disable_torque` | bool | false | Motor torque zero from activation (§3.3): the "no control effort ⇒ it falls" check (journal S2a) |

**What a failed IMU read does: the finding C-2 policy, as the real plugin applies it.** The sim reuses
`ImuFaultTracker` (`rocko_env/.../icm20948/include/rocko_env/ImuFaultTracker.hpp`, written for C-2 with no
ROS types so a simulator can share it), with the same thresholds, 5 in a row and 10 of the last 20:

- On a failed read: **keep the last good pitch** (never write 0.0; the value is NaN only if no good read
  has happened yet), call `record(false)`, and if `tripped()` (5 failures in a row = 50 ms, or 10 of the last 20 reads, finding H-8), return
  `hardware_interface::return_type::ERROR`.
- On a good read: update pitch, call `record(true)`.
- Log at most once a second, never per cycle.
- The pitch rate (§3.7) is held and replaced together with the pitch, and a non-finite rate counts as a
  failed read of the sample. The real plugin does not do this: it judges the rate on its own and never
  returns `ERROR` for it. A gyro-only fault is therefore not modelled.

**A difference from the robot that the sim cannot hide.** On the robot the IMU is its own hardware
component, so an IMU `ERROR` takes down the IMU while the two motor components stay active and keep
running `write()` with whatever is in their command slots. Since the 2026-10-31 changes,
`AddFeedforwardController` writes 0.0 when it is deactivated (finding H-7) and `write()` writes the
speed pin every cycle (H-2), so by reading the code the motors should go to 0 % duty; that has not been
observed. In the sim one `SystemInterface` owns everything, so an `ERROR` stops the whole plant. A
fault scenario can check that `ERROR` is returned and that the controllers never see 0.0; it **cannot**
show what the real motors do afterwards.

**What was observed (check C2, 2026-11-02, `controller_manager` 4.48.1 in the container).** With
`initial_pitch_deg:=2.0` (so a true pitch of exactly 0.0 cannot happen) and an outage starting 1 s
after release:

| Outage | Result |
| --- | --- |
| 0.03 s (3 reads, under the limit of 5 in a row) | `read()` never returned `ERROR`; the controllers and the plant stayed active; no exported pitch sample was exactly 0.0 or NaN (a held value shows as a run of 5 identical samples) |
| 0.2 s (20 reads) | `read()` returned `ERROR`. `controller_manager` then logged `Deactivating following hardware components as their read cycle resulted in an error ... [ SimRocko ]`, deactivated **all five** controllers (`joint_state_broadcaster` too), and also logged `Not acceptable command interfaces combination` and `Error while attempting mode switch when deactivating controllers in read cycle!`. End state: controllers `inactive`, `SimRocko` `unconfigured`. Not one exported pitch sample was 0.0 or NaN |

Also seen: when the failed read was made to export 0.0 on purpose, the check went red with 4 zero
samples. The long-outage row is what the one-owner paragraph above predicted in shape (the whole plant
stops); it is not what the robot would do. Because the IMU is a separate component there, this run
does not tell us which controllers `controller_manager` would deactivate, or whether the motor plugin
would keep its last duty (finding H-7). That remains a question for a Pi.

---

## 6. What the model leaves out

Each omission, and which way it biases a gain found in the sim. "Optimistic" means the sim makes the
gains look safer than they will be on the robot. The longer version, with the loop each omission hits
and why, is [sim-to-real-gap.md](sim-to-real-gap.md).

| Omitted | Effect on a sim-tuned gain |
| --- | --- |
| Gear and belt backlash | A dead band in the torque path. Optimistic: high inner-loop P and D that are smooth in the sim chatter or limit-cycle on the robot |
| Software-PWM aliasing (finding M-2: PWM period = control period = 10 ms) | The sim applies the duty as a smooth average voltage; the robot gets pulses that beat against the loop. Optimistic, worst for D |
| Battery sag | `V_bat` is fixed at 12 V; the real voltage falls with charge and under load, so the real `K_u` is lower. Optimistic: a gain set must survive the motor-gain −30 % corner |
| Stiction, motor dead band | The replica keeps the code's 0.05 rad/s deadband and +2 % offset, but the motor's own static friction is off. Optimistic near upright: small corrections that move the sim's wheels may not move the robot's |
| Encoder quantisation | 1.92 mm per count; over 10 ms one count is 0.19 m/s before the filter. The sim's velocity is noise-free. Optimistic for the velocity loop's D (0.2) |
| Tyre slip | No slip at any acceleration. Optimistic for aggressive recoveries on smooth floors |
| IMU fusion lag beyond one pure delay | The BNO055's filter lags differently at different frequencies, and the blocking service call (C-1) adds jitter. Optimistic on phase margin; the D term suffers most |
| Motor current limits | Torque can reach `τ_stall` instantly; a driver or battery limit would cap it. Optimistic for recovery from large angles |
| Loop-timing jitter on the Pi (H-5) | The container's timing says nothing about the Pi. Unknown direction, usually optimistic |
| Tether or cable drag | No external torque. Unknown direction; adds a disturbance in tethered tests |
| Floor compliance, rolling resistance | Rigid, lossless floor. Carpet adds damping (may help) and rolling resistance (needs a steady lean, loads the integrator) |
| Driver behaviour at zero duty | The model brakes (back-EMF damping present); a coasting driver removes it. Changes the plant's damping; which way the gain moves depends on the loop |
| Turning, yaw, left/right differences | Planar only; turning cannot be tested |

---

## 7. How the numbers are checked

The numbers below come from the parameters in §4 and are what `tools/sim_plant_model_test.cpp` asserts.
They test the **model as specified**, not the robot. The physics is in a plain header with no ROS types
(as `ImuFaultTracker` is), so the unit test calls it directly.

**How they were computed.** Python 3, standard library only (no NumPy on the laptop), on 2026-10-31,
from a scratch script (not committed: the unit test that asserts these numbers becomes the committed
reproduction). Closed-form values by hand from §3.4. Eigenvalues: characteristic polynomial
(Faddeev–LeVerrier) and Durand–Kerner roots. Fall times: the nonlinear equations of §3.2, RK4 at
`h = 1 ms` and again at `h = 0.01 ms` (they agree to 10⁻⁵ s), linearly interpolated at the 35° crossing.
Closed-loop figures: the linear model discretised at 10 ms (zero-order hold, by fine RK4), spectral
radius by repeated squaring.

### 7.1 Sanity numbers for unit tests

| # | Check | Expected | Tolerance |
| --- | --- | --- | --- |
| 1 | Inertia terms | `a11 = 0.01044`, `a12 = 0.024`, `a22 = 0.108333` kg·m², `D = 5.55 × 10⁻⁴` | 10⁻⁶ relative |
| 2 | Open-loop unstable pole, torque off, **no friction**: closed form `λ = sqrt(a11·m_b·g·l / D)` | **8.5915 s⁻¹** (time constant 0.116 s). Eigenvalues of `A` with `c = 0`: `{0, 0, ±8.5915}` | 10⁻³ |
| 3 | Same with friction `b` (torque off, `c = b`) | `{−8.6579, −0.0615, 0, +8.5265}` s⁻¹ | 10⁻³ |
| 4 | Motor braked at `duty = 0` (`c = K_ω + b`) | `{−12.1916, −2.0304, 0, +6.9306}` s⁻¹ | 10⁻³ |
| 5 | Cross-checks on #2: a point mass on a fixed pivot, `sqrt(g/l)` = 7.0036 s⁻¹; the body on a fixed pivot, `sqrt(m_b·g·l/a22)` = 6.0184 s⁻¹. Free wheels let the body fall faster than either | ordering `8.59 > 7.00 > 6.02` | – |
| 6 | Fall time, `disable_torque = true`, from rest at `initial_pitch_deg = 2`, until abs(true lean) reaches 35°, **model only** (no ROS, no delay) | **0.4273 s** with `b`; **0.4250 s** with `b = 0` | ±1 ms |
| 7 | Same through the plugin at 100 Hz, `imu_delay_ms = 20`: first `read()` after release whose exported abs(pitch) ≥ 35 | 0.45 s after release (fall + 20 ms delay, rounded up to the next 10 ms cycle). Seen from outside, timed from the last sample still at the starting pitch, the delay cancels: 0.43 s (`verify_sim.sh` S2a) | ±10 ms |
| 8 | State at the 35° crossing in #6 (with `b`) | lean rate ≈ 254.6 °/s; axle moved ≈ −0.074 m (the wheels roll **backward** as the body falls forward). Stated for a true lean toward sim-forward; with `pitch_sign = −1`, `initial_pitch_deg = +2` is a true lean of −2°, so both signs flip (the unit test measures them along the lean) | 1 % |
| 9 | Energy, `b = 0`, torque off, 2°→35°, RK4 at 1 ms | `T + V` constant to 10⁻⁹ relative (measured 6 × 10⁻¹²) | 10⁻⁹ |
| 10 | Exactly upright, all rates 0, `u = 0` | State stays exactly 0 for 10 s | bitwise |
| 11 | Write replica, `u` (rad/s) → percent written (the new `write()`) | 0 → 0; 0.049 → 0 (deadband); 0.05 → 2 (offset alone); 1.0 → 2; 1.2 → 3; 5.0 → 6; 60.0 → 52; 119.381 → 100 (102 clamped); 10⁹ → 100; −5.0 → 6 with direction negative; NaN → 0. Deadband `abs(u) < 0.05` | exact |
| 12 | Motor function alone, `V_bat = V_rated` | `τ_out(duty 1, ω_out 0) = τ_stall`; `τ_out(duty 1, ω_out = ω_noload) = 0` | 10⁻¹² |
| 13 | Encoder replica, constant chassis-relative speed `v` | Steady state: left 0.5128·v, right 0.4872·v (mean 0.5·v). Per-cycle pole 0.9025, time constant 97.5 ms | 10⁻³ |
| 14 | IMU delay | A step in true lean appears in exported pitch exactly `imu_delay_ms` ms later (at the first `read()` after that) | one sub-step |
| 15 | Sign, default `pitch_sign = −1` | From upright, a positive command on both wheels makes exported pitch **rise**; with torque off, `initial_pitch_deg = +2` grows toward **+35** | sign |
| 16 | IMU fault | With an outage covering 5 reads: reads 1–4 return `OK` and hold the last good pitch; read 5 returns `ERROR`; pitch is never 0.0 unless the held value was 0.0 | exact |
| 17 | Parameter validation (`invalid_reason()`) | The defaults are accepted; `imu_delay_max_ms` 1000 is accepted and 1001 refused; `body_height_m = 0`, `body_depth_m < 0`, `battery_voltage_V` of 0 or NaN are refused; a body with no wheel mass and a height of 10⁻⁹ m (singular mass matrix, `a11·a22 − a12² ≈ 0`) is refused with a reason that names the mass matrix | exact |

Sweep corners of #2–#4 and #6, so a test can check the trend (only the swept parameter changes):

| Swept | ×0.7 | ×1.0 | ×1.3 |
| --- | --- | --- | --- |
| `body_mass_kg`: torque-off pole (with `b`) / fall time / braked pole | 8.042 / 0.4506 s / 6.481 | 8.527 / 0.4273 s / 6.931 | 8.897 / 0.4112 s / 7.316 |
| `com_height_m` (with `I_b` unchanged): same three | 8.182 / 0.4417 s / 6.852 | 8.527 / 0.4273 s / 6.931 | 8.447 / 0.4340 s / 6.720 |
| `motor_stall_torque_Nm`: same three | 8.546 / 0.4266 s / 7.287 | 8.527 / 0.4273 s / 6.931 | 8.507 / 0.4280 s / 6.653 |

(The centre-of-mass row is not monotonic because `I_b` is held fixed while `l` moves.)

### 7.2 Model predictions with the hardware gains (not robot claims)

These say what the **estimated model** does with the gains in `rocko_controllers.yaml`. They are here
so a reader knows what to expect and can tell a bug from the model's own behaviour. They say
**nothing** about whether the real robot balances.

- **Inner loop alone** (velocity loop off, no feedforward, `model_write_quirks = false`,
  `pitch_sign = −1`, no IMU delay): continuous closed-loop poles `−6.93 ± 22.53j`, `0`, `+0.309` s⁻¹. The
  fast pair is the pitch oscillation (about 3.6 Hz); the `+0.309` is the slow run-away of §2.4,
  equation (1). So "hold upright 10 s" with the velocity loop off should **fail** in the model by
  creeping off over a few seconds, not by toppling at once.
- **With IMU delay**, discretised at 100 Hz, inner loop alone (spectral radius per 10 ms cycle; > 1 means
  unstable): 0 ms → 1.0033; 10 ms → 1.0033; **20 ms → 1.0117**; 30 ms → 1.0360. Between 10 and 20 ms of
  delay the fast 3.6 Hz pitch oscillation itself goes unstable in the model, on top of the slow creep.
  This is finding H-6's point in numbers, for this model.
- **Flipped pitch sign** (`pitch_sign = +1`), 20 ms delay, inner loop alone: 1.218 per cycle, a fall in
  a few tenths of a second. This is the deliberately broken input journal criterion S2(b) asks for.
- **Whole cascade**, 20 ms delay, `pitch_sign = −1`: `encoder_sign = +1` → 1.0083 (the slow real root
  of §2.4, about 0.85 s⁻¹); `encoder_sign = −1` → 1.0292 (signs right, but in this model the YAML's
  velocity gains do not stabilise it). With the estimated plant, **no** combination of the two sign
  parameters makes the hardware YAML hold the simulated robot. That is a statement about invented
  numbers, and it is why the sim's own gains go in a `*_sim.yaml`.

**Predicted vs observed (2026-10-31, after the plant was built).** The second bullet predicted
that with a 20 ms IMU delay the fast pitch oscillation itself goes unstable. That is **not what the
built simulator did.** One run was made with the unchanged hardware gains, a 2 deg start, teleop on
(no gamepad, so a zero velocity reference), `pitch_sign = −1`, `encoder_sign = +1` and a 20 ms IMU
delay. The pitch decayed to about 1 deg and then drifted away slowly: 4.8 deg at 8 s, 13 deg at 9 s,
31.7 deg at 10 s after release, past 35 deg at 10.05 s, and lying at 60 deg at 10.20 s (wheel
velocity −2.0 m/s at 10 s). Slow drift, no growing oscillation observed (as reported). The prediction assumed an
idealised write path and no encoder quirks; the run had both switched on (they are the defaults:
`model_write_quirks` and `model_encoder_node`). Which of those differences, if any, accounts for the
different failure was **not investigated**, and neither was whether the slow drift is the run-away of
§2.4, equation (1). The run was not recorded with an exact command, so the numbers above come
from the implementer's report of it, not from something this page can reproduce on demand. All of it is the behaviour of
a model with estimated parameters and assumed signs; it says nothing about the real robot or about
whether the hardware gains are good or bad.

**Observed with H-6 wired in (2026-11-02, model only).** The same kind of run, with the unchanged
hardware gains, a 2 deg start and a zero velocity reference, but the sim YAML's `pitch_pid_controller`
reading `pitch_rate` (§2.2). The figures below are from the integrator's runs; the exact command was
not recorded in this page.

| | pitch at 1 s | at 2 s | at 8 s | at 9 s | at 10 s | first at or past 35 deg |
| --- | --- | --- | --- | --- | --- | --- |
| With H-6 (left / right) | 0.43 / 0.31 deg | 0.26 / 0.04 deg | 10.1 deg | 23.3 deg | – | 9.42 s / 9.34 s |
| Without H-6 | 1.59 deg | – | 5.9 deg | – | 34.4 deg | 10.02 s |

So with the rate wired in, the first two seconds settle faster, and the slow drift then tips the
model **sooner**. Neither holds the model up. This is a slow run-away, like the one in the stage 3
note above, and it is consistent with the outer velocity loop's sign problem of §2.4 (equation (1)), **but that has not been shown**: no run isolated it, and it is **not
yet explained**. Stage 5 is to investigate. It is not evidence that the D term from the gyro is bad or good, and it says nothing about the real robot.

**Explained, in the model (stage 5, 2026-11-03): finding H-12.** The slow drift is the outer loop's
run-away of §2.4: with the default signs it has a real closed-loop pole at +0.82 /s (discrete, as the sim
runs), and more velocity gain moves it further right. With `encoder_sign = −1` that pole goes, but the
fast pair predicted in the second bullet above grows instead (|z| = 1.018 per cycle with H-6), so the
hardware gains do not hold the model with either sign. Reproduce: `python3 -I sim/cascade_linearisation.py`
on the laptop. Numbers and confidence: [stage 5 journal](../journal/2026-11-03-stage5-balance-in-sim.md);
the robot check: [proposal 0006](../proposals/0006-velocity-loop-sign-check.md).

---

## Further reading

- [control-cascade.md](control-cascade.md): why the loops are nested.
- [balance-loop-findings.md](../claude-analysis/balance-loop-findings.md): findings C-1, C-2, H-1, H-2,
  H-6, M-2, M-4, cited above.
- [ADR 0003](../decisions/0003-simulation-target.md) and the
  [`bootstrap-sim` skill](../../.claude/skills/bootstrap-sim/SKILL.md): where this plant sits in the plan.
- Lagrangian mechanics for a wheeled inverted pendulum: any undergraduate dynamics text's "cart-pole"
  chapter; this model is a cart-pole whose cart is a rolling wheel driven by a torque between wheel and
  pole.
