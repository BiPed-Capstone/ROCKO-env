# Proposal 0006 — Check the robot's velocity-loop sign on a stand, then flip one sign if it is wrong

- **Status:** Open
- **Gate:** on-robot step (the check), then hardware gains (if the fix is the gains)
- **Raised:** 2026-11-03, built on `23c1117`, run journal:
  [stage 5](../journal/2026-11-03-stage5-balance-in-sim.md). Finding
  [H-12](../claude-analysis/balance-loop-findings.md)

## The change

**Step 1, a check (no code change).** A human runs the two-step sign check of
[sim-plant.md §2.4](../architecture/sim-plant.md) ("The robot check that settles both signs") on a
stand, and writes the two answers here under *Decision*:

- `s_v`: does `/joint_states` `velocity` read **positive** or **negative** when a positive wheel command
  turns the wheels (tank mode, `linear.x: 0.5`)?
- `s_θ`: does the pitch go **down** or **up** when the robot is tilted toward the direction a positive
  command drives it?

**Step 2, only if `s_v` is positive** (the encoder agrees with the command, which is what the code's
authors intended): flip **one** sign. A human chooses which; do not do both, or the two cancel.

- **Option A, the gains.** In `rocko_env/rocko_env/bringup/config/rocko_controllers.yaml`, lines 55-56:

  ```yaml
  pitch_pid_controller/left_wheel_joint:   {p: 2.0, i: 5.0, d: 0.2, i_clamp_max: 150.0, i_clamp_min: -150.0} # 2.0, 0.1, 0.0
  pitch_pid_controller/right_wheel_joint:   {p: 2.0, i: 5.0, d: 0.2, i_clamp_max: 150.0, i_clamp_min: -150.0}
  ```

  becomes

  ```yaml
  pitch_pid_controller/left_wheel_joint:   {p: -2.0, i: -5.0, d: -0.2, i_clamp_max: 150.0, i_clamp_min: -150.0} # sign flipped 2026-MM-DD, proposal 0006 (was 2.0, 5.0, 0.2)
  pitch_pid_controller/right_wheel_joint:   {p: -2.0, i: -5.0, d: -0.2, i_clamp_max: 150.0, i_clamp_min: -150.0}
  ```

  The clamps stay as they are (the `control_toolbox` header in the container requires `i_clamp_min <= 0
  <= i_clamp_max`; it does not check the gains' signs; negative gains have **not** been run in the sim).
  Keeps `/joint_states` velocity in the encoder's own sign, so a person reading it sees what the wheel
  did; the cost is that the YAML's gains no longer read as "positive means correcting", which needs the
  comment.
- **Option B, the encoder.** In `rocko_env/rocko_env/hardware/sensors/quadEncoder/QuadEncoders.py`, negate
  the reported velocity (and position, so they stay consistent) so that the velocity is negative when a
  positive command drives the robot. The gains stay positive. The cost: "positive velocity" then means
  the opposite of a positive wheel command, and positive `linear.x` from the gamepad drives the robot
  the way the measured velocity calls negative, which is confusing to read; the feedforward's sign
  stays consistent either way (sim-plant.md §2.4, "Feedforward"). Option B touches a sensor node, not a
  gain line, so the hook does not gate it, but it changes what the balance loop does on the robot, so
  it is decided here all the same.

**If `s_v` is negative:** the robot's signs are already what the cascade needs, nothing changes on the
robot, and the sim's default `encoder_sign = +1` was the wrong guess: change its default to `-1` in the
xacro, the launch file and `SimPlantParams` (sim/check_sim_config.py keeps the three equal), and turn
the `AB` scenario of `verify_sim.sh scenarios` into a characterisation of a sign the robot does not have,
or remove it.

**If `s_θ` is "up"** (pitch rises when tilted toward the direction a positive command drives the robot):
stop. The pitch loop itself is then positive feedback with positive gains (sim-plant.md §2.4, "Inner
loop"), which contradicts the report that the robot partly balances; something else is going on, and no
sign should be flipped until it is understood.

## Why

Finding H-12. With every gain positive, the outer velocity loop corrects a speed error only if the
encoder reads negative when a positive command drives the robot. The authors' code assumes the
opposite. Under their assumption, in the model, the loop is positive feedback at DC: a real pole at
+0.82 /s, the simulated robot drifts away and falls about 10 s after release, and **no** positive gain
set holds it (the stable-region map in `sim/cascade_linearisation.py`, section C3). The code cannot say
which sign the robot has: it depends on how the encoders are wired relative to the motor driver's
direction pin. Tuning the velocity gains before this is settled tunes the speed of a run-away.

## Evidence

- **Laptop, linear analysis** (not a tier of the ladder): `python3 -I sim/cascade_linearisation.py`
  on `23c1117`'s YAML files, numbers in the [stage 5 journal](../journal/2026-11-03-stage5-balance-in-sim.md).
  It checks its plant against the unit-tested numbers of sim-plant.md §7.1.
- **Tier 3, the model:** stages 3 and 4 saw the simulated robot fall by a slow drift at 9.3 to 10 s with
  the as-built signs; stage 5's `verify_sim.sh scenarios` keeps that as scenario `AB` (results in the
  journal).
- **Not verified:** anything about the robot. Every plant parameter is an estimate. Whether option A or B
  then holds the robot: the model says the hardware gains' **values** still do not hold it even with the
  right sign (the pitch loop is marginal at the estimated 20 ms IMU delay), so a sign fix alone is not
  expected to make the robot balance well; it removes one structural reason it cannot.

## Risk

The check itself: tank mode on a stand drives the wheels at about 5 % duty for a few seconds; a wheel
can run away if a sign or the driver is wrong, so keep the wheels clear and the launch shell ready to
stop. The fix: a sign flip changes the outer loop's direction, so on the first balancing attempt after
it the robot may behave very differently from before (better or worse). The tipover cutoff (H-4, 35 deg
proposed) and the teleop watchdog (C-3) exist in code but have not been run on a robot; the cutoff
does not reset the velocity loop's integrator (H-10).

## Test for a human

The check (step 1), on the Pi, robot on a stand with both wheels clear:

1. Follow sim-plant.md §2.4 "The robot check that settles both signs", steps 1 and 2, exactly as
   written there (which shells, what to source, the `ros2 topic pub` command, the IMU service call).
2. Write down: the direction the wheels turned (tape an arrow on the chassis), the sign of `velocity`,
   and whether pitch went up or down when tilted toward the arrow.
3. Abort with Ctrl-C in the launch shell if a wheel runs away or the two wheels turn opposite ways
   (then the `invert` parameters are the problem, not this proposal).

If step 2 of *The change* applies, after the chosen flip, tethered, the robot held:

1. `ros2 launch rocko_env rocko.launch.py` (balancing), hands on, tether slack.
2. Release gently from upright with no gamepad input. Success: the robot does **not** creep off steadily
   in one direction, speeding up, over a few seconds (that creep is the run-away). It may still
   oscillate or fall for other reasons (gains, H-1, C-1).
3. Abort (catch it, Ctrl-C) at once if it accelerates away or the wheels run at full speed.
4. Record: commit SHA, launch file and config, battery voltage, surface, where the log went
   ([reproducing-results.md](../guides/reproducing-results.md)).

## Decision

(Human fills in: date, who, the two measured signs, which option if any, why.)
