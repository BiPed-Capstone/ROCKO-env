# Proposal 0008 — candidate gains for the real `rocko_controllers.yaml`, and a tethered test

- **Status:** Open
- **Gate:** hardware gains (the YAML edit; the hook denies an agent) and on-robot step (the test)
- **Raised:** 2026-11-04, built on `77664f4`, run journals:
  [stage 5c-1](../journal/2026-11-03-stage5c1-sim-gain-tuning.md),
  [stage 5c-2](../journal/2026-11-03-stage5c2-sweep.md),
  [final report](../journal/2026-11-04-final-report-stage-4-6.md)

> **Nothing in this proposal was tested on a robot.** The gains below are **CANDIDATES ONLY**. They
> balance **in simulation under an estimated model** ([gap page](../architecture/sim-to-real-gap.md)),
> with an encoder sign nobody has checked. Do not apply them before every precondition is met. The
> agent did not edit `rocko_controllers.yaml`.

## The change

Candidates, in the same units as the file (`p` is (rad/s) per degree for the pitch loop and degrees
of target pitch per (m/s) for the velocity loop):

| Loop | Key | Now (`rocko_controllers.yaml`) | Candidate (sim-tuned) |
| --- | --- | --- | --- |
| `pitch_pid_controller` | `p` | 10.0 | **15.0** |
| | `i` | 0.0 | 0.0 |
| | `d` | 0.1, on the differentiated delayed angle | **1.0, on the gyro rate** |
| `velocity_pid_controller` | `p` | 2.0 | **25.0** |
| | `i` | 5.0 | **0.0** |
| | `d` | 0.2 | 0.2 |

Clamps stay at ±150 on both. The gain lines are lines 35 and 36 (pitch) and 55 and 56 (velocity) of
`rocko_env/rocko_env/bringup/config/rocko_controllers.yaml`; a human makes the edit and keeps the old
values in a comment, as the file does today.

**The `d` of 1.0 is not a like-for-like gain.** It only means what the sim tested if the pitch
controller reads the gyro rate. That needs three more changes together:

1. **YAML wiring** (also gated, line 29): `reference_and_state_interfaces: ["pitch"]` becomes
   `["pitch", "pitch_rate"]` for `pitch_pid_controller`, as in `rocko_sim_controllers.yaml` line 56
   (finding H-6; `sim/check_sim_config.py` allows exactly this difference in the sim file).
2. **The IMU node must read the gyro:** `read_pitch_rate` is a parameter of `ICM20948.py`, false by
   default (finding M-7). `rocko.launch.py` declares only `gui` and `use_mock_hardware` and starts
   the node with no parameters, so there is **no launch argument for it today.** Setting it needs a
   launch-file edit (`parameters=[{"read_pitch_rate": True, ...}]` on the `ICM20948.py` node, along
   with `pitch_rate_axis` and `pitch_rate_sign` once established) that the agent can write after
   this proposal is accepted. It was not written, because writing it changes what a plain launch
   does on the robot.
3. **The rate reference caveat** ([L-8](../claude-analysis/balance-loop-findings.md#l-8--pid_controller-sets-the-rate-reference-once-at-activation)):
   `pid_controller` sets the rate reference once, to the gyro's value at activation, and never
   again; a robot activated while moving carries a constant `d × rate` bias, and activation with no
   good IMU read leaves the reference NaN for good.

## Why

Stage 5c-1 found gains that hold the simulated robot up; stage 5c-2 swept them. The hardware gains do
not hold the model (finding H-12 and a marginal pitch loop at the estimated 20 ms IMU delay), so a
human needs a concrete, bounded set to try, with the test to try it. This is the "retune the gains,
last" item (next-steps row 3.3), prepared.

## Preconditions (all of them, in this order)

1. **[Proposal 0006](0006-velocity-loop-sign-check.md) is done** and its two measured signs recorded.
   If the robot's encoder reads **positive** when a positive command drives it (`s_v = +1`), the sim
   result does not describe the robot at all; one sign is fixed first (option A turns the velocity
   gains negative, which changes this table: p −25, d −0.2; option B changes the encoder), and the
   gains are re-checked in the sim with the sign the robot has. If `s_θ` is "up", stop (0006).
2. **[Proposal 0007](0007-measure-robot-parameters.md) is done**: the model's parameters are
   measured, and the scenarios and the ±30 % sweep are **rerun** on the measured model, with
   these gains or their replacements. Without that the candidates are tuned to invented inertia.
3. **H-6 is verified on the robot:** the gyro axis (`pitch_rate_axis`, guessed 0), its sign
   (`pitch_rate_sign`, guessed +1) and its unit (the Adafruit driver returns rad/s; versions before
   5.0 returned deg/s, a factor of 57), by the steps in "Gyro check" below.
4. **The `read_pitch_rate` launch change and the pitch-rate YAML wiring** (above) are in place and
   built on the Pi.
5. **[Proposal 0004](0004-safety-thresholds.md) thresholds are confirmed** by a human, in
   particular the tipover cutoff (35 deg of reported pitch, including the unexplained +7.6 deg
   offset), its 10 deg re-arm band, and the teleop watchdog (0.2 s). They have never run on a robot.
6. The Pi has the build in which those exist (`git log` shows the commit) and the battery is charged.
   Record its voltage; do not invent a pass threshold.

## Evidence, and its limits

All of it is Tier 3, the model, `encoder_sign:=-1` assumed, 20 ms IMU delay, estimated parameters:

- **Scenarios** (`verify_sim.sh scenarios`, stage 5c-1): U, I and V pass; V's overshoot was 23.5 to
  29.1 % against a 30 % limit, **narrow**, and the verdict is noisy (the verifier's own timing).
  S2b fails for the right reason (0.142 s); AB falls as H-12 predicts (3.8 s).
- **Sweep** (stage 5c-2, ±30 % of mass, centre-of-mass height, motor gain, IMU delay; 25 cases):
  re-run with V's corrected overshoot metric ([journal](../journal/2026-11-07-stage5c2-sweep-rerun.md)):
  PASS 18, MARGINAL 4, FAIL 2, COULD NOT JUDGE 1 (old metric: PASS 14, MARGINAL 4, FAIL 7); no case fell;
  **one corner fails V's overshoot systematically** (heaviest, tallest, weakest motor, 26 ms delay: 45 to
  47 %); the other FAIL is one run where the gamepad watchdog fired mid-step, and one case could not be
  judged in two runs.
- **Linear margins:** +10.1 dB up, −18.2 dB down on the motors' gain; stable to 50 ms of IMU delay
  (a 30 ms margin over the modelled 20 ms).

**Limits** ([gap page](../architecture/sim-to-real-gap.md), reasoning, not measurement):

- The 30 ms delay margin delays only the angle; the sim's gyro rate is exact and undelayed, so the
  margin is probably **too large** for a real rate that shares the service call's delay.
- There is **no gyro noise** in the sim, and `d = 1.0` is ten times the file's value: on the robot
  it multiplies noise into the wheel command.
- **Velocity `i = 0`:** a velocity error is never integrated away, so an error in the upright
  reference (the +7.6 deg offset is unverified) leaves a steady drift speed. The 1 deg example in the
  gap page (section 3.12) is order-of-magnitude reasoning.
- No backlash, PWM aliasing, battery sag, stiction beyond the code's, slip, quantisation or CPU
  jitter; not swept: wheel radius, friction, encoder sign, initial pitch; the `tune-gains`
  held-out set was not run.

## Risk

A change from p 10 to 15 and d 0.1 to 1.0 raises the loop gain and makes the D term react to gyro
noise: possible buzz, heating and chatter, or an oscillation the sim cannot show. A wrong gyro sign
makes D positive feedback, "worse than none" (finding H-6). A wrong encoder sign makes the velocity
loop run away faster at p 25 than at 2. The limits of the damage: wheels off the ground first, then
a tether, the 35 deg tipover cutoff, and a hand on the power cut.

## Test for a human

**What you need:** the Pi with the build, a laptop for the gamepad only if you want a command (the
test below uses none), a harness or tether from an overhead point, a person holding it, and a way
to cut motor power within reach. **This repo documents no kill switch**: if the robot has none, fit
one or do not run it. Every Pi terminal starts with:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

### A. Gyro check (precondition 3), wheels off the ground, motors unpowered if you can

1. Run the IMU node with the gyro on (the parameter exists on the node; this runs it alone, no
   controllers):

   ```bash
   ros2 run rocko_env ICM20948.py --ros-args -p read_pitch_rate:=true
   ```

2. In a second terminal, call it while tilting by hand about the axle:

   ```bash
   ros2 service call /icm20948_data rocko_interfaces/srv/Icm20948Data
   ```

3. Check, and write down: (a) `pitch_rate_deg_s` is **positive while pitch is increasing**; if not,
   the axis or the sign is wrong; (b) its size matches the angle's rate (tilt 10 deg in about a
   second: about 10 deg/s, not 570 deg/s: 57 times too large means a unit error); (c) it is near 0
   at rest, and write down its resting spread (noise). Fix `pitch_rate_axis` or `pitch_rate_sign`
   (node parameters), not the YAML, and repeat.

### B. Wheels off the ground (a stand), controllers running

1. Edit the YAML and launch file for the candidates and the gyro (a human, as above). Build.
2. Robot on a stand, **both wheels clear of everything**, nothing near the wheels. Launch:

   ```bash
   ros2 launch rocko_env rocko.launch.py
   ```

   (no arguments: that is the real-hardware default; `use_mock_hardware` stays false). Do **not**
   start the laptop gamepad script: with no command the velocity reference is zero and the watchdog
   logs `no teleop command yet`.
3. Tilt the body by hand, **at most 3 degrees**, toward the direction a positive command drives the
   wheels (the one you taped in proposal 0006). **Success looks like:** the wheels turn the way
   that would roll them under the lean, in proportion; at 3 deg the pitch loop asks for about
   45 rad/s, about 38 % duty, so the wheels spin fast on a stand. **Abort** (power cut) if they
   turn the other way, if one runs away without a tilt, or if there is a buzz or heat from a motor.
4. Do not tilt past 3 degrees: at 8 deg the request reaches the 119 rad/s clamp.

### C. Tether, wheels on the floor

1. Harness the robot from an overhead point with slack, with a person holding it. Surface: a flat
   floor; note it.
2. Same launch as B2. Hold the robot upright, let go gently, with no gamepad input. Repeat with
   hands ready, **5 seconds** first.
3. **Success looks like** (all of these, none of them "balanced well"): it stays within a few
   degrees of upright for the 5 seconds without catching; does not creep steadily off in one direction
   (that is the H-12 run-away); does not buzz or chatter audibly; the motors stay cool. Then a
   longer run and, only then, a gentle push by hand from a fixed spot.
4. **Abort criteria** (catch it and cut power): the machine's cutoff trips (the log shows `TIPOVER
   CUTOFF TRIPPED` at 35 deg); the human limit of **15 deg** of pitch (a proposed value for you to
   confirm, not measured); wheels at full speed; a growing oscillation; the robot accelerating
   away; buzz, smell or heat. After a trip, do not re-arm until you have stood the robot up:
   `ros2 topic pub --once -w 2 /tipover_rearm std_msgs/msg/Empty {}`. The re-arm is accepted only
   within 10 deg.
5. If it fails, go back one gain at a time ([`tune-gains`](../../.claude/skills/tune-gains/SKILL.md),
   inner loop first), and record the failure: a failed run is a result.

### What to log (reproducibility §9)

For every run, the block from [reproducing-results.md](../guides/reproducing-results.md): result,
date, `git rev-parse --short HEAD` (and whether dirty), the launch line and every argument, hardware
real, the YAML and every gain that differs from the committed file, the `read_pitch_rate`,
`pitch_rate_axis` and `pitch_rate_sign` values, battery volts at the start and end, surface, which
robot, payload, where the log lives. Record a bag:

```bash
ros2 bag record -a -o ~/rocko_logs/$(date +%Y%m%d_%H%M%S)_candidate_gains
```

and film the run.

### What NOT to conclude

- Not "the gains are better", "balance is improved" or "the sim result is confirmed" from one good
  run; or "the sim was wrong" from one bad one (a bad run can be the sign, the gyro, the radius or
  the noise).
- Not that a pass at wheels-off or at five seconds transfers to a longer run, to another battery
  level or to another floor.
- Not that the gyro is verified because A passed once: the axis, sign and unit are verified, the
  noise was only looked at.
- Not that the +7.6 deg offset is right: a drift in C is evidence about it, with velocity `i = 0`.

## Decision

(Human fills in: date, who, which preconditions were met, accepted/declined, where the logs are.)
