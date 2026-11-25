# Proposal 0008 — candidate gains for the real `rocko_controllers.yaml`, and a tethered test

- **Status:** Open
- **Gate:** hardware gains (the YAML edit; the hook denies an agent) and on-robot step (the test)
- **Raised:** 2026-11-04, built on `77664f4`; brought up to date 2026-11-25; run journals:
  [stage 5c-1](../journal/2026-11-03-stage5c1-sim-gain-tuning.md),
  [stage 5c-2](../journal/2026-11-03-stage5c2-sweep.md),
  [final report](../journal/2026-11-04-final-report-stage-4-6.md)

> **Nothing in this proposal was tested on a robot.** The gains below are **CANDIDATES ONLY**. They
> balance **in simulation under an estimated model** ([gap page](../architecture/sim-to-real-gap.md)),
> with the robot's measured signs. Do not apply them before every precondition is met. The
> agent did not edit `rocko_controllers.yaml`.

## The change

*Brought up to date 2026-11-25, without the robot ([journal](../journal/2026-11-25-offline-sim-work.md)): the robot's
signs are measured and in both YAML files, the pitch loop's D term reads the gyro on the robot
([0016](0016-pitch-d-term-from-the-gyro.md), applied), and the simulation was retuned for those signs on 2026-11-24
([journal](../journal/2026-11-24-sim-measured-signs-retune.md)). The candidates are now exactly the gains in
`rocko_sim_controllers.yaml`; the 2026-11-04 version of this table (velocity p +25, positive gains, an assumed encoder
sign) is in this file's git history.*

Candidates, in the same units as the file (`p` is (rad/s) per degree for the pitch loop and degrees
of target pitch per (m/s) for the velocity loop):

| Loop | Key | Now (`rocko_controllers.yaml`) | Candidate (`rocko_sim_controllers.yaml`) |
| --- | --- | --- | --- |
| `pitch_pid_controller` (gains under `left/right_add_feedforward_controller/...`) | `p` | 10.0 | **15.0** |
| | `i` | 0.0 | 0.0 |
| | `d` | 0.1 | **1.0** (both on the gyro rate since 0016) |
| `velocity_pid_controller` (gains under `pitch_pid_controller/...`) | `p` | -2.0 | **-12.5** |
| | `i` | -5.0 | **0.0** |
| | `d` | -0.2 | -0.2 |

The negative velocity gains are proposal 0006 option A (the encoder reads positive under a positive command). The
pitch loop's clamps stay at ±150. The velocity loop's clamps are [proposal 0017](0017-clamp-the-target-lean.md)'s
question (±5 deg; with `i = 0` only its output clamp, `u_clamp`, can act); the candidates pass the simulation with
and without it. A human makes the edit and keeps the old values in a comment, as the file does today.

## Why

Stage 5c-1 found gains that hold the simulated robot up, and the 2026-11-24 retune moved them to the measured signs.
The hardware gains do not hold the model (finding H-12 and a marginal pitch loop at the estimated 20 ms IMU delay),
so a human needs a concrete, bounded set to try, with the test to try it. This is the "retune the gains, last" item
(next-steps row 3.3), prepared.

## Preconditions (all of them, in this order)

1. **Done 2026-11-24:** [proposal 0006](0006-velocity-loop-sign-check.md), both signs measured (pitch goes down
   leaning toward a positive command; the encoder reads positive), option A applied to the hardware YAML and the sim.
2. **Open:** [proposal 0007](0007-measure-robot-parameters.md), at least the wheel's outside diameter (the robot's code
   uses it) and the centre-of-mass height (it sets how fast the model falls); then the scenarios and the ±30 % sweep
   are rerun on the measured model. Without that the candidates are tuned to estimated inertia.
3. **Done 2026-11-23:** the gyro's axis (0), sign (-1) and unit ([journal](../journal/2026-11-23-gyro-check.md)); they
   live in `rocko.ros2_control.xacro` since 2026-11-24.
4. **Done 2026-11-25:** the pitch-rate wiring ([0016](0016-pitch-d-term-from-the-gyro.md)) is in the hardware YAML and
   was built and loaded on the Pi, balance left disabled ([journal](../journal/2026-11-24-pitch-d-term-from-the-gyro.md)); its stand test is still to do.
5. **Open:** [proposal 0004](0004-safety-thresholds.md)'s thresholds confirmed by a human: the tipover cutoff (35 deg
   of reported pitch), its 10 deg re-arm band, the teleop watchdog (0.2 s).
6. **Open:** [proposal 0017](0017-clamp-the-target-lean.md) decided, the 2026-11-25 pitch offset pulled and built on the
   Pi (level reads about 0), and the pitch read at the robot's balance point (0017's test, step 1).
7. The Pi has the build in which those exist (`git log` shows the commit) and the battery is charged.
   Record its voltage; do not invent a pass threshold.

## Evidence, and its limits

All of it is Tier 3 or the offline linear model: estimated plant parameters, the measured signs, a 20 ms IMU delay.

| Check | Result |
| --- | --- |
| `verify_sim.sh scenarios`, these gains (2026-11-24, `tools/repeat_check.sh scenarios 3` and `"scenarios V" 10`, [retune journal](../journal/2026-11-24-sim-measured-signs-retune.md)) | all pass: U late \|pitch\| 0.045 to 0.065 deg; I peak 4.4 to 4.6 deg, settled in 0.89 to 0.90 s; V 90 % in 1.55 to 1.92 s, overshoot -5.5 to +2.5 %; K 2.02 to 2.04 s; S2b and AB `ok` |
| The same with proposal 0017's clamp (2026-11-25, once) | all pass; the clamp only shows in AB (falls later) and U (late \|pitch\| 0.22 deg) |
| ±30 % sweep, these gains (2026-11-25, [results](../../sim/results/2026-11-25-measured-signs-sweep.md)) | 25 cases, 125 runs: PASS 24, FAIL 1, no fall anywhere; U and I pass in all 25. The one FAIL is one V run of the corner `m-h-k-d+` (light, short, weak motor, 26 ms): 90 % reached at 3.107 s against a 3.0 s limit; its other runs took 2.08 to 2.09 s, and 5 reruns 1.96 to 2.21 s (7 of 8 pass). That run had a 137 ms container stall, which explains about 0.09 s, not the 1 s; cause not found, kept red. The old corner failure (`m+h+k-d+`, V overshoot about 46 % with the 2026-11-04 gains) now passes, overshoot at most -5.7 % |
| Linear model, `python3 -I sim/cascade_linearisation.py --candidate yaml` (2026-11-25, with the C++ encoder filter) | stable (largest \|z\| 0.977 per cycle); gain margin x3.2 up (+10.1 dB), x0.12 down (-18.2 dB); stable for an IMU delay from 0 to 70 ms (50 ms over the modelled 20 ms); its time simulation of U, I and V passes |

**Limits** ([gap page](../architecture/sim-to-real-gap.md), reasoning, not measurement):

- The delay margin delays only the angle; the sim's gyro rate is exact and undelayed, so the
  margin is probably **too large**.
- There is **no gyro noise** in the sim, and `d = 1.0` is ten times the file's value: on the robot
  it multiplies noise into the wheel command.
- **Velocity `i = 0`:** a velocity error is never integrated away, so an error in the upright reference (level reads 0
  since 2026-11-25, but the balance point is not measured) leaves a steady drift speed. The 1 deg example in the gap
  page (section 3.12) is order-of-magnitude reasoning.
- No backlash, PWM aliasing, battery sag, motor dead band beyond the code's (the robot's is 21 to 28 % duty, [0013](0013-tank-mode-motor-breakaway.md)),
  slip, quantisation or CPU jitter; not swept: wheel radius, friction, encoder sign, initial pitch; the `tune-gains`
  held-out set was not run.

## Risk

A change from p 10 to 15 and d 0.1 to 1.0 raises the loop gain and makes the D term react to gyro
noise: possible buzz, heating and chatter, or an oscillation the sim cannot show. A wrong gyro sign
makes D positive feedback, "worse than none" (finding H-6). A wrong encoder sign makes the velocity
loop run away faster at p -12.5 than at -2. The limits of the damage: wheels off the ground first, then
a tether, the 35 deg tipover cutoff, and a hand on the power cut.

## Test for a human

**What you need:** the Pi with the build, a laptop for the gamepad only if you want a command (the
test below uses none), a harness or tether from an overhead point, a person holding it, and a way
to cut motor power within reach. **This repo documents no kill switch**: if the robot has none, fit
one or do not run it. Every Pi terminal is opened with `pi/rocko_shell.sh` (a root shell with ROS, the
workspace and `ROS_DOMAIN_ID` set up: [Run ROCKO on the Pi](../guides/run-on-the-pi.md)), and the robot is
launched with `pi/launch.sh balance`:

```bash
cd ~/ROCKO-env
pi/rocko_shell.sh
```

### A. Gyro check (precondition 3; done 2026-11-23, kept for a re-check), wheels off the ground, motors unpowered if you can

1. *(Superseded 2026-11-24: the node is gone; watch `pitch_rate` in `/dynamic_joint_states` with the balance stack
   running but disabled instead.)* Run the IMU node with the gyro on (the parameter exists on the node; this runs it alone, no
   controllers):

   ```bash
   # Since 2026-11-24 (supervisor in balance mode, NOT enabled):
   ros2 topic echo /dynamic_joint_states --field interface_values
   ```

2. *(Superseded 2026-11-24: the service below is gone; tilt by hand while step 1's echo runs.)* In a second terminal, call it while tilting by hand about the axle:

   ```bash
   ros2 service call /icm20948_data rocko_interfaces/srv/Icm20948Data
   ```

3. Check, and write down: (a) `pitch_rate_deg_s` is **positive while pitch is increasing**; if not,
   the axis or the sign is wrong; (b) its size matches the angle's rate (tilt 10 deg in about a
   second: about 10 deg/s, not 570 deg/s: 57 times too large means a unit error); (c) it is near 0
   at rest, and write down its resting spread (noise). Fix `pitch_rate_axis` or `pitch_rate_sign`
   (node parameters), not the YAML, and repeat.

### B. Wheels off the ground (a stand), controllers running

1. Edit the YAML for the candidates (a human, as above; the gyro wiring is already in). Build.
2. Robot on a stand, **both wheels clear of everything**, nothing near the wheels. Launch:

   ```bash
   pi/launch.sh balance
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
real, the YAML and every gain that differs from the committed file, the
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
- Not that the pitch zero is right at the balance point: a drift in C is evidence about it, with velocity `i = 0`.

## Decision

(Human fills in: date, who, which preconditions were met, accepted/declined, where the logs are.)
