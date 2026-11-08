# The sim-to-real gap: what the simulated robot leaves out, and what each omission does to a gain

**What this page is for:** `SimPlant` ([sim-plant.md](sim-plant.md)) is the Tier 3 plant. Gains were
tuned against it ([stage 5c-1](../journal/2026-11-03-stage5c1-sim-gain-tuning.md)) and swept against
it ([stage 5c-2](../journal/2026-11-03-stage5c2-sweep.md)). This page says what that does **not**
tell you about the robot. For each thing the model leaves out it gives the direction a gain moves
if the robot were tested, which loop it hits, and why.

> **The strongest claim available: a gain set "balances in simulation under an estimated model".**
> It does not say the robot balances. It does not say a gain is good. Every physical number in the
> model is an estimate, the encoder sign is an assumption, and the model omits everything on this
> page. Promoting a gain to the robot is [proposal 0008](../proposals/0008-candidate-gains-and-tethered-test.md),
> a human decision followed by a tethered test.

> **How to read the "direction" columns.** Every direction below is **reasoning from the loops and
> from the model's own linear analysis, not a measurement.** Nothing here was seen on a robot. Where
> a number appears it comes from the sim journals or from arithmetic shown on the spot; both are
> about the estimated model. "Optimistic" means the sim makes a gain look safer than the robot will.
> The list is a set of reasons to expect a difference, not a forecast of its size.

Last written 2026-11-04, on top of `77664f4`; sweep and V-metric figures updated 2026-11-07. The six places this page can go stale: the gains in
`rocko_sim_controllers.yaml`, the model's parameters (sim-plant.md §4), the write path in
`Motor12VoltQuadEncoder.cpp`, the encoder node, the IMU node, and the sweep results.

---

## 1. The numbers this page is about

The sim gains of stage 5c-1 (the CANDIDATES of proposal 0008; hardware values in brackets):

| Loop | p | i | d | Unit of p / i / d |
| --- | --- | --- | --- | --- |
| `pitch_pid_controller` | 15 [10] | 0 [0] | 1.0 [0.1] | (rad/s) per deg, per (deg·s), per (deg/s). In the sim the D term reads the **gyro rate** (`pitch_rate`); on the robot today it differentiates the delayed angle |
| `velocity_pid_controller` | 25 [2] | 0 [5] | 0.2 [0.2] | deg of target pitch per (m/s), per m, per (m/s²) |

Linear-model margins of those gains (estimated plant, 20 ms IMU delay, `encoder_sign:=-1`; stage
5c-1): the loop's gain on everything the motors deliver can rise ×3.20 (+10.1 dB) or fall to ×0.123
(−18.2 dB) before the model goes unstable; the IMU delay can grow from 20 ms to 50 ms, a **delay
margin of 30 ms**.

## 2. What the model puts in the loop, and what it does not

| Present in the model | Absent |
| --- | --- |
| Rigid planar body and wheel pair, rolling without slip | Slip, backlash, flex, anything off the pitch axis (sim-plant.md §3) |
| DC motor with back-EMF, at a **fixed** 12 V | Battery sag, driver current limits, PWM pulses |
| The real write path's deadband (0.05 rad/s), +2 % offset, integer truncation, clamp | Motor and gearbox static friction beyond that |
| One pure IMU **angle** delay, a whole number of ms (20 ms) | Sensor noise, any delay on the gyro rate, jitter |
| Encoder node's half-speed reading and filter (finding U-2), noise-free | Quantisation (1.92 mm per count), timing jitter of the real node |
| An ideal 100 Hz loop (the sim's own clock) | A Raspberry Pi's scheduling, DDS hops, the blocking service calls (C-1) |

---

## 3. Each omission, and what it would do to a gain

Loops: **inner** is `pitch_pid_controller` (pitch → wheel command); **outer** is
`velocity_pid_controller` (wheel velocity → target pitch). Background for the equations cited:
[sim-plant.md §2.4](sim-plant.md#24-sign-conventions-is-the-cascade-negative-feedback) (equation 1)
and [§3](sim-plant.md#3-the-model).

### 3.1 Gear and belt backlash

- **What is missing:** a dead band, a small free angle, between the motor shaft and the wheel when
  the torque reverses.
- **Hits:** the inner loop, which reverses the torque several times a second while holding upright.
- **Direction (reasoning):** optimistic for large pitch p and D. A loop with a smooth plant and a
  high gain becomes, with a free angle in it, a loop that adds phase lag exactly when the torque
  changes sign, so it can settle into a small steady oscillation (a limit cycle) the sim cannot show.
  Reducing gain usually shrinks such a cycle; raising it usually grows it. Pitch p 15 and d 1.0 are
  the largest gains the sim has tried.
- **Not covered by the sweep:** nothing in it adds a dead band.

### 3.2 Software-PWM aliasing (finding M-2)

- **What is missing:** the motor driver is fed pulses from WiringPi software PWM with a **10 ms
  period, the same as the control period** ([M-2](../claude-analysis/balance-loop-findings.md#m-2--pwm-period-equals-the-control-period)).
  The sim applies the commanded duty as a smooth average voltage held for 10 ms.
- **Hits:** the inner loop, through the torque the motor delivers.
- **Direction (reasoning):** optimistic. A pulse train whose period equals the loop period can beat
  against the loop (the number of pulses inside a given 10 ms window depends on the phase), so the
  torque seen by the robot is the commanded one plus a disturbance at the loop rate. The average
  delay of the actuator is also longer than the sim's pure hold. Both reduce phase margin. The
  duty resolution is 1 % (integer percent, 100-step range), worth about 1.2 rad/s of command at
  the code's 119.38 rad/s full scale (`MAX_RAD_PER_SEC`, finding U-3); with pitch p 15 that is 0.08
  deg of pitch error, small by itself.
- **Not covered by the sweep:** the IMU delay sweep (14 to 26 ms) is the nearest stand-in for
  extra lag, but it moves the sensor side only.

### 3.3 Battery sag (finding H-1, open-loop motor command)

- **What is missing:** the sim holds 12.0 V. The robot's voltage falls with charge and under load,
  and the motor command is open loop (finding H-1), so the plant gain falls with it.
- **Hits:** both loops, as a single factor on everything the motors deliver (`K_u` in sim-plant.md
  §3.4).
- **Direction (reasoning, with the model's own numbers):**
  - Too little gain is as unstable as too much: the linear margin down is ×0.123 (−18.2 dB), so a
    sag of tens of percent leaves the inner loop stable in the model.
  - The outer loop is more sensitive. Equation (1) of sim-plant.md §2.4 gives a steady
    over-speed of `k·Kpv / (k·Kpv − c)` for the velocity step (stage 5c-1, section 2), with `k`
    proportional to the motor gain. A weaker motor lowers `k`, which raises that factor. This is
    visible in the sweep: the only systematic failure, case `m+h+k−d+`, is the one with the
    **weakest motor** (stall torque −30 %) together with the heaviest, tallest body and the longest
    delay.
- **Covered by the sweep:** a ±30 % change of the motor gain (stall torque 0.542 to 1.007 N·m),
  which is a stand-in for a battery at roughly 70 % to 130 % of the nominal voltage (the model
  scales torque with voltage the same way). Only combined with the other parameters at the
  extremes did it fail.

### 3.4 IMU latency beyond a whole-millisecond pure delay (findings C-1, H-3, H-6)

- **What is missing:** the sim delays the **angle** by 20 ms (an estimate, integer ms) and delivers
  the pitch **rate undelayed and exact** (sim-plant.md §3.7). The real path is a blocking Python
  service call per cycle (C-1) that reads the BNO055 over I²C; the fusion output lags "tens of
  milliseconds" (H-6), the gyro read is a second I²C access (M-7), and the delay varies from cycle
  to cycle.
- **Hits:** the inner loop, and its D term most.
- **Direction (reasoning):** optimistic, and the stated **30 ms delay margin is probably too
  large.** The linear analysis delays only the angle. D on the gyro rate works because the rate
  has no delay: it supplies phase lead. If the real rate arrives late by even part of what the
  angle does (it travels through the same service call, so most of the delay is shared), that lead
  shrinks, and pitch d of 1.0 buys less damping than the sim shows. The tuned loop's stability then
  rests more on the angle path (p 15), which the 20-to-50 ms stable range already shows is the
  delayed one.
- **Not covered by the sweep:** delay on the rate; jitter (the sweep moves one fixed delay between
  14 and 26 ms).

### 3.5 Gyro noise

- **What is missing:** the sim's rate is exact. The real gyro has noise (its size on this robot is
  **unmeasured**).
- **Hits:** the inner loop's D term, through to the motors.
- **Direction (reasoning, an arithmetic example, not a measurement):** the wheel command gets
  `d × noise`. If the rate noise were `σ` deg/s rms, pitch d 1.0 adds `1.0 × σ` rad/s rms to the
  command; at σ = 1 deg/s that is about 1.0 rad/s, which the write path turns into 2 % duty
  steps (the deadband and the +2 % offset do not hide it). The hardware's d 0.1 acts on a
  differentiated, delayed angle instead, a different noise path. **Ten times the d of the hardware
  file is the one number in the candidate set that the sim is least able to judge.** An upper
  limit on d exists on the robot that the sim cannot find: audible buzz, motor heating and
  chatter, and a noisy wheel command that excites the backlash of 3.1.
- **Not covered by the sweep:** there is no noise parameter to sweep.

### 3.6 Motor dead band and stiction in the write path

- **What is missing:** the replica keeps the code's deadband (0.05 rad/s) and the +2 % offset, but
  the motor and gearbox have their own static friction, off in the model (`coulomb_friction_Nm`
  = 0).
- **Hits:** the inner loop near upright, and the outer loop's low-speed behaviour.
- **Direction (reasoning):** optimistic near upright. Small corrections that move the model's
  wheels may not move the robot's. **With both integrators at zero in the candidates** (pitch i 0
  and, in the candidates, velocity i 0) nothing slowly pushes through a dead zone, so a stiction
  band shows up as a standing offset: the robot can rest at a pitch where `p × error` just exceeds
  the band, or hunt around it. The hardware velocity i of 5 would have been working against
  exactly that; the candidates remove it.
- **Not covered by the sweep:** friction was not swept.

### 3.7 Wheel slip

- **What is missing:** the model rolls without slip at any acceleration.
- **Hits:** both loops, in large recoveries.
- **Direction (reasoning):** optimistic, for aggressive recoveries and smooth floors. Slip lowers
  the torque that reaches the ground (an effective loop-gain drop, like the weak-motor corner of
  3.3), and the encoder then reads wheel speed, not ground speed, so the outer loop sees a speed
  the robot does not have. In the impulse scenario I the wheel command peaks near 100 rad/s of the
  119 rad/s clamp, so the sim already asks for close to full torque at a 100 deg/s push.

### 3.8 Friction model and what the driver does at zero duty

- **What is missing:** the model has viscous friction only (`b` = 6.4e-4 N·m·s/rad, estimated, about
  37 times smaller than the back-EMF damping), Coulomb friction off, no rolling resistance. It also
  **assumes the driver brakes** at zero duty (back-EMF damping stays in the loop, sim-plant.md
  §3.3).
- **Hits:** the inner loop's damping, and the outer loop's stability condition `k·Kpv > c`
  (equation 1), where `c` is that back-EMF plus friction damping.
- **Direction (reasoning):** a driver that coasts instead of braking removes most of `c`. Less
  `c` makes the outer loop's condition easier and its over-speed smaller, but it also removes
  damping the pitch loop was getting for free, so the pitch pair rings more. The sign of the net
  effect on any one gain depends on which effect wins; the model cannot say, and the driver's
  behaviour is **not documented in the repo**. Rolling resistance on carpet needs a steady lean to
  overcome it, which a zero velocity integral (candidate i 0) leaves as a standing speed error.

### 3.9 Encoder quantisation and the encoder node's quirks

- **What is missing:** the sim's velocity is a smooth number. The real one comes from counts
  (1.92 mm per count by the code's own constants, `QuadEncoders.py`: 145 × 1.355 counts per wheel
  turn of 2π × 0.06 m, assuming those comments are right). Over one 10 ms cycle a single count is
  about 0.19 m/s before the filter.
- **Hits:** the outer loop, and its D term (0.2 deg·s²/m) most.
- **Direction (reasoning):** optimistic for the velocity loop's D. Through the node's low-pass
  filter (`alpha = 0.05`) one count moves the reading by about 0.01 m/s in one cycle; the PID
  differentiates that over 10 ms, about 1 m/s², so d 0.2 turns one count into about 0.2 deg of
  target pitch, and the inner loop then multiplies that by p 15 into about 3 rad/s of command.
  (Arithmetic from the code's constants, not a measurement.)
- **A related change this page cannot price:** finding U-2 makes the node read about **half** the
  true speed, and the sim reproduces that on purpose (`model_encoder_node`). If finding C-1's
  encoder fix lands, the true speed is read and the outer loop's gain roughly **doubles** (about
  +6 dB on that loop alone). The sweep did not vary this; the tuned outer p of 25 would need to be
  re-derived.
- **Not covered by the sweep:** the encoder scale (counts per wheel turn) and its noise.

### 3.10 CPU jitter of the real 100 Hz loop (finding H-5)

- **What is missing:** the sim's clock is its own; the real `controller_manager` is an ordinary
  process on a Pi, competing with the Python nodes and the two software-PWM threads (H-5).
- **Hits:** both loops, through variable sample time, and the encoder node most (it divides by an
  assumed 0.01 s).
- **Direction (reasoning):** optimistic, size unknown. **The container already shows the effect on
  the node's replica:** stage 5c-1 saw velocity samples 3.3 to 28 ms apart and a noisy measured
  velocity, and the nominal model failed the velocity-step overshoot in some runs of three
  (stage 5c-2, old V metric; the noise was a late control cycle read as a faster wheel and the
  corrected metric passes the nominal model). The Pi's timing is not the container's and neither has been measured against the
  other.
- **Not covered by the sweep:** timing jitter.

### 3.11 DDS and service-call delays

- **What is missing:** every topic and service crossing between processes. Three matter. The IMU
  and encoder service calls are on the **sensor** path of both loops (C-1: blocking, with no
  deadline). The velocity reference and the feedforward travel from `DiffDriveController.py` by
  topic. The gamepad travels over TCP and then a topic.
- **Hits:** the sensor path adds delay to the measurement of both loops (reduces margins, as in
  3.4). The reference path sits **outside** the loops: delay there does not change stability, it
  only slows the response to a command. The gamepad path matters for the 0.2 s teleop watchdog
  (`command_timeout_s`): a hiccup longer than that zeroes the reference.
- **Direction (reasoning):** optimistic for the sensor path; neutral for the stability of the
  reference path.
- **Not covered by the sweep:** all of it. The 20 ms IMU delay is the only stand-in.

### 3.12 An upright reference that is wrong (the +7.6 degree offset)

Not on the task's list, but it changes the meaning of a zero integral: `ICM20948.py` adds a fixed
+7.6 deg to the raw pitch, and nobody has checked that the reported 0.0 is "centre of mass over the
axle" (sim-plant.md §2.1). `imu_pitch_bias_deg` is 0 in the sim.

- **Hits:** both loops' steady state.
- **Direction (reasoning, from equation 1 with the estimated plant):** with outer i = 0 an error of
  `e` degrees in the upright reference leaves a steady measured speed of roughly
  `e / (Kpv × (1 + c/(k·Kpv)))`. With Kpv = 25 and the estimates' `c/(k·Kpv)` of 0.123, that is
  about 0.036 m/s per degree **in the node's units, which read half the true speed** (U-2). The
  hardware's velocity i of 5 would slowly cancel it; the candidate's i of 0 does not. Scenario U's
  limit on late speed is 0.05 m/s: one degree of reference error would use most of it.
  Order-of-magnitude reasoning on an estimated plant, not a prediction.

---

## 4. The model's estimated parameters: `ESTIMATE, NOT MEASURED`

> **`ESTIMATE, NOT MEASURED`.** None of these was weighed, measured or read off a part on ROCKO.
> This is a copy of [sim-plant.md §4.1](sim-plant.md#41-physical-parameters-estimate-not-measured),
> made on 2026-11-04 (`77664f4`); **that page is the canonical one and wins on any difference.**
> How to measure each: [measure-inertia.md](../guides/measure-inertia.md), requested by
> [proposal 0007](../proposals/0007-measure-robot-parameters.md).

| Parameter (xacro / plugin name) | Estimate | Unit | Basis, shortened |
| --- | --- | --- | --- |
| `wheel_radius_m` | 0.06 | m | The value the Python nodes use. **Not verified; four values disagree, see §5** |
| `belt_ratio` | 1.355 | – | A comment in `QuadEncoders.py`; the older `QuadEncoder.py` has none |
| `wheel_mass_kg` (each) | 0.3 | kg | URDF value marked "arbitrary" |
| `body_mass_kg` | 2.0 | kg | **Invented** from a guessed parts list |
| `com_height_m` | 0.20 | m | **Invented** |
| `body_height_m`, `body_depth_m` | 0.40, 0.10 | m | **Invented**; used only for the body inertia (uniform box, 0.02833 kg·m²) |
| wheel inertia | 1.08e-3 (both) | kg·m² | Derived: solid discs |
| `motor_stall_torque_Nm` | 0.774725 (each, gearbox output, 12 V) | N·m | **Inferred** from a datasheet recalled from memory for a motor the repo never names |
| `motor_no_load_speed_rad_s` | 120.43 | rad/s | 1150 RPM from a code comment |
| `motor_stall_current_A`, `motor_no_load_current_A` | 9.2, 0.25 | A | Same inferred datasheet; only used for the viscous friction |
| `coulomb_friction_Nm` | 0.0 (off) | N·m | Unknown |
| `battery_voltage_V`, `motor_rated_voltage_V` | 12.0, 12.0 | V | Nominal; no sag |
| `imu_delay_ms` | 20 | ms | A guess from "tens of milliseconds" (H-6) |
| `imu_pitch_bias_deg` | 0.0 | deg | Assumes the 7.6 deg offset makes 0.0 upright |
| `fallen_pitch_deg` | 60 | deg | Invented; only stops meaningless physics |

Conventions that are **assumptions**, not physics: `pitch_sign` −1 (a positive wheel command makes
the exported pitch go up) and `encoder_sign` (see §6).

## 5. The wheel radius: four values disagree (issue #16), none is picked here

The repo holds 0.06 m (Python nodes, feedforward, encoder conversion), an effective 0.0813 m
(`QuadEncoder.py` without the belt factor, if the belt is real), 0.127 m (tank mode's controller
YAML) and 0.015 m (the URDF). [Issue #16](https://github.com/BiPed-Capstone/ROCKO-env/issues/16) is
open and next-steps row 0.7 waits on a human with a ruler. **This page does not choose one.** The
sim uses 0.06 m as an estimate only because the balancing stack uses it. What a wrong radius does
in the model is not known: it enters the dynamics (the wheel inertia terms and the ground speed
for a given wheel speed), the feedforward (divided by 0.06) and the encoder conversion
separately, and **the sweep did not vary it.** Measuring it is item 5 of
[measure-inertia.md](../guides/measure-inertia.md).

## 6. The assumption `encoder_sign:=-1` ([proposal 0006](../proposals/0006-velocity-loop-sign-check.md))

With every gain positive the outer velocity loop is stable **only if the encoder reads negative
when a positive wheel command drives the robot** (finding H-12). The code's authors assumed the
opposite. Nobody has checked the robot. Every scenario that passes (U, I, V, the sweep) was run
with `encoder_sign:=-1`, labelled as an assumption. With the as-built sign, scenario AB shows the
model running away and falling about 3.8 s after release with the sim gains (9.4 s with the
hardware gains). **If the robot's encoder has the other sign, none of the gain results on this
page or in the stage 5 journals describes it**, and the velocity loop's sign has to be fixed
first (proposal 0006, options A and B).

## 7. What the sweep covered, and what it did not

[Stage 5c-2](../journal/2026-11-03-stage5c2-sweep.md): 25 cases, 125 runs, on `77f8ee1`, judged with
V's old overshoot metric; **re-run on `03238ca` with the corrected metric**
([re-run journal](../journal/2026-11-07-stage5c2-sweep-rerun.md)), which is the result below. Mass
(1.4 to 2.6 kg), centre-of-mass height (0.14 to 0.26 m), motor gain (stall torque 0.542 to 1.007
N·m) and IMU delay (14 to 26 ms), each alone at ±30 % and in all 16 corners.

| Result | Detail (corrected metric; old metric in brackets) |
| --- | --- |
| Cases | PASS 18, MARGINAL 4, FAIL 2, COULD NOT JUDGE 1 (old: PASS 14, MARGINAL 4, FAIL 7) |
| Falls | none; U and I passed in all 25 cases |
| Systematic failure | one corner, `m+h+k−d+` (heaviest, tallest, weakest motor, longest delay): V overshoot 45 to 47 % in all three runs (old: 49 to 51 %); the linear model predicts 44.8 %. Its I settle time was also the longest (1.39 s, peak 12.2 deg) |
| One lurch | `m+`, one run of three: 68 %, the teleop watchdog fired while driving (the test harness delivered commands late); the other two runs 15.5 and 24.0 % |
| One case not judged | `d+`, two runs of three: the metric refused to judge (a plant sample stamp went backwards: the container's system clock, which stamps the messages, stepped back about 16 ms, [review fixes](../journal/2026-11-07-stage5c-review-fixes.md#3-why-v-could-not-judge-two-d-runs-the-containers-system-clock-stepped-back)); the third run 15.5 %. 9 later re-runs all judged: 8 pass (15.2 to 25.2 %), 1 fail (31.5 %) after a 46 ms late control cycle made the robot really surge (see that journal) |
| The old noisy single-run FAILs | the nominal model, `h−` and `m−h−k+d−` now pass in all three runs; the encoder replica's reading of a late control cycle was the cause, and it is a property of the robot's `QuadEncoders.py` too (finding U-2) |

**Not covered:** wheel radius, friction, the encoder sign, inertia separately from mass, the IMU
bias, the initial pitch, every omission of section 3, and the `tune-gains` held-out set (another
initial pitch, ±20 % on mass and wheel friction, one more control period of delay; not run in the
nonlinear sim). S2b and AB are not part of the sweep. The sweep also moves one fixed IMU delay on
the angle only (3.4).

## 8. Where this page is used

- [Proposal 0008](../proposals/0008-candidate-gains-and-tethered-test.md) quotes it for the limits
  of the sim evidence behind the candidate gains.
- [Proposal 0005](../proposals/0005-amend-adr-0003-simulpant-first.md) links it as the mitigation
  for "a confident-looking simulator with invented inertia".
- The summary of sim-plant.md §6 is the shorter table; this page is the longer one.
