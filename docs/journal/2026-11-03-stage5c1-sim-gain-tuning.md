# Stage 5c-1: tune the SIM gains until scenarios U, I, V pass, against the thresholds of `0b0e39b`

- **Date / commit:** 2026-11-03, built on `93f7a82`. Follows
  [stage 5](2026-11-03-stage5-balance-in-sim.md), which wrote the scenarios and their thresholds
  (commit `0b0e39b`) before any tuning. Commits of this stage are listed under [Commits](#commits).
- **Branch:** `ai-forward`. Local commits only: this stage was told not to push.
- **Environment:** laptop (macOS, Python 3, standard library) for the linear analysis;
  `rocko-env-dev` container (`ros:jazzy`, arm64 on Docker Desktop) for `verify_sim.sh` and
  `verify_urdf.sh`. `./check_container_mount.sh` printed `ok    the container sees this checkout` at
  the start. This run was the only user of the container.

**Everything in this entry is about a model with estimated, unmeasured parameters
([sim-plant.md §4](../architecture/sim-plant.md)) and an assumed encoder sign.** U, I, V run with
`encoder_sign:=-1`, an **ASSUMPTION** nobody has checked on the robot
([proposal 0006](../proposals/0006-velocity-loop-sign-check.md), finding H-12), and with the modelled
20 ms IMU delay. The strongest claim this entry can make is "**balances in simulation under an
estimated model**". It says nothing about the robot, and no gain here is a recommendation for
`rocko_controllers.yaml`; that needs a candidate-gains proposal (a later task, not written here).

## Goal and success criteria

The person's goal: make scenarios U, I, V of `verify_sim.sh scenarios` pass by changing **sim gains
only**, in `rocko_env/rocko_env/bringup/config/rocko_sim_controllers.yaml`, against the thresholds
committed in `0b0e39b`, unchanged.

1. **The drift check allows exactly the tuned gains to differ.** `sim/check_sim_config.py` lets the
   sim YAML differ from `rocko_controllers.yaml` only in the gains it lists by name, still fails on
   structure, wiring, interface names and `update_rate`, fails on any other gain change in either
   file (including a change to a tuned gain's hardware value), and is shown red on deliberate breaks.
   Its own commit, before any gain changes.
2. **Candidates come from a script, not from guessing.** `sim/cascade_linearisation.py` gains a
   `--sweep` (grid over the five gains, linear stability, then a time simulation of U, I, V with
   SimPlant's write quirks) and a `--candidate` report (gain margins up and down, IMU delay range).
3. **Each change is confirmed in the nonlinear sim**, one gain at a time, inner loop first, at most
   8 iterations per scenario, each logged below as a row (change, metrics before and after, result).
4. **U, I and V pass** (`verify_sim.sh scenarios` exit 0 with S2b and AB "ok"), no threshold
   changed; or, if 8 iterations per scenario run out, a report of which fail and why.
5. **If they pass:** S2b still fails *for the right reason* with the tuned gains (a sign-flipped pitch
   loop falls within 1 s), AB still falls as H-12 predicts, the default `verify_sim.sh` (S1..FF)
   passes, `check_sim_config.py` and the host tests pass.
6. **Margins reported, not just verdicts:** the final gains' linear gain margins and delay margin,
   and each scenario's distance from its thresholds in the nonlinear sim.
7. The hardware YAML is untouched (`git diff 93f7a82 -- rocko_env/rocko_env/bringup/config/rocko_controllers.yaml` is empty).

## Plan

1. Extend `sim/check_sim_config.py` (criterion 1), with docs, in its own commit. Tier 0 (laptop and
   container); deliberate breaks on scratch copies with `--root`.
2. Extend `sim/cascade_linearisation.py` (criterion 2) without changing its existing report (the
   default output must be byte-identical before and after; checked with `diff`). Laptop.
3. Pick a target from the sweep, then walk to it one gain at a time in the nonlinear sim
   (`verify_sim.sh scenarios U I V`), inner loop (pitch) first, recording each run. Tier 3.
4. Commit the gains with this entry's results, separate from step 1. Then run S2b, AB, the default
   `verify_sim.sh`, `verify_urdf.sh` and the host tests.

Risks: the screen (a linear plant with the write quirks) and the nonlinear sim disagree, which costs
iterations; a threshold that cannot be met by any gain (then stop and report it, do not widen it);
a structural problem (journal it as a finding, do not tune around it); gains that pass the tuning
scenarios only because of a quirk of the model (the 30 ms held-out column and the margins are there
for that).

## 1. The drift check: only the tuned gains may differ

Before this stage `check_gains` compared only the gain-map keys that are not numbers
(`angle_wraparound` and similar). Every `p`, `i`, `d` and clamp could differ between the two files
unchecked, in either file: a gain edited in `rocko_controllers.yaml` passed silently, and so did an
accidental clamp change in the sim file.

Now (`SIM_TUNED_GAINS` in `sim/check_sim_config.py`):

- every number under `gains` must be equal in the two files, with its type, **except** the gains
  listed in `SIM_TUNED_GAINS`, as `(controller, key)`;
- each entry pins the **hardware** value it was tuned away from. If `rocko_controllers.yaml` changes
  that gain, the check fails: the sim gains were tuned against a cascade the robot's file no longer
  describes, and a hardware gain change is a human decision anyway;
- in the sim file a tuned gain must be a finite number and equal on both joints (the linear analysis
  treats the two wheels as one loop; two values would be a different controller);
- the wiring check (names, types, `dof_names`, interfaces, `update_rate`) is unchanged.

**Why a list of keys, not "any p, i, d":** each listed gain is one place where the sim tests a
cascade the robot does not run. A short, named list makes that visible in review, and makes an
accidental edit anywhere else red. The list was written from the sweep (section 2) before any gain
was changed: pitch `p` and `d`, velocity `p` and `i`. It does not have to differ (the check cannot
know whether a tuning run is in progress), so a listed gain that ends up equal to the hardware value
should be removed from the list.

**Deliberate breaks** (each on a fresh scratch copy of the tree outside the repo, run with
`python3 -I sim/check_sim_config.py --root <copy>`):

| Break | Result |
| --- | --- |
| Control: sim pitch `d` 0.1 → 0.8 on both joints (a listed gain) | all `ok`, exit 0 |
| Sim velocity `d` 0.2 → 0.3 (not listed) | exit 1: `gains setting differs at /velocity_pid_controller/gains/pitch_pid_controller/left_wheel_joint/d: rocko_controllers.yaml has 0.2, rocko_sim_controllers.yaml has 0.3` (and the right joint) |
| Sim pitch `i_clamp_max` 150 → 100, left joint only | exit 1: `gains setting differs at .../left_wheel_joint/i_clamp_max` |
| Sim pitch `i` 0.0 → 0.5 (not listed) | exit 1, both joints named |
| Sim pitch `d` 0.8 on the left joint only | exit 1: `pitch_pid_controller \`d\` differs between joints (...: 0.8, ...: 0.1); one loop, one gain` |
| Sim velocity `p` written as the string `"20"` | exit 1: `... is '20', not a finite number` |
| Hardware pitch `d` 0.1 → 0.2 (a listed gain) | exit 1: `rocko_controllers.yaml changed pitch_pid_controller \`d\` ...: 0.2, but the sim gains were tuned against 0.1 (SIM_TUNED_GAINS) ...` |
| Hardware velocity `d` 0.2 → 0.4 (not listed) | exit 1: `gains setting differs at .../d: rocko_controllers.yaml has 0.4, rocko_sim_controllers.yaml has 0.2` |
| Sim `update_rate: 100` → `200` | exit 1: `wiring differs at /controller_manager/ros__parameters/update_rate` |
| Sim `pitch_rate` renamed `pitch_rat` | exit 1: the allowed wiring difference no longer matches, and `wiring differs` |
| Sim velocity loop reading `["pitch"]` instead of `["velocity"]` | exit 1: `wiring differs at /velocity_pid_controller/ros__parameters/reference_and_state_interfaces` |
| Sim `angle_wraparound: true` added to one entry | exit 1: `gains setting differs at .../angle_wraparound` |

The real tree, after the change and before any gain change: six `ok` lines, exit 0, the gains line
ending `sim differs from hardware in: none`.

## 2. The sweep is a script: `sim/cascade_linearisation.py --sweep` and `--candidate`

What was added (laptop, standard library only; the default report is unchanged: its output was
`diff`ed byte for byte before and after the change, identical):

- `discrete_step(...)`: the 10 ms control cycle of the stage 5 model, now also with a velocity
  reference, a feedforward and a write path as inputs, and an IMU delay in whole **milliseconds**
  (SimPlant's resolution) instead of whole cycles: for a delay `j·10 ms + f`, the state carries the
  pitch `f` ms before each read, computed exactly from the zero-order hold. Its matrix is the linear
  model the stage 5 report already used.
- `margins(gains)`: the **gain margins**, up and down, as the factor on everything the motors deliver
  (`command_scale`: what a motor-gain error or a battery-voltage change does), found by bisection on
  the largest |z|. A balancing robot is open-loop unstable, so too little gain fails as well; both
  sides are reported. And the **IMU delay range**: every whole millisecond from 0 to 80 ms tried, the
  stable range containing the modelled 20 ms reported; delay margin = its upper end − 20 ms.
- `simulate(gains, "U" | "I" | "V")`: the scenario on the discrete model, with SimPlant's write path
  (`model_write_quirks`: 0.05 rad/s deadband, +2 % stiction offset, integer percent, the clamp) and
  the scenario thresholds copied by name from `verify_sim.sh`'s block. A **screen**, not a verdict: a
  linear plant, no open-loop gap after release, no container timing jitter.
- `--sweep`: 1800 grid points (pitch p 6-15, pitch d 0.2-0.6, velocity p 4-16, velocity i 0-5,
  velocity d 0-0.2), the largest |z| of each, the most damped screened in `simulate` (also at a 30 ms
  IMU delay, the held-out "one more control period" of `tune-gains`). `--candidate KP,KD,KPV,KIV,KDV`
  (or `yaml`): the report for one gain set. `--hardware-gains`: the stage 5 report with the hardware
  YAML's gains, so the H-12 diagnosis still reproduces now that the sim YAML differs.

**Check of the screen against the nonlinear sim, before tuning.** With the hardware gains the screen
says U oscillates at about 25 deg with |v| 0.86 m/s late, I falls at 7.4 s, V falls at 7.6 s; the
stage 5 nonlinear runs saw a 23-30 deg oscillation (U), a fall at 6.5 s (I) and at 6.7 s (V). Same
failure, same size, the times 1 s apart: good enough to rank candidates, not to replace a run.

**What the first sweep said** (`python3 -I sim/cascade_linearisation.py --sweep --top 25`, 68 s):
1287 of 1800 points are stable in the linear model; stability needs pitch d ≥ 0.2 and velocity p ≥ 4,
as the stage 5 map said. The 25 most damped all have velocity i = 0 and velocity p = 16 (the grid's
top): the slowest mode is the velocity loop's, and an integral adds a slower one. **V is the binding
scenario everywhere**: its overshoot limit (30 %).

**Why V's overshoot binds (an observation about the cascade, not a new finding).** With velocity i = 0
the steady speed does not settle at the reference but above it. In the slow-time-scale reduction of
sim-plant.md §2.4 (eq. 1, with `encoder_sign = −1`) the DC gain from the reference to the measured
velocity is `k·Kpv / (k·Kpv − c)` > 1, `k = K_u·Kp·0.5·r`, `c` the motor's back-EMF plus friction: the
same `c` that makes the loop need `k·Kpv > c` to be stable at all. With pitch p 12 and velocity p 20 that
is 1.24, a 24 % over-speed before any transient. DiffDriveController's feedforward (0.1 m/s / 0.06 m =
1.67 rad/s) adds to it: the closed loop turns a forward wheel push into more measured speed under
these signs (screen: velocity p 16, pitch p 12: overshoot 28 % with the feedforward, 19 % without).
An integral removes the steady error, but slowly (its mode is the slowest in the loop) and it adds to
the peak, so with the 30 % limit counted on the maximum sample over the 8 s step, the way to pass is
a large `k·Kpv`, i.e. large pitch p × velocity p. The threshold's own comment says it was checked at
pitch d 0.3, velocity p 8; the screen puts V's overshoot there at 79 % (U and I match their comments:
peak 3.4 deg, 0.6 s). So the V threshold was not checked against this effect; it is **not** changed
here (it binds, it is not shown impossible), and this is reported under *Thresholds* below.

## 3. The target, and the walk to it

**Target picked from the screen** (`--candidate`, section 2), not from a feeling: pitch d high (it is
the inner loop's damping; at 0.1 the 20 ms IMU delay leaves the pitch pair unstable, stage 5's −1.1 dB),
velocity i = 0 (in the screen the integral only added to V's overshoot) and pitch p × velocity p large
(V's steady over-speed `c / (k·Kpv − c)`). Among the points that pass U, I, V on the screen also at a
30 ms IMU delay, pitch d 1.0 was preferred to 0.8 for the IMU delay range (0-50 ms against 0-37 ms at
pitch p 15, velocity p 25), since the IMU delay is the least known number in the model.

**The walk: one gain per iteration, inner loop first**, each iteration one run of
`verify_sim.sh scenarios U I V` (so it counts once for each of U, I and V). The screen's prediction is
written before each run. "Before" for iteration 1 is the stage 5 run at `0b0e39b` with the hardware
gains. Times are seconds; pitch in degrees (exported, delayed, |value|); v is the mean measured velocity
[m/s]. Thresholds (from `0b0e39b`, unchanged): U late |pitch| ≤ 2.0 and |v| ≤ 0.05 from 5 to 10 s; I back
under 2.0 deg within 3.0 s of the push; V 90 % within 3.0 s, overshoot ≤ 30 %, |pitch| < 10.

| It. | Change (sim YAML) | Gains after (pitch p, d · velocity p, i, d) | Screen predicted | U (nonlinear sim) | I (nonlinear sim) | V (nonlinear sim) |
| --- | --- | --- | --- | --- | --- | --- |
| 0 | none (stage 5, hardware gains) | 10, 0.1 · 2, 5, 0.2 | U oscillates ~25 deg; I, V fall | FAIL: late \|pitch\| 30.25 | FAIL: fell 4.51 s after the push | FAIL: fell 5.65 s after the step |
| 1 | pitch d 0.1 → 0.8 | 10, 0.8 · 2, 5, 0.2 | U pass (0.175, 0.0136); I settles 4.88 s; V overshoot 186 % | **PASS**: 0.150, 0.0132 | FAIL: settled 4.97 s (peak 2.72) | FAIL: overshoot 198.6 %, rise 1.99 s |
| 2 | velocity p 2 → 20 | 10, 0.8 · 20, 5, 0.2 | U pass (0.069, 0.0015); I 0.97 s; V 35.7 % | PASS: 0.064, 0.0019 | **PASS**: 1.02 s (peak 4.67) | FAIL: overshoot 51.4 %, rise 0.89 s |
| 3 | pitch d 0.8 → 1.0 | 10, 1.0 · 20, 5, 0.2 | V 36.3 % | PASS: 0.055, 0.0025 | PASS: 1.04 s (peak 4.27) | FAIL: overshoot 63.1 %, rise 0.90 s |
| 4 | pitch p 10 → 15 | 15, 1.0 · 20, 5, 0.2 | V 25.6 % | PASS: 0.048, 0.0017 | PASS: 0.91 s (peak 4.68) | FAIL: overshoot 38.5 %, rise 1.02 s |
| 5 | velocity i 5 → 0 | 15, 1.0 · 20, 0, 0.2 | V 17.9 % | PASS: 0.052, 0.0014 | PASS: 0.76 s (peak 4.33) | **PASS**: overshoot 23.5 %, rise 1.13 s. Repeats, no change: **31.6 % FAIL**, 26.2 %, 26.8 % |
| 6 | velocity p 20 → 25 | **15, 1.0 · 25, 0, 0.2** | V 13.9 % | PASS: 0.048, 0.0013 | PASS: 0.77 s (peak 6.01) | PASS: overshoot 23.5 %, rise 0.95 s. Repeats, no change: 25.4, 26.3, 24.2, 29.1 % (all PASS) |

**Count, honestly:** 6 tuning iterations, each a run of all three, so 6 of the allowed 8 for each of U,
I and V. U first passed at iteration 1, I at 2, V at 5. Iteration 6 was spent on V's margin after the
repeats at iteration 5 failed 1 run in 4. Not counted as iterations: the 7 V repeats with no gain
change, and the final full run below (same gains).

**Why iteration 6 and not more.** At iteration 5 V passed 3 runs of 4. Iteration 6 moved its mean
overshoot from 27.0 % (4 runs) to 25.3 % (6 runs, 23.5 to 29.1 %). More margin would need a larger pitch p
(screen: pitch p 20 gives 10.3 % but cuts the IMU delay range from 0-50 to 0-35 ms), i.e. trading the
least-known number's margin for a verdict made noisy by the verifier itself (next paragraph). I
stopped there and report V as narrow.

**The screen against the sim.** U and I agreed with the screen to within a few hundredths of a degree
and a tenth of a second at every step. V's overshoot was always **higher** in the sim than on the
screen, by +5.6 to +26.8 points, and it varied from run to run at fixed gains (23.5 to 31.6 % at
iteration 5). Looking at the samples:

- The measured velocity in the sim is noisy where the screen's is smooth: at iteration 3, after 6 s,
  mean 0.111 m/s with a standard deviation of 0.0052 m/s (4.7 %); the samples' stamps are 3.3 to 28 ms
  apart. The likely cause is the encoder replica (finding U-2, copied from `QuadEncoders.py` on purpose):
  it divides each cycle's travel by an assumed 0.01 s, so a cycle that `controller_manager` runs late
  in the container reads as a faster wheel. **Not measured directly** (the control periods were not
  recorded).
- V's overshoot is the **largest single sample** after the step. In the run that failed at iteration 5
  (31.6 %) the largest sample came at 8.86 s, a one-sample spike (0.119, 0.132, 0.126 m/s), not the
  step response; its 20-sample mean peaked at 0.125 (25 %). That run's launch log also has two
  `teleop command timed out` lines while the gamepad was driving (the container's delivery stalls, as
  in the C3/FF journal).

So part of V's distance from its threshold is spent on noise the threshold did not plan for. Not
changed here (see *Thresholds*).

## 4. Results with the final gains

**Final gain table (sim YAML, against the MODEL, `encoder_sign:=-1` ASSUMED; not a recommendation for
the robot):**

| Loop | Gain | Hardware (`rocko_controllers.yaml`, unchanged) | Sim (`rocko_sim_controllers.yaml`) | Unit |
| --- | --- | --- | --- | --- |
| `pitch_pid_controller` | p | 10.0 | **15.0** | (rad/s) per deg of pitch error |
| | i | 0.0 | 0.0 | |
| | d | 0.1 (on the delayed angle) | **1.0** (on the gyro rate, H-6, sim only) | (rad/s) per deg/s |
| | i_clamp | ±150 | ±150 | rad/s |
| `velocity_pid_controller` | p | 2.0 | **25.0** | deg of target pitch per m/s |
| | i | 5.0 | **0.0** | deg per m |
| | d | 0.2 | 0.2 | deg s² per m |
| | i_clamp | ±150 | ±150 | deg |

The hardware pitch d differs in *kind* as well as size: it differentiates the delayed angle, the sim's
reads the gyro (finding H-6), so 0.1 → 1.0 is not a like-for-like step for the robot.

**Linear model, final gains** (`python3 -I sim/cascade_linearisation.py --candidate yaml`, 20 ms, encoder
sign −1):

| Margin | Value |
| --- | --- |
| Largest \|z\| per 10 ms cycle | 0.98375 (slowest mode decays at 1.64 /s) |
| Gain margin, up | ×3.20 (+10.1 dB) on everything the motors deliver |
| Gain margin, down | ×0.123 (−18.2 dB): below that, too little gain to hold the pendulum |
| IMU delay | stable from 0 to 50 ms: **delay margin 30 ms** over the modelled 20 ms |
| Held out on the screen | at a 30 ms IMU delay U, I, V all pass (V 13.9 %) |

For comparison, the hardware gains (same model): unstable, gain margin about −1.1 dB on the pitch loop
alone (stage 5).

**Nonlinear sim, final gains, distance from each threshold** (the full run below, plus the V repeats):

| Scenario | Measured | Threshold | Margin |
| --- | --- | --- | --- |
| U late \|pitch\| | 0.048 to 0.050 deg (2 runs) | ≤ 2.0 deg | about 40× |
| U late \|v\| | 0.0013 m/s | ≤ 0.05 m/s | about 38× |
| I settle time | 0.773 to 0.779 s (2 runs) | ≤ 3.0 s | 3.9× |
| I peak \|pitch\| | 6.0 to 6.5 deg | fall at 35 deg | 29 deg |
| I push seen | 98.8 to 99.9 deg/s | ≥ 80 deg/s | the push was applied |
| V rise to 90 % | 0.91 to 0.96 s (6 runs) | ≤ 3.0 s | 3.1× |
| V overshoot | **23.5 to 29.1 %** (6 runs) | ≤ 30 % | **0.9 to 6.5 points: narrow** |
| V peak \|pitch\| | 4.06 to 4.13 deg | < 10 deg | 2.4× |
| S2b fall (pitch sign flipped) | 0.142 s | within 1.0 s | right reason: the pitch diverged (linear model: +55 /s) |
| AB fall (as-built signs) | **3.801 s** | no sooner than 3.0 s | **0.8 s: narrow** (see *Thresholds*) |

**S2b still fails for the right reason:** with `pitch_sign:=1` and the tuned gains, U fell at 0.142 s
(0.194 s with the hardware gains): faster, because the flipped pitch loop now has a larger gain, and far
inside the 1.0 s that separates a sign error from a slow drift. **AB still falls the H-12 way:** pitch at
1, 2, 3 s: 0.72, 2.78, 11.3 deg, a one-way run-away (no oscillation), then lying at 60 deg. It falls
sooner than with the hardware gains (3.80 s against 9.40 s) because a larger velocity p feeds the
positive feedback harder: the linear model's run-away pole moves from +0.82 /s to +1.40 /s, which is
what stage 5's table C predicted for more velocity gain.

## Verification

Commit at the time of the runs: `1c042c7` with the working tree carrying exactly the changes later
committed as `4622b09` (the script) and the gains commit (the sim YAML, `SCENARIOS_EXPECTED_RED`); no
other file that a run reads differed. `SCENARIOS_EXPECTED_RED` was still 1 during the iterations (it
changes only a FAIL's wording) and 0 for the final run; the `SIM-TUNED` comments in the YAML were added
after the runs (comments only).

| Tier | Command | Result |
| --- | --- | --- |
| — laptop | `python3 -I sim/cascade_linearisation.py` before and after the script change | outputs identical (`diff`), self-checks passed |
| — laptop | `python3 -I sim/cascade_linearisation.py --sweep --top 25` | 1800 points, 1287 stable; 68 s |
| 0 static | `python3 -I sim/check_sim_config.py` (host) | six `ok`, exit 0; gains line ends `sim differs from hardware in: pitch_pid_controller d, pitch_pid_controller p, velocity_pid_controller i, velocity_pid_controller p` |
| 0 static | `verify_urdf.sh` (container) | `PASS: all seven expansions ... and the sim config has not drifted.`, exit 0 |
| 0 static | `.claude/hooks/test-guard-protected-files.sh` (host) | 81 cases, exit 0 |
| 0 static | deliberate breaks of the gains check | section 1: 11 breaks red (exit 1), control green |
| 1 build | `build_ros.sh` (container), at the start | `Summary: 2 packages finished`, exit 0 (no C++ changed in this stage) |
| 2 mock | `verify_mock.sh`, `verify_mock.sh tank` | **not run**: no file the mock loads changed (`rocko_controllers.yaml` and the tank YAML are untouched) |
| 3 sim, scenarios | `verify_sim.sh scenarios` (all five), final gains | **`SCENARIOS: PASS (exit 0)`**, 82 s: U, I, V `PASS`; S2b `ok` (0.142 s); AB `ok` (3.801 s) |
| 3 sim, default | `verify_sim.sh` (S1 S2a H4 C3 C2 H6 FF), final gains | **`PASS (simulation: S1 S2a H4 C3 C2 H6 FF)`, exit 0**, 159 s |
| — | `git diff 93f7a82 -- rocko_env/rocko_env/bringup/config/rocko_controllers.yaml` | empty: the hardware YAML is untouched |

The guard hook never refused an edit in this stage; `rocko_sim_controllers.yaml` is not a guarded file.
One edit of the sim YAML (iteration 1) was made with `sed -i` from Bash rather than the Edit tool, by
habit, not to get round anything (no edit had been refused); every later edit used Edit.

**Reproducibility record** ([reproducing-results.md](../guides/reproducing-results.md)):

```
Result:        scenarios U, I, V pass; S2b, AB ok (final full run); V overshoot 23.5-29.1 % in 6 runs
Date:          2026-11-03
Commit:        the gains commit of this stage (runs made on 1c042c7 + exactly the changes of 4622b09 and that commit)
Launch:        verify_sim.sh scenarios -> rocko_sim.launch.py, per scenario: teleop:=false (U, I, S2b, AB) or
               teleop:=true (V); encoder_sign:=-1 (U, I, V, S2b; ASSUMPTION) or 1 (AB); initial_pitch_deg:=2.0
               (U, S2b, AB); impulse_time_s:=2.0 impulse_angular_velocity_deg_per_s:=100 (I); pitch_sign:=1 (S2b);
               every other argument at its default (imu_delay_ms 20, model_encoder_node true)
Hardware:      SimPlant (simulated); not the mock, not the robot
Config:        rocko_sim_controllers.yaml: pitch p 15.0 d 1.0, velocity p 25.0 i 0.0 (the rest as the hardware file)
Battery:       n/a (sim; SimPlant's battery_voltage_V default 12.0)
Surface:       n/a (sim; no floor model)
Robot:         n/a (sim)
Payload:       n/a (sim)
Logs:          passing runs delete their directory (verify_sim.sh); failing V repeats kept inside the container
               under /tmp/tmp.* until it restarts; the printouts are quoted above
Observed:      see sections 3 and 4
```

## Thresholds (none changed; three things a human should know)

1. **V's overshoot limit is met narrowly and its verdict is noisy.** It judges the largest single sample
   of a measured velocity that carries the encoder replica's timing noise (section 3), worth about
   +10 points in these runs. The threshold's comment says the thresholds were checked at pitch d 0.3,
   velocity p 8; the screen gives V an overshoot of 79 % there (U and I match their comments), so V's
   limit was not checked against the over-speed of section 2. If a human thinks the limit or the metric
   should change (for example, judge a moving average, or allow for the replica's noise), that is its own
   commit with a journal paragraph, made **before** any further tuning; it was not done here.
2. **AB's 3.0 s floor depends on the gains.** A larger velocity p makes the H-12 run-away faster; at the
   final gains AB falls at 3.80 s. On the screen (AB's signs, same start) the fall comes at 3.77 s with
   velocity p 25 (the sim: 3.80 s), 3.26 s at 30 and 2.88 s at 35 (9.77 s with the hardware gains; the
   sim: 9.40 s); so a velocity p above about 33 would turn AB red although its cause is still H-12.
   Screen estimates, not sim runs. Recorded, not changed.
3. S2b's 1.0 s still separates cleanly (0.142 s).

## Not verified — needs a human (or a later task)

- **Anything about the robot.** These gains balance in simulation under an estimated model. The model's
  parameters are estimates ([sim-plant.md §4](../architecture/sim-plant.md)); the encoder sign is an
  assumption (proposal 0006); the 20 ms IMU delay is an estimate.
- **The held-out set in the sim** (`tune-gains`: another initial pitch, ±20 % on mass and friction, one
  more control period of delay): not run in the nonlinear sim. The ±30 % sweep of sim-plant.md §4.3
  was run afterwards, in [stage 5c-2](2026-11-03-stage5c2-sweep.md).
- **The sim-to-real gap for these gains** (sim-plant.md §6). Optimistic in the sim: the gyro has no
  noise, so a pitch d ten times the hardware value costs nothing here, while on a robot it multiplies
  gyro noise into the wheel command; no backlash, no software-PWM aliasing, no battery sag (the gain
  margin down, ×0.123, is the model's answer to a weaker motor, nothing more). In I the command peaks at
  about 100 rad/s against the 119 rad/s clamp.
- **Velocity i = 0** means nothing removes a steady velocity error: in V the measured speed settles above
  the reference (0.114 m/s on the screen at 9 s for a 0.1 m/s step).
- The `COULD NOT JUDGE` / exit 2 path of the scenarios mode (still never exercised).
- Review: the reviewer agents were not run in this stage (the task did not include them; no §6 runtime
  file changed, but `rocko_sim_controllers.yaml` is a `bringup/config/*.yaml` file, so a later
  `realtime-auditor` and `ros2-control-reviewer` pass should look at it). Waiting for
  `explainability-reviewer` (the two scripts), `docs-reviewer` and `ros2-control-reviewer`.
- No candidate-gains proposal is written: the gains stay in the sim YAML until one is (a later task).

## Reproduce

Laptop, repo root:

```bash
python3 -I sim/check_sim_config.py                          # six ok, exit 0
python3 -I sim/cascade_linearisation.py --candidate yaml    # margins and screen of the sim YAML's gains
python3 -I sim/cascade_linearisation.py --sweep --top 25    # the sweep, about 70 s
python3 -I sim/cascade_linearisation.py --hardware-gains    # the stage 5 (H-12) report
```

Container (start it from this checkout, `./check_container_mount.sh` must print `ok`):

```bash
docker compose -f docker/compose.yml exec -T ros2 bash -c 'cd /ros2_ws && src/ROCKO-env/build_ros.sh'
docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_sim.sh scenarios   # exit 0 expected; V narrow
docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_sim.sh
docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_urdf.sh
```

## Commits

- `1c042c7`: the drift check (section 1), with this entry's first version. No gain changed.
- `4622b09`: `sim/cascade_linearisation.py`'s tuning modes (section 2). No gain changed.
- The gains commit: the sim YAML, `SCENARIOS_EXPECTED_RED=0`, the status docs and this entry's
  results: the commit that adds this section.

**Why the run stopped here:** U, I and V pass in 6 iterations each (of 8), with S2b, AB, the default
run, the drift check and the host tests green. The remaining work is a human's or a later task's: the
held-out runs in the sim, the reviewers, a decision on V's metric, and a candidate-gains proposal.
