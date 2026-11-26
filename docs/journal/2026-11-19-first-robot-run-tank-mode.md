# First run on the robot since `ai-forward`: tank mode, and where it stands

- **Date / commit:** 2026-11-19. The owner ran "the current code on the `ai-forward` branch" on the Pi; the
  head at that time was `df91b9e` (the owner's "turn off precise mode"), on top of `7b4d20e`, `0158991`
  and `5d1119c`. Which exact commit was checked out on the Pi was **not recorded**.
- **Branch:** `ai-forward`
- **Environment:** the Pi (Ubuntu 24.04, ROS Jazzy, WiringPi, the motor plugin built there) and the owner's
  macOS laptop (`keyboard_teleop.py`, Foxglove, a browser). Everything below about the robot is the owner's
  report in chat; **an agent saw none of it** (CLAUDE.md section 4, Tier 4).
- **Why this entry exists:** it is the handoff for a session that is being closed. A new session starts
  with the *Next* section below and [next-steps.md](../next-steps.md).

## What was asked, in order

1. Why the wheels only hummed with the keyboard in tank mode (not PID, not the watchdog: see
   [proposal 0013](../proposals/0013-tank-mode-motor-breakaway.md)).
2. A bench test ([guide](../guides/bench-test-tank-mode.md)), then root helper scripts
   ([guide](../guides/run-on-the-pi.md); [journal](2026-11-19-pi-helper-scripts.md)).
3. Close the session with everything logged and a plan for what comes next: Xbox controller over Bluetooth on
   macOS, then self-balancing.

## What the owner reports from the robot

| Item | Result (reported 2026-11-19) |
| --- | --- |
| Tank mode launched with `pi/launch.sh tank` | works; "the helper scripts worked" |
| Driving from the keyboard | **works: the robot drives** |
| Foxglove | works (odom, the model; odom is integrated from the *commands* in tank mode, `open_loop: true`, so it says nothing about wheel speed) |
| Web dashboard | "most features" work. **Which did not is not recorded** |
| Motors | **still hum a lot.** The owner's suspicion: much of the time is spent close to stall |
| Bench test (earlier the same day) | left wheel first turned at `linear.x` = 4.0 m/s (28 % duty), right at 3.0 m/s (21 %); at 4.0 the left was "barely moving" |

**Not recorded, so unknown:** the commit on the Pi, battery voltage (start and end), the surface, whether
the wheels were on the ground, how long it ran, motor or driver temperature, which dashboard features
failed, the encoder velocities, a video, and where any log went. Per
[reproducing results](../guides/reproducing-results.md) these make the run an anecdote, not a result; the
next run should fill them in.

## Why the hum is plausible, from the code (arithmetic, not measured)

`df91b9e` turned the keyboard's "precise mode" off and made the keys ten steps, so each key level goes
through the gamepad's mapping (`expo_curve(level / 10) x 10 m/s`) and then the motor plugin
(`duty = 6.5957 x speed + 2`, truncated, `Motor12VoltQuadEncoder.cpp:231`):

| Key level | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | 9 | 10 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Duty [%] | 5 | 9 | 13 | 18 | 23 | 30 | 37 | 46 | 56 | 67 |

With breakaway at 28 % (left) and 21 % (right): levels 1 to 4 only hum, level 5 turns the right wheel
alone, and level 6 turns both just past breakaway. A person gently driving spends their time at levels 5 to 7,
a few points above breakaway, which is the "close to stalling" region the owner suspects. A stalled or
nearly stalled brushed motor draws far more current than a running one and heats up: **nobody has measured
current or temperature.** The 100 Hz software PWM at low duty is also audible by itself. This is a hypothesis
that fits the numbers; the bench data do not separate it from "the supply sags" or "the driver has a dead zone".

## What changed in the repo today (all pushed to `ai-forward`)

| Commit | Change |
| --- | --- |
| `5d1119c` | the [bench-test guide](../guides/bench-test-tank-mode.md) and [proposal 0013](../proposals/0013-tank-mode-motor-breakaway.md) |
| `0158991` | `pi/` helper scripts, `build_ros.sh` takes the checkout as the workspace, `tools/pi_scripts_test.sh`; docs say `~/ROCKO-env`, not `~/ros2_ws` ([journal](2026-11-19-pi-helper-scripts.md)) |
| `7b4d20e` | tank mode starts the dashboard by default (the owner's edit, kept at their request); the bench results recorded in proposal 0013 |
| `df91b9e` | **the owner's:** `keyboard_teleop.py` precise mode off, `STEPS_TO_FULL` 5 to 10, re-wrapped |
| this commit | this entry, the keyboard guide corrected to `df91b9e`, [next-steps.md](../next-steps.md) rows and handoff, the [ladder](../architecture/verification-ladder.md)'s Tier 1 and 4 lines, the Xbox controller plan (removed 2026-11-26; in git history) |

## Known red, on purpose left red

`df91b9e` is a deliberate design change, and two checks still encode the old design. I did not edit the
owner's code or loosen the checks to make them pass.

| Check | Result on `df91b9e` | Why |
| --- | --- | --- |
| `python3 -I tools/keyboard_teleop_test.py` | **4 of 5**: `test_packet_is_a_gamepad_packet_in_precise_mode` fails ("only the precise-mode button is set") | the script no longer sets button 15 |
| `verify_sim.sh scenarios KB` (container, run 2026-11-19; output not saved, run directory kept in the container) | **FAIL**: speeds 0.5545, 1.136, 1.7715 m/s are "off the 0.1 m/s grid" | one press is now 0.55 m/s, not 0.1 |

The other ladder steps passed earlier today on `7b4d20e` (the table printed by `tools/run_ladder.sh --no-sim` was not saved to a file; the run is in the session transcript only) (`tools/run_ladder.sh --no-sim`: build, URDF, mock
in both modes, every host test but the keyboard one, links). Tier 3 as a whole was **not** rerun (only `KB`).
Fix when the keyboard's design settles (next-steps row 4.7): either restore precise mode with a tank-mode scale
(proposal 0013, option B), or rewrite both checks for the gamepad mapping.

## Reproduce this run

```bash
# Pi, from ~/ROCKO-env (see docs/guides/run-on-the-pi.md)
./build_ros.sh
pi/launch.sh tank
# laptop, from the repo root
python3 joystick/keyboard_teleop.py <robot-ip>
```

Record the battery voltage, surface and commit as the [reproducing-results](../guides/reproducing-results.md)
block asks; none was recorded this time.

## Next

Two jobs, in the owner's order. The details are in [next-steps.md](../next-steps.md) (the *Handoff* box at the
top and rows 4.5 to 4.7).

1. **Xbox controller over Bluetooth on macOS** (row 4.5). Plan, unknowns and a first step the owner can do in
   ten minutes: the plan, `docs/claude-analysis/xbox-controller-macos-plan.md` (superseded, and removed 2026-11-26; it is in git history). Settle the low-duty problem
   first or alongside (row 4.6, proposal 0013): a stick passes through the 0 to 28 % band on every push, and a
   stick that does not rest at exactly zero will hum the motors.
2. **Self-balancing.** Nothing about balancing was run today, and tank mode says nothing about it (no
   gyro, no pitch loop). The path is the existing queue, not re-derived, in the order of
   [proposal 0008](../proposals/0008-candidate-gains-and-tethered-test.md)'s preconditions: the velocity-loop sign check
   on a stand ([proposal 0006](../proposals/0006-velocity-loop-sign-check.md), finding H-12); measure the robot
   (rows 0.7 and 2.2, [guide](../guides/measure-inertia.md), proposal 0007); verify the gyro axis, sign and unit on
   the robot and build the `read_pitch_rate` wiring on the Pi (0008, preconditions 3 and 4); confirm the safety
   thresholds ([proposal 0004](../proposals/0004-safety-thresholds.md)); then the first tethered test with
   candidate gains (0008). The C-1 IMU/encoder read in C++
   ([proposal 0010](../proposals/0010-c1-imu-uart-and-encoders-in-cpp.md), row 1.1, needs a wiring change) is
   recommended before trusting balance timing but is not a 0008 precondition.
   Today's bench result adds one input: **the motors have a dead zone of 21 to 28 % duty**, which a balance loop
   that commands small corrections around zero will feel (the plugin adds only 2 % today; proposal 0013 is where
   this is decided). Findings [L-7](../claude-analysis/balance-loop-findings.md) (integer division in
   `MAX_RAD_PER_SEC`, about 1 %) and H-1 (open-loop velocity) are related.

## Open decisions for the owner

- Proposal 0013: option A (a per-motor stiction duty, left 28 / right 21), B (a larger keyboard scale in tank
  mode), A and B together, or C (nothing). **Open.** My recommendation is A and B together, after one more
  bench measurement (encoder velocity at 4.0 and 5.0 m/s, battery voltage under load).
- What the keyboard should be now that precise mode is off (row 4.7).
- Whether to keep the dashboard on by default in tank mode: it has no login, and its CPU cost on the Pi is
  unmeasured.
