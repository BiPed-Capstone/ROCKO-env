# Proposal 0013 — make a small drive command start the wheels turning in tank mode

- **Status:** Open
- **Gate:** on-robot step (a bench measurement) and a hardware limit (a motor output constant in a
  critical-path file, `Motor12VoltQuadEncoder.cpp`/`.hpp`, section 6 of [CLAUDE.md](../../CLAUDE.md))
- **Raised:** 2026-11-18, commit `0c6fb98` (the working tree also held an uncommitted edit to
  `rocko_tank_mode.launch.py` that this proposal does not depend on), run journal: none (documentation
  only). Related findings: H-2 and M-3 in
  [balance-loop-findings.md](../claude-analysis/balance-loop-findings.md) (the write path this proposal
  changes).

## The problem, as seen

First run of `rocko_tank_mode.launch.py` on the robot, driven with `joystick/keyboard_teleop.py`: the
keys change the numbers on the laptop, the odom object moves in Foxglove, and the motors **hum**,
with an occasional twitch of the wheels. The wheels do not turn.

## What the code does with a key press

Read, not run on a robot. The chain (the numbers are worked out in the
[bench-test guide](../guides/bench-test-tank-mode.md)):

1. `keyboard_teleop.py` sends "precise mode". `Joystick.py` turns a full axis into 0.5 m/s forward
   (`DEFAULT_PRECISE_LINEAR_SCALE_M_S`, `Joystick.py:47`), so each `w` is 0.1 m/s.
2. `diff_drive_controller` divides by the 0.127 m wheel radius: 0.79 rad/s per key press, 3.9 rad/s at
   the top.
3. `Motor12VoltQuadEncoder::write` (`Motor12VoltQuadEncoder.cpp:231`) makes duty = speed / 119.4 rad/s
   x 100 + `kStictionDutyPercent` (2.0), as a whole percent: **2 % for one press, 5 % for five**.
4. Software PWM at those duties is a 0.2 to 0.5 ms pulse every 10 ms. A hum, not a turn.

Things that are *not* the cause, and why:

- **PID.** Tank mode has none. Gains in `rocko_tank_mode_controllers.yaml` are the stock
  `diff_drive_controller`'s limits; nothing there adds a feedback term.
- **The watchdog.** The 0.2 s teleop watchdog is in `DiffDriveController.py`, which tank mode does not
  launch. Tank mode has only `cmd_vel_timeout: 1.0 s`, and the keyboard resends every 0.05 s. A
  watchdog cut would give 0 % duty and silence.
- **Odom moving.** `open_loop: true` (`rocko_tank_mode_controllers.yaml:29`) integrates odom from the
  *commands*, so it moves whenever commands arrive, whether or not a wheel turns.

The same chain with the gamepad's mapping asks for up to 10 m/s (67 %, see the guide's table), which is why the
motor code looks reasonable when driven by a gamepad. The keyboard's precise mode was sized for the
simulated balancing robot (scenario `K`), not for tank mode, and never run on a robot.

## Why

The goal is to get tank mode, the first thing run on the robot on this branch, to move the wheels from
the keyboard, so the rest of the first-run checks (direction, encoders, odom) mean something. Without
a change, only a command far above the keyboard's range moves the wheels, and a student following
[drive-with-keyboard.md](../guides/drive-with-keyboard.md) sees a hum. The turn keys are weaker still:
a full `a` or `d` is `angular.z` 0.3 (`Joystick.py:233`), 0.048 m/s per wheel, about 2 % duty;
in tank mode `diff_drive_controller` reads it as rad/s (the `Joystick.py:49` comment is true only for
balance mode's `DiffDriveController.py`).

## The change (a choice, after the bench test)

**Step 0 (a human, no code change):** run the
[bench test](../guides/bench-test-tank-mode.md) and record, per wheel, the command at which the
wheel starts and the one at which it turns steadily. Convert to duty with the table in the guide. Call the
steady value of the weaker wheel `D_move` (percent).

Then choose one of:

### Option A (recommended): a per-motor stiction duty, tank mode only

Add an optional hardware parameter `stictionDutyPercent` to `Motor12VoltQuadEncoder`, default **2.0**
(exactly today's behavior, so `rocko.launch.py` is unchanged). `rocko_tank_mode.ros2_control.xacro`
sets it for both motors to `D_move` (rounded up, plus a margin of your choosing, written next to the
number). Exactly: `Motor12VoltQuadEncoder.hpp:62` `static constexpr double kStictionDutyPercent = 2.0;` becomes
a member `double stictionDutyPercent_ = 2.0;` that `on_init` (`.cpp:93-95`, beside `invert`) overwrites
when the `stictionDutyPercent` parameter is present; `.cpp:232` uses the member in place of the constant;
the two `<param>` lines go in `rocko_tank_mode.ros2_control.xacro` (lines 36-38 and 49-51 of the real-hardware
block). The value in the xacro is `D_move` from the bench test, not a number chosen here. The deadband
(`kDeadbandRadPerSec`, 0.05 rad/s) stays: a zero command still writes 0 %.

Effect (arithmetic only): with `D_move` = 15 %, one key press gives 15 %, five give 18 %. The
keyboard then moves the wheels at every level but changes their power by only 3 percentage points
across the five levels. A coarse speed range is the price of one number for
"start moving" and "speed", and it is the existing design (an offset plus a proportional term).

- **Why per motor, not a new global constant:** in balance mode the pitch loop's output crosses zero
  all the time. A 15 % offset on every nonzero command would make the wheels jump from -15 % to +15 %
  through the deadband; that could start a limit cycle (the chatter around zero) and nobody can check
  it without a robot. Keeping the default at 2.0 leaves balance mode as it is.
- **Why a hardware parameter, not a ROS parameter:** hardware plugin parameters are read once in
  `on_init`, off the 100 Hz path; this follows how `invert` is passed today.

### Option B: change the keyboard's scale for tank mode only

Pass `precise_linear_scale_m_s` and `precise_angular_scale_m_s` to the joystick node in
`rocko_tank_mode.launch.py`, and raise `MAX_PRECISE_SCALE_M_S` (`Joystick.py:45`, now 1.0, a
safety clamp: this loosens it, which is why it is a proposal). With the cap at 2 m/s the top key level
is about 15 %. It does not fix the shape of the problem: the first key press (0.4 m/s, 4 %) is still below
breakaway. Not recommended unless A is declined.

### Option C: do not change anything

Use the gamepad (`computer_joystick.py`) in tank mode, which already asks for larger commands. Choose
this if the bench test finds `D_move` close to what the gamepad gives anyway, or if tank mode is only a
wiring check for your purposes.

## Evidence

- **Reached:** none of the verification ladder. This proposal comes from reading
  `Joystick.py`, `Motor12VoltQuadEncoder.cpp/.hpp`, `rocko_tank_mode_controllers.yaml`,
  `rocko_tank_mode.ros2_control.xacro` and `keyboard_teleop.py`, and from arithmetic on them.
- **Not verified:** that the wheels do not turn at 2 to 5 % (the user reports humming; nobody has
  measured the duty or the voltage). That `D_move` is anywhere near 15 %; that number is an example.
  That the motor plugin's PWM period is 10 ms (it follows WiringPi's `softPwm`, which uses 100
  microsecond steps; not measured here). The occasional twitch has no explanation yet: candidates are
  the duty rounding between whole percents, software-PWM scheduling on the Pi, and the plugin's blocking
  encoder call (`Motor12VoltQuadEncoder.cpp:186`, a service call inside `read()`). The bench test does not
  separate these.
- **Option A would touch a section-6 file.** If accepted, the change needs the `realtime-auditor`, a
  check that `check_motor_plugin_compiles.sh` still passes (type-checks only; the plugin is not
  compiled in the container), and `verify_urdf.sh` run on both modes, since the xacro changes. None of
  that has been run, because nothing has been changed.

## Risk

- The bench test spins the wheels in the air with a 12 V supply: keep fingers and cables clear, and
  keep one hand on the supply switch. Every command in it stops by itself after 5 seconds.
- Option A, as written, changes what a "small" command does in tank mode only. A stiction value that
  is too high makes the wheel jump to a speed larger than the command asks for. Start from the lowest
  value at which both wheels turn steadily, not from the first value at which one twitches.
- A related defect worth fixing in the same change, but not part of the decision: `MAX_RAD_PER_SEC =
  1150 / 60 * M_PI * 2` (`Motor12VoltQuadEncoder.hpp:54`) divides integers, giving 119.4 rad/s and not
  120.4. The error is about 1 %.

## Test for a human

1. Run the [bench test](../guides/bench-test-tank-mode.md), wheels off the ground. Fill in its results
   block below.
2. If Option A: after the change, repeat the bench test. Success looks like: both wheels turn
   steadily at one `w` press from the keyboard (`python3 joystick/keyboard_teleop.py <robot-ip>`), stop
   at `space`, and keep a constant speed at a steady key level. Abort if a wheel keeps turning after
   the launch exits, if either motor gets hot, or if a wheel speeds up without a key press.
3. Only then put the robot on the floor, with a person next to it and a hand on the supply switch.
4. Record the outcome in the format from [reproducing results](../guides/reproducing-results.md):
   commit SHA, launch file and arguments, battery voltage, surface, where the log went.

**Bench results** (reported in chat by the owner on 2026-11-19; the wheels were off the ground; step 3 of
the guide). Duty = `6.5957 x + 2`, truncated:

| | left wheel | right wheel |
| --- | --- | --- |
| Lowest `linear.x` at which it turned | 4.0 m/s | 3.0 m/s |
| Duty at that command | 28 % | 21 % |
| Owner's note | "barely moving" at 4.0; "should be a little more" | |

Read this way (the report was "left wheel is 4 and right wheel is 3": taken as the breakaway command of
each wheel; **to be confirmed**). Not given, so not known: the encoder velocities, direction and turn
tests, battery voltage, surface, commit. What follows from it:

- Breakaway is about **21 to 28 %** duty, not the 15 % used as an example above, and the two motors differ
  by 7 percentage points. With one command for both, the right wheel runs ahead of the left, and the
  robot would curve. A per-motor stiction value (option A, which already sets it per motor in the xacro)
  fixes that; a single global number would not.
- Option A alone does **not** give "a little more": with stiction 28 % (left) and 21 % (right), one key
  press gives 28 % and 21 % (just at breakaway) and five give 31 % and 24 %. Every key level would still
  be "barely moving". The keys also need a larger range, which is option B's job: with the cap at 1.0 m/s
  a full key is +6.6 percentage points (left 34 %, right 27 %). Whether that is "a little more" depends on
  how fast the wheels turn per percent of duty above breakaway, which this test does not give: read the
  encoder velocity (terminal B of the guide) at 4.0 and 5.0 m/s for both wheels and add it here.
- **Robot run, 2026-11-19 (owner's report; [journal](../journal/2026-11-19-first-robot-run-tank-mode.md)):** the robot
  drives from the keyboard, but the motors **still hum a lot**, and the owner suspects they spend their time near
  stall. After the bench test the owner turned the keyboard's precise mode off and made it ten steps
  (`df91b9e`), so key levels now give 5, 9, 13, 18, 23, 30, 37, 46, 56 and 67 % duty (table in the keyboard guide):
  levels 1 to 4 only hum, 5 moves the right wheel alone, 6 and up move both just past breakaway. This fits the
  suspicion; it is not a measurement. A nearly stalled brushed motor draws a high current and heats: **motor
  and driver temperature were not checked**. Status stays **Open**.
- The 28 % and 21 % may also be the supply: breakaway of 21 to 28 % with no load is high. Note the
  battery voltage under load before settling on numbers (the stiction value is only right at that
  voltage).

## Decision

(Human fills in: date, who, A / B / C, why.)
