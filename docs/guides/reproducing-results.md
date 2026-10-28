# Recording a result so it reproduces

**Who this is for:** anyone who measures something on ROCKO — a gain set, a calibration, a latency
number, "it balanced for 30 seconds."

**Why it exists:** a number without its conditions is a rumor. ROCKO is a teaching platform, and the
point is that the next person can *get your result again* and build on it. A gain set that worked on
a full battery on carpet, recorded with neither fact, is not a result — it is an anecdote that will
waste somebody's afternoon.

---

## The minimum record

Copy this block into your commit message, PR, or the doc where you report the result. Every field, or
an explicit "unknown" — a blank is indistinguishable from a forgotten field.

```
Result:        <what you were measuring, and what happened>
Date:          YYYY-MM-DD
Commit:        <git rev-parse --short HEAD>   (and note if the tree was dirty)
Launch:        <ros2 launch rocko_env rocko.launch.py | rocko_tank_mode.launch.py>, with every argument
Hardware:      <real | mock (use_mock_hardware:=true)>   (a mock result is a wiring check, not a measurement)
Config:        <which *_controllers.yaml, and any gain differing from the committed value>
Battery:       <volts at start, volts at end>
Surface:       <carpet / tile / plywood / ...>
Robot:         <which physical unit, if there is more than one>
Payload:       <anything added or removed from the chassis>
Logs:          <where the rosbag / webserver capture / video lives>
Observed:      <what actually happened, in plain words>
```

### Why each field is there

- **Commit SHA.** Gains live in YAML that `--symlink-install` links rather than copies, so an edited
  file is live without a rebuild — the SHA is the only thing that says which code produced the number.
  Note a dirty tree, or the SHA is misleading.
- **Battery voltage.** The motor command path is currently **open loop**: commanded velocity converts
  straight to a PWM duty cycle with no feedback (finding H-1 in
  [`../claude-analysis/balance-loop-findings.md`](../claude-analysis/balance-loop-findings.md)). So the
  plant gain the PID sees *drops as the battery drains*. The same gains behave differently at 12.4 V
  and 11.1 V. Until H-1 is fixed, voltage is not optional context — it is part of the plant.
- **Surface.** Wheel friction and compliance change the plant. Carpet and tile are different robots.
- **Launch file, arguments and config.** Balancing and tank mode are different control structures
  entirely. `use_mock_hardware:=true` runs the controllers against a mock with no physics, so
  anything it "measures" is the mock's, not the robot's.
- **Logs.** So a disagreement can be settled by looking, not by re-running from memory.

## Capturing the run

```bash
# On the Pi, in a terminal with the workspace sourced.
# Record everything, so a question you did not think to ask is still answerable later.
ros2 bag record -a -o ~/rocko_logs/$(date +%Y%m%d_%H%M%S)_describe-the-test
```

Narrower, if `-a` is too much for the SD card:

```bash
ros2 bag record -o ~/rocko_logs/$(date +%Y%m%d_%H%M%S)_pitch_step \
  /left_feedforward /right_feedforward /joint_states /dynamic_joint_states
```

Check what exists before recording — topic names drift:

```bash
ros2 topic list
```

Video of the robot is worth as much as the bag for balance work. Oscillation frequency is obvious in
a video and hard to see in a plot.

## Changing a gain, reproducibly

1. **Change one number in one loop.** A gain set found by moving three at once does not transfer and
   teaches nothing about which term did the work.
2. **Note the old value** before you change it. The committed YAML already carries commented-out
   historical values — keep that habit, it is a record of the search.
3. **Test the same way every time.** Same surface, same starting battery range, same disturbance. A
   repeatable test matters more than a clever one: hold the robot upright, release, push it gently by
   hand from a fixed spot.
4. **Record the block above** whether it worked or not. A negative result is a result, and stops the
   next person repeating it.
5. **Tune inner loop before outer.** Pitch first, then velocity, then feedforward — retuning pitch
   invalidates every velocity gain. See
   [`../architecture/control-cascade.md`](../architecture/control-cascade.md).

## Before an on-robot test

- Tether or hold the robot, and know where the kill path is.
- Note that **there is currently no tipover cutoff and no comms-loss failsafe** (findings H-4 and C-3).
  The robot will keep driving with the gamepad unplugged. Plan the test around that.
- Confirm the battery is above the range you intend to report.

## Reporting honestly

- Say what you measured and what you inferred. "It balanced 30 s" is a measurement; "the D term is
  better" is an inference.
- Report failed runs. In a cascade, a failure often localizes the problem better than a success.
- If you cannot fill in a field, write `unknown` rather than leaving it out. Somebody will otherwise
  assume the value was nominal.

An AI agent working in this repo **cannot** produce any of these numbers — it has no robot. If an
agent hands you a gain change, it owes you this block as a *test procedure to run*, with the results
blank. See [`CLAUDE.md`](../../CLAUDE.md) §9.
