# The balance control cascade

**What this explains:** why ROCKO uses two nested PID loops instead of one, and why the outer loop
commands a *lean angle* rather than a wheel speed.

**Code this describes:** `rocko_env/rocko_env/bringup/config/rocko_controllers.yaml`,
`rocko_env/rocko_env/controllers/add_feedforward/AddFeedforwardController.cpp`

---

## The problem

ROCKO is an **inverted pendulum on wheels**: its center of mass is above the wheel axle, so left
alone it falls. The only actuator is the pair of wheels. That gives the control problem an awkward
property — the thing you want to control (where the robot goes) and the thing you must control to
avoid falling (which way it leans) fight each other.

To accelerate forward, the robot must first lean forward. To lean forward, it must briefly drive the
wheels *backward* to let the body tip. So a naive "I want to go forward, therefore spin the wheels
forward" controller does the opposite of what balance requires. This is what makes a balancing robot
a genuinely interesting controls problem rather than a plumbing exercise.

## The structure

The fix is a **cascade**: two PID loops, one inside the other, where the outer loop's output is the
inner loop's setpoint.

```
commanded velocity (from the gamepad, via DiffDriveController.py)
        │
        v
┌──────────────────────────────────────────────────────────────┐
│ velocity_pid_controller                 OUTER, slow          │
│   measures: wheel velocity (from the encoders)               │
│   asks:     "am I going the speed I was told?"               │
│   outputs:  a TARGET PITCH ANGLE                             │
└──────────────────────────────┬───────────────────────────────┘
                               │  setpoint
                               v
┌──────────────────────────────────────────────────────────────┐
│ pitch_pid_controller                    INNER, fast          │
│   measures: pitch (from the IMU)                             │
│   asks:     "am I leaning the amount I was told?"            │
│   outputs:  a WHEEL VELOCITY                                 │
└──────────────────────────────┬───────────────────────────────┘
                               │
                               v
┌──────────────────────────────────────────────────────────────┐
│ left/right_add_feedforward_controller                        │
│   adds a feedforward term, then writes the wheel command     │
└──────────────────────────────┬───────────────────────────────┘
                               v
              Motor12VoltQuadEncoder  (hardware plugin -> PWM)
```

Read the `command_interface` lines in `rocko_controllers.yaml` and you can see the inversion
directly: `velocity_pid_controller` has `command_interface: pitch`, while `pitch_pid_controller` has
`command_interface: velocity`. Each loop commands the *other* quantity.

## Why nested, and why this order

**The inner loop must be faster than the outer loop.** Pitch is the unstable direction — if the lean
angle runs away, the robot is on the floor in a fraction of a second, and no amount of velocity
tracking matters. Velocity error, by contrast, is survivable: being 10% slow is an inconvenience.

So the inner loop's job is stability, the outer loop's job is performance, and the outer loop is
allowed to be slow and gentle. The inner loop treats whatever lean angle it is handed as gospel.

**Consequence for tuning:** the outer loop's plant *is* the closed inner loop. Retune pitch and every
velocity gain you found is invalid. Always tune inner first, then outer, then feedforward. This is
not a style preference; it falls out of the structure. See the
[`tune-gains` skill](../../.claude/skills/tune-gains/SKILL.md).

## Where feedforward fits

The two PID loops are *reactive* — they only act once an error exists. The feedforward path adds a
term computed from the command rather than the error, so a known, predictable demand doesn't have to
be discovered as a mistake first. `AddFeedforwardController` is a **chainable** controller: it sits
between the pitch loop and the hardware, adding its term to what it is passed.

Feedforward is a refinement on a working cascade, not a substitute for one. Tune it last.

## How ros2_control expresses this

`ros2_control` supports **chained controllers**: a controller can export *reference interfaces* that
another controller writes into, instead of publishing on a topic. That is what the
`dof_names` entries like `pitch_pid_controller/left_wheel_joint` mean — "my output goes into that
controller's reference, not onto a topic."

The whole chain runs **in one thread at one rate**, set by `update_rate: 100` in
`rocko_controllers.yaml`. There is no DDS hop between the loops and no scheduling uncertainty
between them — which is exactly the property you want in a nested loop, and exactly what gets lost
when part of the chain is moved out into a separate Python node.

## Current state — read this before trusting the diagram

The structure above is right. The **implementation underneath it is not yet sound**, and the balance
loop is not reliable today. In short: the sensor reads in the loop are blocking service calls into
Python nodes with no timeout, a failed IMU read reports "upright", and the motor command is open
loop. Full detail, with severities and locations, in
[`../claude-analysis/balance-loop-findings.md`](../claude-analysis/balance-loop-findings.md).

Tuning gains against that feedback path is largely wasted effort, because the plant changes character
once it is fixed.

## Further reading

- [ros2_control concepts](https://control.ros.org/jazzy/doc/getting_started/getting_started.html) —
  the official explanation of hardware interfaces, controllers, and chaining.
- [`ADR 0002`](../decisions/0002-cascaded-balance-control.md) — this choice as a decision record,
  with the alternatives that were not taken.
