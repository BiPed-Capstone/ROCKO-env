---
name: tune-gains
description: Guided, reproducible PID tuning for ROCKO's cascaded balance controller. Use when asked to tune, adjust, or reason about PID gains (p/i/d, i_clamp) in rocko_controllers.yaml, when the robot oscillates, falls, drifts, or feels sluggish, or when asked which loop to tune first.
---

# Tuning ROCKO's gains

## Gains are experimental data, not code

A gain in `rocko_env/rocko_env/bringup/config/rocko_controllers.yaml` is the outcome of a physical
experiment on a specific robot, on a specific surface, at a specific battery voltage. **Never edit
one on an agent's judgment.** You have no robot and cannot observe the result.

What you *can* do, and should: explain the loop, predict the direction of a change, propose one
experiment at a time, and write down the result the human reports.

## Check the plant before touching a gain

If the feedback path is broken, tuning is fitting gains to noise and the work is thrown away when
the path is fixed. Before proposing gains, confirm with the human:

- Is the loop actually running at 100 Hz, without overruns?
- Does a failed sensor read still report "upright"? (finding C-2)
- Is the motor command still open loop — PWM straight from commanded velocity? (finding H-1)
- Is the D term differentiating a fused angle instead of reading gyro rate? (finding H-6)

If any answer is yes, say clearly that tuning is premature and point at
[`balance-loop-findings.md`](../../../docs/claude-analysis/balance-loop-findings.md).

## The cascade, and which loop to tune first

```
velocity_pid_controller   (outer)  measured wheel velocity  -> commands TARGET PITCH
pitch_pid_controller      (inner)  measured pitch           -> commands WHEEL VELOCITY
left/right_add_feedforward_controller       adds feedforward -> motor PWM
```

**Always tune inner before outer.** The outer loop's plant *is* the closed inner loop; retuning
pitch invalidates every velocity gain you found. Concretely:

1. **Pitch loop alone.** Hold the robot, disable or zero the velocity loop, and get the robot to
   hold an upright reference without oscillating.
2. **Velocity loop.** Only once pitch is stable and repeatable.
3. **Feedforward.** Last — it is a correction on top of a working cascade, not a substitute for one.

Current values, for reference (as of this writing):

| Loop | p | i | d | i_clamp |
| --- | --- | --- | --- | --- |
| `pitch_pid_controller` | 10.0 | 0.0 | 0.1 | ±150 |
| `velocity_pid_controller` | 2.0 | 5.0 | 0.2 | ±150 |

Note `i = 5.0` on the outer loop with `i = 0.0` on the inner — worth a conversation. Integral on the
outer loop of a cascade winds up whenever the inner loop cannot deliver the commanded lean, and
±150 is a large clamp.

## Symptom → first suspicion

Offer these as hypotheses to test, never as edits to make.

| Symptom | Usually |
| --- | --- |
| Fast buzzing / chatter around upright | Pitch `p` or `d` too high; or D amplifying sensor noise (see H-6) |
| Slow rocking, growing | Pitch `p` too low, or loop delay eating phase margin |
| Holds still, falls when driven | Velocity loop commanding a lean the pitch loop can't reach |
| Drifts steadily off level | Missing integral term, or an uncalibrated IMU zero |
| Lurches after being held still | Integral wind-up against the clamp |
| Works on carpet, not tile, or worse as battery drains | Plant gain varies — that's H-1, open-loop motor command, not a gain problem |

## One change at a time, and write it down

Change **one gain in one loop**, then observe. Gains found by moving three numbers at once do not
transfer and teach nothing.

Record each attempt in the log format from
[`docs/guides/reproducing-results.md`](../../../docs/guides/reproducing-results.md): commit SHA,
launch file, the gain changed and its old value, battery voltage, surface, what happened.

When the human lands on a gain set worth keeping, write it into the YAML **with a comment giving the
date and conditions**, and record the run in `docs/guides/`. The file already carries commented-out
historical values — keep that habit, it is a real record of the search.

## Safety before any on-robot test

Tether or hold the robot. Have the kill path ready. Confirm with the human that a tipover cutoff and
comms-loss failsafe exist — today they do not (findings C-3, H-4), which means the robot will happily
keep driving into a wall with the gamepad unplugged. Say that out loud before suggesting a test.
