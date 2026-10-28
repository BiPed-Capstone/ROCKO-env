---
name: tune-gains
description: Guided, reproducible PID tuning for ROCKO's cascaded balance controller. Use when asked to tune, adjust, or reason about PID gains (p/i/d, i_clamp) in rocko_controllers.yaml, when the robot oscillates, falls, drifts, or feels sluggish, or when asked which loop to tune first.
---

# Tuning ROCKO's gains

## Two kinds of gain, and who owns each

| Kind | Where | Who changes it |
| --- | --- | --- |
| **Hardware gains** | `rocko_env/rocko_env/bringup/config/rocko_controllers.yaml` | A human. The hook **denies** your edit; you write a proposal. |
| **Sim gains** | a separate `*_sim.yaml` next to it, once a physics sim exists | You, freely. The hook does not guard files with `sim` in the name. |

A hardware gain is the outcome of a physical experiment on one robot, one surface, one battery
voltage. You have no robot and cannot observe the result, so you never edit one. A sim gain is a
number tuned against a model; it is a *candidate* for the hardware file, never a replacement.

## What you can do without a physics simulation

Check the [verification ladder](../../../docs/architecture/verification-ladder.md) for whether Tier 3
(a physics simulation, [ADR 0003](../../../docs/decisions/0003-simulation-target.md)) exists. The mock
hardware, which always exists, echoes commands and holds pitch at 0.0, so a balance controller always
looks fine against it: without Tier 3 nothing exists for you to tune *against*. Say so plainly rather
than inventing a number. Until Tier 3 exists, gain work is:

- explain the loop and predict the direction of a change,
- find which loop a symptom implicates (table below),
- design **one** experiment and write the human's test procedure,
- record the result the human reports, in the format of
  [`reproducing-results.md`](../../../docs/guides/reproducing-results.md),
- build the missing sim (the `bootstrap-sim` skill) — that is the real unlock.

## Check the plant before touching a gain

If the feedback path is broken, tuning is fitting gains to noise and the work is thrown away when
the path is fixed. Before proposing gains, check the code and the findings doc for:

- Is the loop actually running at 100 Hz, without overruns?
- Does a failed sensor read still report "upright"? (finding C-2)
- Is the motor command still open loop — PWM straight from commanded velocity? (finding H-1)
- Is the D term differentiating a fused angle instead of reading gyro rate? (finding H-6)

If any answer is yes, say clearly that tuning is premature and point at
[`balance-loop-findings.md`](../../../docs/claude-analysis/balance-loop-findings.md). Fixing the
plant is code you may write.

## The cascade, and which loop to tune first

```
velocity_pid_controller   (outer)  measured wheel velocity  -> commands TARGET PITCH
pitch_pid_controller      (inner)  measured pitch           -> commands WHEEL VELOCITY
left/right_add_feedforward_controller       adds feedforward -> motor PWM
```

**Always tune inner before outer.** The outer loop's plant *is* the closed inner loop; retuning
pitch invalidates every velocity gain you found. Concretely:

1. **Pitch loop alone.** Zero or disable the velocity loop and get the robot to hold an upright
   reference without oscillating.
2. **Velocity loop.** Only once pitch is stable and repeatable.
3. **Feedforward.** Last — it is a correction on top of a working cascade, not a substitute for one.

Current hardware values, for reference (as of this writing — check the YAML, it is the truth):

| Loop | p | i | d | i_clamp |
| --- | --- | --- | --- | --- |
| `pitch_pid_controller` | 10.0 | 0.0 | 0.1 | ±150 |
| `velocity_pid_controller` | 2.0 | 5.0 | 0.2 | ±150 |

Note `i = 5.0` on the outer loop with `i = 0.0` on the inner — worth raising in a proposal. Integral
on the outer loop of a cascade winds up whenever the inner loop cannot deliver the commanded lean,
and ±150 is a large clamp.

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

## Tuning in simulation (once Tier 3 exists)

Written down now so it is followed when the sim lands.

1. **Define the score before you tune.** Write the metric and the pass threshold in the run journal
   first (settling time, peak pitch, steady-state velocity error, saturation count). A metric chosen
   after seeing the result is a story, not a measurement.
2. **One gain, one loop, one change at a time**, inner loop first. Gains found by moving three
   numbers at once do not transfer and teach nothing.
3. **Scenarios in two sets.** *Tuning scenarios*: hold upright 10 s; recover from a pitch impulse;
   follow a velocity step. *Held-out scenarios* you never tune on: different initial pitch, ±20 %
   on mass and wheel friction, an added sensor delay of one control period. A gain set that passes
   tuning scenarios and fails held-out ones is overfit to the sim's quirks. That is the main way a
   sim-tuned gain misleads, and it is the first thing a reviewer checks.
4. **Report the margin, not the pass.** "Passes with 4× headroom before saturating" is evidence;
   "passes" is not.
5. **State the sim-to-real gap** next to every number: what the sim does not model (backlash,
   software-PWM aliasing, battery sag, IMU latency). See ADR 0003.

## Promoting a gain to the robot

Never by editing the hardware YAML. Write a proposal
([`docs/proposals/`](../../../docs/proposals/README.md)) containing: the gain set, old → new, the
sim evidence including the held-out results, the gap statement, and the **hardware test**:

1. Tether or hold the robot; kill path ready.
2. Change one gain, in one loop.
3. What success looks like, and when to abort.
4. Record: commit SHA, launch file and config, battery voltage, surface, log location.

A human runs it. When they land on a set worth keeping, it goes into the YAML **with a comment giving
the date and conditions** — the file already carries commented-out historical values, and that
habit is a real record of the search.

## Safety before any on-robot test

Confirm a tipover cutoff and a comms-loss failsafe exist. Today they do not (findings C-3, H-4),
which means the robot will happily keep driving into a wall with the gamepad unplugged. Put that in
the proposal's Risk section in so many words. Writing those safeguards is code you may do; their
thresholds are a hardware decision (named parameter, value in a proposal).
