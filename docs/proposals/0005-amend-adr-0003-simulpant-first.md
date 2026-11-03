# Proposal 0005 — amend ADR 0003: build `SimPlant` first, keep Gazebo as a later option

- **Status:** Open
- **Gate:** none of the §3 gates, but ADR 0003 is `Proposed` and a human owns the decision
- **Raised:** 2026-11-02, built on `85bdc31`, run journal:
  [2026-10-31-sim-plant-and-critical-fixes.md](../journal/2026-10-31-sim-plant-and-critical-fixes.md)

## The change

[ADR 0003](../decisions/0003-simulation-target.md) chose Gazebo with `gz_ros2_control`. Finding M-4 and the
`bootstrap-sim` skill recommend a `SimPlant` hardware plugin first. The 2026-10-31 run (decided in advance
by the person who started it) built `SimPlant`. Amend the ADR to say: **Tier 3 is `SimPlant`; Gazebo is a
later option if contact fidelity turns out to matter**, and keep the ADR `Proposed` until a human accepts.

## Why

The ADR and the code now disagree. `SimPlant` is one readable header plus a plugin, needs no image change,
exercises the whole control stack through the hardware interface, and found real issues in its first
sessions (an outer-loop sign problem, a half-speed encoder reading: see
[sim-plant.md §2](../architecture/sim-plant.md)). Its limit is that it models only what is written down, and every
physical number is still an estimate.

## Evidence

Tier 3 partly built: `verify_sim.sh` passes S1, S2a and the H4, C3, C2, H6 and FF checks on a model with estimated
parameters. Nothing about the real robot.

## Risk

A confident-looking simulator with invented inertia is worse than none (ADR 0003's own warning). Mitigation:
the measurement guide (`docs/guides/measure-inertia.md`, not yet written), the `ESTIMATE, NOT MEASURED` labels, and
[the sim-to-real gap page](../architecture/sim-to-real-gap.md).

## Test for a human

Read sim-plant.md's honesty box and §6, decide, and edit ADR 0003's status and decision lines.

## Decision

(Human fills in.)
