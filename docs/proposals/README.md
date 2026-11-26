# Proposals — changes that need a human decision

On the `ai-forward` branch the AI plans, implements, verifies and reviews without stopping to ask.
A few changes are different: the AI cannot verify them, and getting them wrong damages a robot or
silently changes what the project supports. For those, the AI **writes a proposal here and carries
on with the rest of its work** instead of making the edit.

## What needs a proposal

| Gate | Why the AI cannot decide it | Enforced by |
| --- | --- | --- |
| **Hardware gains and limits** — any line in `bringup/config/*.yaml` (not `*_sim.yaml`) that has one of these keys: `p`, `i`, `d`, `i_clamp_max/min`, `u_clamp_max/min`, `feedforward_gain`, `tracking_time_constant`, `antiwindup*`, `update_rate`, `cmd_vel_timeout`, `has_velocity/acceleration/jerk_limits`, `max_/min_velocity/acceleration/jerk` | They are experimental results from one robot, one surface, one battery voltage. Sim-tuned values do not transfer. This table is the canonical key list | Hook denies the edit |
| **Version pins** — ROS 2 Jazzy, `ros2_control` ≥ 4.0.0, Ubuntu 24.04, C++17, CMake ≥ 3.16 | A version bump is a project decision with a migration plan ([ADR 0001](../decisions/0001-version-lock.md)). | Hook denies the edit |
| **Weakening a safety behavior** — tipover cutoff, watchdog, command clamp, error return | A simulator cannot see a real fall or a lost gamepad link. | Hook denies an edit to `.cpp`/`.hpp`/`.cc`/`.h`/`.py` under `rocko_env/rocko_env/` that leaves fewer mentions of tipover, watchdog, clamp, fault and similar words than before. It counts words, not behavior; reviewers catch the rest |
| **The guardrails** — `.claude/hooks/`, `.claude/settings.json`, `.claude/settings.local.json`, `.claude/commands/`, `.mcp.json`, `.git/config`, `.git/hooks/` (the last two run code on an ordinary `git status`) | The AI must not loosen the rules it works under. | Hook asks (a headless run is refused). `CLAUDE.md`, agents and skills are **not** guarded; a human reads their diffs |
| **Putting anything on a robot** — a launch on a Pi, a flash, a first tethered test | Only a human can be there to catch it. | Permission prompt on `ros2 launch` / `ros2 run` |

The hook is a tripwire, not a proof: it cannot see every way to weaken a safeguard. The
`realtime-auditor` reviewer and [CLAUDE.md](../../CLAUDE.md) carry the rest.

## If you are the human

1. Read the open proposals in the index below.
2. Decide. Either make the change yourself, or set the proposal's `Status` to `Accepted` with a note
   and the AI will make it, or set it to `Declined` with the reason (the reason stops it being
   proposed again).
3. If it needs a robot test, the proposal contains the procedure. Record the outcome in it, in the
   format from [reproducing results](../guides/reproducing-results.md).

## If you are the AI

- Write the proposal **in the same turn you were denied.** Do not wait, and do not retry the edit.
- One proposal per decision. Number it by taking the next free `NNNN`.
- Be specific enough that a human can say yes or no in a minute: exact file and line, old → new.
- Say what you verified and at which tier of the
  [verification ladder](../architecture/verification-ladder.md), and say plainly what you could not.
  A proposal that implies it was tested on hardware is a lie.
- The test procedure must say: tether or hold the robot, what to change, what success looks like,
  and when to abort.

## Template

Copy to `NNNN-short-slug.md`.

```markdown
# Proposal NNNN — <the change, as a statement>

- **Status:** Open | Accepted | Declined | Done
- **Gate:** hardware gains | version pin | safety behavior | guardrail | on-robot step
- **Raised:** YYYY-MM-DD, commit <sha>, run journal: <link to docs/journal/ entry>

## The change

File and line, old → new. Exact.

## Why

The goal this serves and what goes wrong without it. Link findings by ID (C-1, H-3, ...).

## Evidence

Which verification tier you reached and the command and result for each. Say what you did NOT
verify. Never write that a gain is better or that balance improved: you cannot observe that.

## Risk

What could go wrong on the robot, and what limits the damage.

## Test for a human

Numbered steps. Tethered. Success looks like ... Abort if ...
Record: commit SHA, launch file and config, battery voltage, surface, where the log went.

## Decision

(Human fills in: date, who, accepted/declined, why.)
```

## Index

Proposals keep their numbers and files when decided, so links to them never break; only this index
sorts them.

### Waiting on you

In the order that most helps the robot: the first rows unblock the next balance test. "Needs" says whether
deciding takes the robot or only a desk.

| # | What you decide | Needs | Gate |
| --- | --- | --- | --- |
| [0017](0017-clamp-the-target-lean.md) | Clamp the velocity loop's target lean to ±5 deg (from ±150): the likely cause, reasoned not recorded, of the one-way wheel spin on the 2026-11-25 floor test | desk to decide; robot to test | hardware limit |
| [0016](0016-pitch-d-term-from-the-gyro.md) | Nothing to decide (accepted and applied 2026-11-24): run its robot test | robot | wiring of the pitch controller |
| [0008](0008-candidate-gains-and-tethered-test.md) | Sim-tuned candidate gains for `rocko_controllers.yaml`, with preconditions and a tethered test (nothing tested on a robot) | robot | hardware gains |
| [0004](0004-safety-thresholds.md) | Confirm the proposed safety thresholds (watchdog, tipover, IMU faults, motor) | desk; some want a robot check | hardware limits |
| [0013](0013-tank-mode-motor-breakaway.md) | The motors' dead band: add a per-motor stiction duty (A), or change nothing (C); matters for balancing, where commands stay near zero | robot (bench measurement) | hardware limit |
| [0012](0012-command-ramp-and-watchdog-step.md) | Confirm the drive command's ramp limit (H-13), and whether the watchdog's zero should ramp too | desk | hardware limit / safety behavior |
| [0007](0007-measure-robot-parameters.md) | Measure the robot's mass, inertia and geometry ([guide](../guides/measure-inertia.md)) so the simulator stops using estimates | bench | on-robot step |
| [0014](0014-tank-mode-full-speed-for-shaped-sticks.md) | Confirm tank mode's full stick = 100 % motor duty (applied at your request 2026-11-20) | desk | hardware limit (in spirit) |
| [0003](0003-finish-tipover-refactor-and-guard-new-keys.md) | A five-minute hand edit the hook refused (delete a duplicate copy of the tipover check), and add four safety keys to the hook's list | desk | safety behavior / guardrail |
| [0009](0009-controller-manager-minimum-version.md) | Which minimum `controller_manager` version the build should require | desk (read the Pi's installed version) | version pin |
| [0005](0005-amend-adr-0003-simulpant-first.md) | Amend ADR 0003 to say what was built: `SimPlant` first, Gazebo later if needed | desk | decision |
| [0002](0002-unattended-run-settings.md) | Items E (git hooks) and F (GitHub branch protection); A to D were applied 2026-10-30 | desk | guardrail |
| [0011](0011-agent-loop-cost-settings.md) | Settings that make agent runs cheaper: allow-list entries for the helper scripts, fewer plugins and MCP servers | desk | guardrail |

### Decided or superseded

| # | Proposal | Outcome |
| --- | --- | --- |
| [0001](0001-allow-container-mount-check.md) | Allow `./check_container_mount.sh` without a prompt | Accepted; applied 2026-10-30 with 0002, item B |
| [0006](0006-velocity-loop-sign-check.md) | Check the velocity-loop sign on a stand (H-12) | Accepted (option A, 2026-11-24); see its record |
| [0010](0010-c1-imu-uart-and-encoders-in-cpp.md) | Finding C-1: read the BNO055 over UART and the encoders in C++ | Mostly superseded 2026-11-24: C-1 was done with the IMU over I²C; the UART sections are the fallback if I²C proves unreliable |
| [0015](0015-tank-mode-faster-reversal.md) | Tank mode reverses slowly: raise `diff_drive_controller`'s deceleration limit | Accepted 2026-11-22; applied, then raised to 160 m/s² by the owner |
