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

| # | Proposal | Gate | Status |
| --- | --- | --- | --- |
| [0001](0001-allow-container-mount-check.md) | Allow `./check_container_mount.sh` without a prompt | guardrail | Accepted (applied 2026-10-30 with 0002, item B) |
| [0002](0002-unattended-run-settings.md) | Settings and enforcement for a run nobody is watching (items A to F; B includes 0001) | guardrail | Partly accepted: A, B, C, D applied 2026-10-30; E, F open |
