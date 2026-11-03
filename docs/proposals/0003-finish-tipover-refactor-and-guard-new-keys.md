# Proposal 0003 — finish the tipover refactor, and teach the guard hook the new safety keys

- **Status:** Open
- **Gate:** safety behavior (the hook refused an edit) and guardrail (`.claude/hooks/`)
- **Raised:** 2026-11-02, built on `85bdc31`, run journal:
  [2026-10-31-sim-plant-and-critical-fixes.md](../journal/2026-10-31-sim-plant-and-critical-fixes.md)

## The change

**A. One edit the hook refused.** In `rocko_env/rocko_env/controllers/add_feedforward/AddFeedforwardController.cpp`
the tipover cutoff exists twice: inline in `update_and_write_commands()` (this copy runs) and as the
private helper `tipover_tripped_this_cycle()` (unused, byte-identical apart from where a comment sits;
the stage 4 round 2 real-time audit diffed them). The author tried to replace the inline block with
one call and the guard hook denied it ("fewer mentions of cutoff, tipover"), because it counts words
and cannot tell a move from a removal. Please make this edit by hand:

- replace the inline block (the `if (tipover_enabled_) { ... }` that reads the pitch, runs the latch
  and logs the trip or re-arm) with
  `const bool tipover_stop = tipover_enabled_ && tipover_tripped_this_cycle();`
- delete the `NOT CALLED YET` notes in the `.cpp` and the `.hpp` that say to edit both copies.

Until then, edit both copies or neither: only the inline one runs, and `tools/tipover_latch_test.cpp`
tests the latch, not the controller.

**B. Add the new safety keys to the hook's lists** (`.claude/hooks/guard_protected_files.py`, the key
list that `docs/proposals/README.md` calls canonical): `tipover_pitch_deg`, `rearm_max_pitch_deg`,
`pitch_interface` and `command_timeout_s`, so that setting `tipover_pitch_deg: 0.0` (which disables
the cutoff with only a log line) or loosening it in a YAML file is a gated edit like a gain.

**C. Set the tipover keys explicitly in the real controller YAML** once B is in, so the value that
runs on the robot is written where the guard can see it, not only as a default in
`add_feedforward_controller_parameters.yaml` (which the hook does not watch):
```yaml
left_add_feedforward_controller:
  ros__parameters:
    tipover_pitch_deg: 35.0
    rearm_max_pitch_deg: 10.0
```
(and the same for the right controller). Values are in [proposal 0004](0004-safety-thresholds.md).

## Why

Finding H-4 (tipover cutoff) is a safeguard that is only as strong as the place its limit is written.
Today a one-line change to a parameter file nobody guards could turn it off.

## Evidence

Tier 3 (model with estimated parameters): `verify_sim.sh H4` shows the cutoff tripping at 35° and
writing exactly 0.0 afterwards, and shows it going red when the limit is 0. Tier 1: builds, no
warnings. Not verified: anything on a robot.

## Risk

A and C change nothing the robot does (the values are the current defaults). B makes more edits
need a human, which is the point.

## Test for a human

1. After A: `docker compose -f docker/compose.yml exec -T ros2 bash -c 'cd /ros2_ws && src/ROCKO-env/build_ros.sh'`,
   then `verify_sim.sh H4` and `verify_mock.sh`: both pass.
2. After B: `.claude/hooks/test-guard-protected-files.sh` passes with new cases for the four keys.

## Decision

(Human fills in.)
