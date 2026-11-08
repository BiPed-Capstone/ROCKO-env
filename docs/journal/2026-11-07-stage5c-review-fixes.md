# Stage 5c review fixes, and why scenario V could not judge two `d+` runs

- **Date / commit:** 2026-11-07 (the container's clock said 2026-11-08 for part of it), built on
  `27e9788`. Commits listed under [Commits](#commits).
- **Branch:** `ai-forward`. Local commits only: this run was told not to push.
- **Environment:** `rocko-env-dev` container (`ros:jazzy`, arm64, Docker Desktop on macOS).
  `./check_container_mount.sh` printed `ok    the container sees this checkout`. This run was the only
  user of the container.

**Everything here is about the verifiers and an estimated model.** Nothing says anything about the real
robot. No gain, threshold, controller YAML or guarded file was changed, and every verdict a check gives
on the committed tree is the same as before.

## Goal and success criteria

1. **`check_sim_config.py` (ros2-control review, Medium):** each of the four `SIM_TUNED_GAINS` must be
   present for every joint in its controller's `dof_names`, in both YAML files. Red on a deliberate
   break in each file (one wheel's `p:` deleted), green on the real tree, every older break still red.
2. **Explainability review (six Medium, several Low):** the same behaviour, written so a student can
   follow it. Checked by identical outputs: `cascade_linearisation.py` (default, `--candidate yaml`,
   `--hardware-gains`), the sweep's case list and CSV-driven report, and the scenarios in the container.
3. **The new `COULD NOT JUDGE` of the sweep re-run** (case `d+`, two V runs): find the cause from
   evidence; fix it only if the metric mishandles a legitimate stamp, never making an overshoot look
   smaller; show `d+` V judging on re-runs.
4. The final report's sweep numbers match the re-run.

## 1. `check_sim_config.py`: a tuned gain must be on every joint

Before, the check for a `SIM_TUNED_GAINS` entry asked only that each file had the gain on *some*
joint (a non-empty list). Deleting `p:` from one wheel's entry left the other wheel's value, so the
check passed, while `pid_controller` would run that wheel with `p = 0`. Now, for each listed gain and
each file, every joint in the controller's `dof_names` must have it; otherwise
`<file>: <controller> \`p\` (SIM_TUNED_GAINS) is missing for <joint>`.

Breaks, each on a fresh scratch copy of the files the check reads (`--root`):

| Break | New check | Check at `27e9788` |
| --- | --- | --- |
| sim YAML: `p:` deleted from `pitch_pid_controller`'s right wheel only | exit 1, `rocko_sim_controllers.yaml: pitch_pid_controller \`p\` (SIM_TUNED_GAINS) is missing for right_add_feedforward_controller/right_wheel_joint ...` | exit 0 (missed) |
| hardware YAML: the same | exit 1, the same line naming `rocko_controllers.yaml` | exit 0 (missed) |
| sim YAML: velocity `i:` deleted from the right wheel only | exit 1, `velocity_pid_controller \`i\` ... is missing for pitch_pid_controller/right_wheel_joint` | not run |
| the 12 breaks of the [stage 5c-1 journal](2026-11-03-stage5c1-sim-gain-tuning.md#1-the-drift-check-only-the-tuned-gains-may-differ), adapted to today's sim gains (the control now moves pitch `d` 1.0 to 0.8) | 11 red with the same lines as then, control green | — |

Real tree: six `ok` lines, exit 0.
