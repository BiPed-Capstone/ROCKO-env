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

## 2. The sim scripts, made easier to read (no behaviour change)

What changed, finding by finding:

| Finding | Change |
| --- | --- |
| `cascade_linearisation.simulate()` had bare thresholds (`0.09`, `30`, `35.0`, ...) with names only in comments | It reads them from `verify_sim.sh` by name (`scenario_thresholds()`, through `verify_sim_values()`, which follows `TIPOVER_SCENARIO_DEG=$TIPOVER_DEG`). `parameter_sweep.thresholds()` now calls the same function, so there is **one** copy of each number, in `verify_sim.sh`. No circular import: `parameter_sweep` already imported `cascade_linearisation`, not the other way. The feedforward's wheel radius is named (`DIFFDRIVE_WHEEL_RADIUS_M`) |
| `(1 - a) ** (2 * cycles - 1)` in `velocity_from_true_periods()` unexplained | Comment: each cycle applies the filter twice (left call, right call); the last factor is the next line's right call |
| Positional sample tuples (`s[3]`, `b[0]`) in `scenario_checks.py` | `Sample` namedtuple (`t_s pitch_deg pitch_rate_deg_s mean_velocity_m_s position_m right_velocity_m_s`, the order the code built them in), every use site in U, I, V and the metric; `write_samples` writes the same header and columns (`SAMPLES_CSV_HEADER`). Nothing outside the file reads the tuples (`verify_sim.sh` reads `metrics.json` only). The tipover and IMU-hold checks keep their own, differently shaped tuples (commented where they are built) |
| Corner enumeration by bit masks | `itertools.product((+1, -1), repeat=4)`, the same order (the 25-case list is identical, checked) |
| `report()` one long function; legend typed by hand; `CODE_SCENARIO_ARGS` said "read from" `verify_sim.sh` | `header_lines()` and `table_row()`; the legend is built from `NOMINAL`, `LETTER` and `UNIT`; the comment now says the launch arguments are copied by hand and checked by nothing |
| `derive_plant`/`set_plant` reset with retyped literals | `reset_plant()` restores `DEFAULT_PLANT`, captured from the module's own estimates |
| Low | C3: `command_sent_s`, `command_left_s`, `mid_drive_silences`; the "two cycles" reason written once, as a numbered chain, in `handled_by`; `record_from_release`'s docstring re-wrapped; `discrete_cascade` says `delay_cycles` is ignored when `delay_ms` is given; `number_` became `case_index`; the two files `run_verify` left open (and the YAML reader's) are closed with `with`; `MARGINAL_OVERSHOOT_PCT`'s comment |

**`MARGINAL_OVERSHOOT_PCT = 27`:** the brief suggested "the 30 % limit minus a 3-point noise
allowance". The [stage 5c-2 journal](2026-11-03-stage5c2-sweep.md) only states the rule ("above 27 % in
some run"); no derivation of the 3 points was written. The comment now says so, and gives the scale it can
be compared with: V's old metric spread over 23.5-29.1 % in 6 runs of one model (stage 5c-1), so 3 points is
about half that spread.

**Checks that the behaviour did not change** (laptop unless stated; "before" = a scratch worktree at `27e9788`):

| Check | Result |
| --- | --- |
| `python3 -I sim/cascade_linearisation.py` (default report), `--candidate yaml`, `--hardware-gains` | each output identical to before (`diff`, no lines) |
| `python3 -I sim/cascade_linearisation.py --sweep --top 15` (1 800 grid points, 15 time-simulated) | identical to before |
| `parameter_sweep.all_cases("all")` | the same 25 cases in the same order |
| `parameter_sweep.py report` on a copy of `stage5c2-sweep-vmetric2.csv` | the same verdicts and every table cell identical; two lines differ: the CSV's relative path (the copies were in different folders) and the legend's `0.20 m`, now `0.2 m` (built from `NOMINAL`) |
| container: `velocity_from_true_periods()` at `27e9788` and now, on the 13 kept V `samples.csv` files in the container | identical output for all 13 (including the two refused `d+` runs) |
| `python3 -I -m py_compile sim/*.py` | ok |
