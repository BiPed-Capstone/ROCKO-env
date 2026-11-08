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

## 3. Why V could not judge two `d+` runs: the container's system clock stepped back

The sweep re-run's two refused runs (`d+`, V runs 1 and 3) kept their run directories in the container
(`/tmp/tmp.WpK7nV9MIN`, `/tmp/tmp.Aayx8EK9jP`), with `samples.csv` and the launch log. Evidence:

- **One pair in each run, not a repeat.** Each run has exactly one pair whose stamp goes *backwards*:
  5.4621 then 5.4551 s after release (-7.0 ms) in run 1, 5.4809 then 5.4742 s (-6.7 ms) in run 3. No
  equal stamps, no other gap under 0 and none over 15 ms. The wheel travel (`position_m`) keeps rising
  across the pair by the usual amount (1.8 mm, about 8 ms of travel at 0.227 m/s), so the message is not
  a duplicate or out of order: the physics advanced normally and only the stamp went back.
- **The stamp clock lost 16-18 ms against the physics.** Over the 60 cycles around each event the
  stamps advance 584.8 and 582.0 ms while the travel divided by the speed gives 602.6 and 597.8 ms:
  the stamps lost 17.8 and 15.8 ms, all in that one pair.
- **Another process saw the same thing at the same moment.** `robot_state_publisher` (which reads the
  same `controller_manager` stamps, on `/joint_states`) logged `Moved backwards in time, re-publishing
  joint transforms!` at 5.455 s after release in run 1 and 5.474 s in run 3: exactly the two pairs. So
  the stamps were already backwards when published; it is not this script's recording.
- **Two clocks.** The launch log says `Using Steady (Monotonic) clock for triggering controller manager
  cycles` (`controller_manager` 4.48.1): the cycle period it hands to `read()`, which SimPlant integrates,
  is on the steady clock, while the stamps on its messages are the ROS clock, which is the system clock
  (CLOCK_REALTIME). A step of the system clock moves the stamps and nothing else.
- **The system clock does step in this container.** The two warnings are 29.99 s apart in wall time
  (1794097708.939 and 1794097738.928). A watcher that compared `time.time()` with `time.monotonic()`
  every 0.5 ms for 15 minutes in the idle container (scratch, not committed) saw steps at 89.754 s
  (-0.72 ms) and 119.757 s (+0.93 ms), 30.00 s apart, and later -0.71/+0.71 ms and -0.65 ms ones: a
  periodic correction of the VM's clock, presumably Docker Desktop's time sync (its mechanism was not
  looked up). Idle it is under a millisecond; during the sweep, with the container busy, it was about
  -16 ms.

**So it is not a bug in the metric's handling of a legitimate stamp, and the refusal stands.** The
metric takes the gap between two stamps as the cycle's real length; across a backward step that gap is
-7 ms while the cycle really lasted about 8 ms, and nothing in the recorded samples gives the real
length without a guess (the travel divided by a neighbouring speed would, but only while the robot moves,
and it is an estimate). Assuming the nominal 10 ms instead would hide part of the jump if the cycle was
short, so it is not conservative. `COULD NOT JUDGE` is the honest answer for such a run; the case verdict
already says so, and a re-run judges (below).

**What did change** (`sim/scenario_checks.py`, verdicts unchanged): the refusal now says where and how
large the backward step was and points at the explanation, e.g. `... so a period is unknown (1 pair(s),
the first at 5.474 s after release, -6.7 ms; seen when the system clock that stamps them stepped back,
see velocity_from_true_periods)`; and the function's docstring no longer says the stamps are "the same
period SimPlant integrated". It says what is true: they agree unless the system clock steps, a backward
step makes V refuse, and **a forward step would look like a late cycle and be corrected as one**,
removing a jump that is not there. That last case would make an overshoot look smaller, and nothing
detects it. Only sub-millisecond forward steps were seen (+0.93 and +0.71 ms; on a 10 ms cycle at
0.23 m/s that is under 0.001 m/s, under 1 % of the 0.1 m/s step, decaying over 0.2 s), but a large one
is not ruled out. A real fix needs the steady-clock period, which SimPlant does not export.
On the 13 kept V runs the new code gives the same values and verdicts as before (the two refused runs
refuse, with the new text).

### `d+` V re-run: it judges

`SIM_PLANT_OVERRIDES="body_mass_kg:=2.0 com_height_m:=0.2 motor_stall_torque_Nm:=0.774725 imu_delay_ms:=26"
verify_sim.sh scenarios V`, three batches of three, on `74e8fe5`, with the clock watcher running alongside:

| Batch | Runs | Judged overshoot (highest exported sample) |
| --- | --- | --- |
| 1 | 3 PASS | 15.2 (17.0), 15.9 (17.4), 15.6 (18.1) % |
| 2 | 3 PASS | 15.4 (18.4), 25.2 (24.5; the gamepad watchdog fired mid-drive), 18.7 (20.4) % |
| 3 | 2 PASS, **1 FAIL** | **31.5 (62.8)**, 17.4 (19.7), 19.7 (22.7) % |

**All 9 judged; none refused.** The only clock step during the 9 runs was +0.60 ms. Rise 0.88-0.97 s,
peak |pitch| 4.37-4.45 deg in every run.

**The FAIL is real motion caused by the harness, not a metric error.** `controller_manager` logged
`Overrun detected! ... The loop took 40.164503 ms (missed cycles : 5)` at 4.26 s after release; the samples
show one 45.9 ms cycle there. The replica read that cycle's 10.9 mm of travel as a wheel at 0.1575 m/s
(exported); the metric removed it (judged 0.1153 m/s at that sample, the speed before). But the velocity
loop had already acted on the false reading: it asked for a lean back (pitch went from +0.06 to -1.0 deg in
the next 0.12 s), and a balancing robot leans back by first driving its wheels forward, so the true wheel
speed (travel / cycle length) rose from about 0.23 to 0.33 m/s for three cycles. The judged peak, 0.1315 m/s
at 4.35 s, is that real surge. The same mechanism on the robot: `QuadEncoders.py` has the same fixed 0.01 s
divisor (finding U-2), so a late cycle there would cause the same surge. Not changed: V judges the motion it
got, as with the gamepad-watchdog lurch of the sweep re-run.

## Verification

| Tier | Command | Result |
| --- | --- | --- |
| 0 | `./check_container_mount.sh` (host) | `ok    the container sees this checkout` |
| 0 | `python3 -I sim/check_sim_config.py` (host and container) | six `ok`, exit 0 |
| 0 | deliberate breaks of `check_sim_config.py` (section 1) | 3 new red, 11 older red, control green |
| 0 | `verify_urdf.sh` (container) | `PASS: all seven expansions ... and the sim config has not drifted.`, exit 0 |
| 0 | `python3 -I -m py_compile sim/*.py`; `bash -n verify_sim.sh` | ok; ok |
| 0 | `cascade_linearisation.py` default, `--candidate yaml`, `--hardware-gains`, `--sweep` against `27e9788` | identical |
| 0 | relative-link scan over `docs/`, `README.md`, `CLAUDE.md` (scratch script: every relative link's file exists, every `#anchor` matches a heading) | 0 broken |
| 1 | `build_ros.sh` | **not run**: no C++ changed (the build from the sweep re-run is current) |
| 2 | `verify_mock.sh` | **not run**: no file the mock loads changed |
| 3 | `verify_sim.sh scenarios` (all five) | `SCENARIOS: PASS (exit 0)`: U PASS (\|pitch\| ≤ 0.052 deg, \|v\| ≤ 0.0014 m/s), I PASS (0.758 s, peak 6.28 deg), V PASS (17.0 %), S2b ok (0.149 s), AB ok (3.770 s) |
| 3 | `verify_sim.sh scenarios V`, 3 more nominal runs | 3 PASS: 17.2, 17.6, 16.2 % |
| 3 | `d+` V, 9 runs | above: 8 PASS, 1 FAIL (a harness-caused surge), 0 refused |
| 3 | default `verify_sim.sh` (S1 S2a H4 C3 C2 H6 FF) | `PASS (simulation: S1 S2a H4 C3 C2 H6 FF)`, exit 0 (C3 timed out 0.206 s after the last command) |
| — | `git diff 27e9788 -- rocko_env/rocko_env/bringup/config verify_sim.sh` | empty: no gain, threshold or YAML changed |

Tier 3 is **not** called built. Reviewer agents were **not** run in this task (it was itself the fix of a
review); the fixes are not re-reviewed.

## Not verified, and what a human should know

- **Anything about the robot.** Every result is about an estimated model with `encoder_sign:=-1` assumed.
- **A forward step of the system clock** would make V's metric remove a jump that is not there (section 3).
  Only sub-millisecond ones were seen; a real fix needs SimPlant to export the steady-clock period.
- **Why the container's clock is corrected every 30 s, and why by ~16 ms during the sweep** but under 1 ms
  idle: Docker Desktop's time sync is the likely cause; not looked up.
- **The late-cycle surge** (the `d+` FAIL) is a property of the copied robot code (U-2) and of the container's
  overruns; how often a Pi overruns is unknown.
- The sweep was **not** re-run (as asked); its `d+` verdict stays COULD NOT JUDGE in the committed results.

**Why the run stopped here:** the review findings are fixed with behaviour shown unchanged, the `d+`
refusal is explained with evidence, and the docs match; nothing else was asked.

## Commits

- `27e7f2b`: `check_sim_config.py`, a tuned gain on every joint (section 1).
- `1ca5685`: the explainability fixes (section 2).
- `74e8fe5`: V's refusal text and docstring (section 3).
- The docs commit: this entry's results, the final report's sweep numbers, the ladder, next-steps, the gap
  page and a pointer in the sweep re-run journal.
