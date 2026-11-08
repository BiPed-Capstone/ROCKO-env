# Stage 5c-2 sweep re-run with V's corrected overshoot metric

- **Date / commit:** 2026-11-07 (the files say 2026-11-08: the sweep ran past midnight). Every run was made
  on `03238ca`, clean tree (every row of the CSV says `03238ca`). Follows
  [stage 5c-2](2026-11-03-stage5c2-sweep.md), whose verdicts used V's old overshoot metric (the highest
  exported sample), and [the V metric change](2026-11-07-c3-v-verifier-noise.md) (`70cb870`), which left
  the sweep un-re-run.
- **Branch:** `ai-forward`. Local commits only: this run was told not to push.
- **Environment:** `rocko-env-dev` container (`ros:jazzy`, arm64, Docker Desktop). `./check_container_mount.sh`
  printed `ok    the container sees this checkout`; `ps` in the container showed only the ROS daemon before
  the run; `build_ros.sh` printed `Summary: 2 packages finished`. This run was the only user of the container.

**Everything here is about a model with estimated, unmeasured parameters
([sim-plant.md §4](../architecture/sim-plant.md)), with `encoder_sign:=-1` ASSUMED
([proposal 0006](../proposals/0006-velocity-loop-sign-check.md)).** The strongest claim is "balances in
simulation under an estimated model". Nothing here is about the robot. No gain, threshold or controller
YAML was changed, and the 30 % limit is the one it was.

## Goal and success criteria

1. Re-run the stage 5c-2 sweep exactly as before (same grid, V three times per case, same thresholds) with
   the corrected V metric, into **new** files; keep the old ones.
2. Report the counts, which corners fail and why, how verdicts moved against the old metric, and whether the
   corner `m+h+k−d+` still fails.

## Command

```bash
docker compose -f docker/compose.yml exec -T ros2 python3 /ros2_ws/src/ROCKO-env/sim/parameter_sweep.py run \
  --stem /ros2_ws/src/ROCKO-env/sim/results/stage5c2-sweep-vmetric2
```

25 cases, 125 launches (the old sweep took about 33 minutes; the wall time of this one was not recorded). Results:
[`sim/results/stage5c2-sweep-vmetric2.csv`](../../sim/results/stage5c2-sweep-vmetric2.csv) (one row per run) and
[`sim/results/stage5c2-sweep-vmetric2.md`](../../sim/results/stage5c2-sweep-vmetric2.md) (written by the script).
The 2026-11-03 files `stage5c2-sweep.csv` and `.md` are kept and are the old-metric record.

## Result

**Cases by verdict: PASS 18, MARGINAL 4, FAIL 2, COULD NOT JUDGE 1** (old metric: PASS 14, MARGINAL 4,
FAIL 7). **No case fell, and U and I passed in all 25 cases** (U late |pitch| at most 0.131 deg, I settle
0.63 to 1.39 s; the same picture as before).

| Case | Old metric (2026-11-03) | New metric | V overshoot, 3 runs, new [%] |
| --- | --- | --- | --- |
| nominal | FAIL (1/3) | PASS | 15.0-24.3 |
| `h−` | FAIL (1/3) | PASS | 15.0-15.8 |
| `k−` | MARGINAL | PASS | 15.3-21.6 |
| `m+h+k+d+` | FAIL (1/3, watchdog run 88 %) | PASS | 15.7-18.6 |
| `m+h+k+d−` | MARGINAL | PASS | 15.3-19.1 |
| `m−h−k+d−` | FAIL (1/3) | PASS | 15.3-16.9 |
| `m−h−k−d+` | FAIL (1/3, watchdog run 63 %) | PASS | 15.1-23.3 |
| **`m+h+k−d+`** | **FAIL (3/3)** | **FAIL (3/3)** | **45.3, 46.7, 46.1** (old: 48.9-50.9) |
| `m+` | PASS | **FAIL (1/3)** | 15.5, **68.0**, 24.0 |
| `d+` | FAIL (1/3) | **COULD NOT JUDGE** (2 of 3 runs) | 15.5 and two runs with no verdict |
| `m+h−k+d−` | PASS | MARGINAL | 17.1-29.3 |
| `m+h−k−d−` | PASS | MARGINAL | 15.1-27.9 |
| `m+h+k−d−` | MARGINAL | MARGINAL | 17.2-27.7 |
| `m−h−k−d−` | MARGINAL | MARGINAL | 14.7-29.9 |
| the other 11 cases | PASS | PASS | 14.7 to 23.3 at most |

(m = body mass 1.4 / 2.6 kg, h = centre-of-mass height 0.14 / 0.26 m, k = stall torque 0.542 / 1.007 N m,
d = IMU delay 14 / 26 ms.) Of the 25 cases, 7 went from FAIL or MARGINAL to PASS (the first seven rows),
2 PASS cases became MARGINAL, 1 PASS case became FAIL, 1 FAIL became COULD NOT JUDGE, and 14 kept their verdict.

### The corner still fails, for the same reason

`m+h+k−d+` fails V in all three runs: 45.3, 46.7 and 46.1 % against the 30 % limit, rise to 90 % in
0.32-0.33 s. This is the "expected about 47 %" and is the 46.7, 46.8, 46.5 % of the metric-change journal
again. The linear model predicts 44.8 % for the same case. It also has the slowest I (1.39 s, peak 12.2 deg).
It is the only case that fails in every run, and the only failure that the plant itself, not a verifier
artefact, explains.

### The two other non-PASS verdicts that are not noise in the metric

- **`m+` V run 2: FAIL, 68.1 % (highest exported sample 67.0 %).** The run printed `command_timed_out went
  True while the gamepad was driving`: the fake gamepad's commands did not reach `DiffDriveController` for
  longer than its 0.2 s watchdog, the reference dropped to zero and returned mid-step, and the robot
  lurched. This is the same cause as the two 88 % and 63 % runs of the old sweep (the container's delivery
  stalls, [C3/FF journal](2026-11-03-c3-ff-flaky-checks.md)). The corrected metric cannot remove it: the
  motion really happened. The other two runs of `m+` were 15.5 and 24.0 %. Not changed.
- **`d+` V runs 1 and 3: COULD NOT JUDGE**, `two plant samples have the same or decreasing stamps, so a
  period is unknown` (run 2 passed, 15.5 %). The metric's safeguard refused to judge a run with a repeated
  or backwards sample stamp, and `verify_sim.sh` exited 2. The rise time and peak pitch are recorded
  (0.93-0.95 s, 4.4 deg) and U and I passed, but V has no overshoot number for those two runs. **The cause
  of the repeated stamps is not known** (a message delivered twice, or two plant samples stamped with the
  same clock reading, are both plausible; neither was checked), and the failing runs' samples were not
  inspected. This is a new failure mode of the corrected metric that the 20 nominal runs of the metric-change
  journal did not show; `d+` is the case with the longest single-parameter delay (26 ms). **Not fixed here.**
  It is a refusal to judge, not a pass: the case verdict says so. **Cause found later the same day:**
  neither guess; the container's system clock, which stamps the messages, stepped back about 16 ms
  (the [review-fixes journal](2026-11-07-stage5c-review-fixes.md#3-why-v-could-not-judge-two-d-runs-the-containers-system-clock-stepped-back)).

### MARGINAL cases (all runs pass, one above 27 %)

`m+h+k−d−` 27.7, `m+h−k+d−` 29.3, `m+h−k−d−` 27.9 and `m−h−k−d−` 29.9 %. All four are the 14 ms delay
corners with the other two runs near 15-20 %. Four are the same count as before, but only two are the same
cases: the old marginal `k−` and `m+h+k+d−` are now PASS, and two old PASS cases (`m+h−k+d−`, `m+h−k−d−`)
are marginal.

**The corrected overshoot can be larger than the highest exported sample** (`m+h−k+d−` run 2: judged 29.3 %,
highest exported 27.2 %; `m−h−k−d−` run 3: 29.9 against 24.9 %). That is possible by design: the metric
recomputes the velocity with each cycle's real length, which is smaller than the exported value after a
late cycle but can be larger when the exported replica read an early or short cycle low (its divisor is
fixed at 0.01 s). The two runs are inside the limit, but 29.9 is within 0.1 of it. The 27 % MARGINAL rule is
the sweep's, not the metric's, and was not changed.

### Watchdog notes

`command_timed_out went True while the gamepad was driving` appears in 23 of the 75 V runs' logs (old sweep:
18 of 75). Of the 23, one failed (`m+` run 2, above); the others passed. The delivery stalls were still
there; the metric change removed their effect on the *encoder replica*, not their effect on the *gamepad
reference*.

## What this says, at most

Under an estimated model, with the corrected V metric: the tuned sim gains hold the robot up and pass U and
I at every ±30 % combination of mass, centre-of-mass height, motor gain and IMU delay, and V follows the step
within the 30 % limit in all but one corner (a real overshoot of about 46 %), one case with a watchdog
lurch in one run, and one case where V could not be judged twice. Of the old sweep's five noisy single-run FAILs, three cases
(nominal, `h−`, `m−h−k+d−`) now pass in all three runs, and `d+` could not be judged in two runs. It says
nothing about the robot.

## Verification

| Tier | Command | Result |
| --- | --- | --- |
| 0 static | `./check_container_mount.sh` | `ok    the container sees this checkout` |
| 1 build | `build_ros.sh` (container) | `Summary: 2 packages finished`, exit 0 (no C++ changed) |
| 2 mock | `verify_mock.sh` | **not run**: no file the mock loads changed |
| 3 sim | `parameter_sweep.py run --stem .../stage5c2-sweep-vmetric2` (25 cases, 125 runs, on `03238ca`) | COULD NOT JUDGE 1, FAIL 2, MARGINAL 4, PASS 18; no fall; script exit 0 |
| — | `git diff 03238ca -- rocko_env/rocko_env/bringup/config` | empty (no YAML touched) |

The run itself changed no code. The reviewer agents were not run for this entry (docs and results only).
Tier 3 is **not** called built here.

**Reproducibility record** ([reproducing-results.md](../guides/reproducing-results.md)):

```
Result:        25 cases: PASS 18, MARGINAL 4, FAIL 2, COULD NOT JUDGE 1 (V overshoot, corrected metric)
Date:          2026-11-07/08
Commit:        03238ca (every CSV row)
Launch:        as the stage 5c-2 journal; scenario launch arguments in sim/results/stage5c2-sweep-vmetric2.md
Hardware:      SimPlant (simulated); not the mock, not the robot
Config:        rocko_sim_controllers.yaml at 03238ca: pitch p 15.0 d 1.0 (gyro D), velocity p 25.0 i 0.0 d 0.2
Battery:       n/a (sim; battery_voltage_V 12.0)
Surface:       n/a (sim)
Logs:          sim/results/stage5c2-sweep-vmetric2.csv (committed); verify_sim.sh output per call in the
               container under /tmp/parameter_sweep_logs (until it restarts)
```

## Not verified, and what a human should know

- **Anything about the robot.** Estimated parameters, an assumed encoder sign, and the omissions in
  [sim-to-real-gap.md](../architecture/sim-to-real-gap.md).
- **Why `d+` produced repeated sample stamps** (above), and whether more cases would, with more runs. A
  verifier that sometimes cannot judge is better than one that guesses, but it still needs a cause.
  (Found afterwards: a step back of the container's system clock; see the review-fixes journal linked above.)
- **Variability of a re-run:** one re-run, one sample. The nominal case now has 3 PASS runs here and 20 of 20
  in the metric-change journal; the 7 verdict changes are consistent with removing the artefact, but a
  second re-run was not made, and `m+` and `d+` show that a single case can still change verdict for
  reasons outside the plant.
- **The watchdog lurch** (`m+` run 2) is still a V failure caused by the test harness delivering commands
  late, not by the model.
- Not swept, as before: wheel radius, friction, the encoder sign, inertia separately from mass, the IMU
  bias, the initial pitch, and the `tune-gains` held-out set.

**Why the run stopped here:** the sweep is re-run, compared and written down; the remaining items (the
repeated-stamps cause, the harness's delivery stalls, any decision on V) are a later task's or a human's.
