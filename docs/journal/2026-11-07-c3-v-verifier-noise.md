# Two noisy verifiers found by the sweep: scenario V's overshoot metric, and check C3's second flake

- **Date / commit:** 2026-11-07, built on `77664f4` (the stage 5c-2 results). Two commits, one per
  verifier, listed under [Commits](#commits). Follows [stage 5c-2](2026-11-03-stage5c2-sweep.md), which
  found both problems and left them open.
- **Branch:** `ai-forward`. Local commits only: this run was told not to push.
- **Environment:** `rocko-env-dev` container (`ros:jazzy`, arm64, Docker Desktop on macOS, 14 CPUs).
  `./check_container_mount.sh` printed `ok    the container sees this checkout` at the start and again
  when the run resumed after an interruption. This run was the only user of the container; before each
  batch `ps` in the container showed only the ROS daemon. `build_ros.sh`: `Summary: 2 packages finished`
  (no C++ changed in this run).

**Everything here is about the model and the checks around it.** Nothing says anything about the real
robot. No gain, no threshold and no controller YAML was changed.

## Goal and success criteria

A verifier that fails at random is a broken sensor (CLAUDE.md §4). Fix both without loosening what
they prove.

1. **V:** quantify the noise in V's overshoot from recorded samples. If it is measurement noise, change
   the **metric**, not the 30 % limit, in its own commit with this paragraph: what changed, why, the
   numbers before and after on the same recorded runs. S2b still fails fast, U and I unchanged. V 20 times
   at nominal (aim: 0 failures), and the corner `m+h+k−d+` 3 times: it must still **fail**, for the real
   reason; if the new metric passed it, the change would hide a real failure and is reverted.
2. **C3:** find the cause of the new failure mode (`command_timed_out went True only 0.15-0.16 s after a
   command`) from evidence, fix it so the check judges only what it can know, 20 consecutive green runs,
   red on the deliberate breaks `command_timeout_s` = 1000, 0.03 and 0.5.

## 1. Scenario V: the overshoot is judged on the velocity with each cycle's real length

### What the noise is (measured on the kept runs)

The failing V runs of stages 5c-1 and 5c-2 kept their `samples.csv` (11 runs). Around the largest sample
of every "noisy" run there is a **gap of 26 to 30 ms between two plant samples' stamps** where all other
gaps are about 10 ms, and the measured velocity jumps up by 15 to 20 % in that one sample, then decays
over about 0.2 s. Example, nominal run 3 of the sweep (raw overshoot 32.6 %): at 8.139 s after release
0.1142 m/s, next sample 26.7 ms later 0.1310, then 0.1326, 0.1222, 0.1211 ... The 5c-2 `d+` run (38.4 %):
0.1117 m/s, then a 30.5 ms gap, 0.1346, 0.1384, 0.1341 ...

The cause is the encoder replica, written to reproduce `QuadEncoders.py` exactly (finding U-2,
[sim-plant.md §3.7](../architecture/sim-plant.md)): each cycle it divides the wheel travel by an
**assumed 0.01 s** and low-passes it with `y = 0.95·y + 0.05·(travel / 0.01)`. When `controller_manager`
runs a cycle late in the container, SimPlant integrates the physics over the real period (as it should),
so that one cycle's travel is ~3 cycles' worth and the replica reads it as a wheel 3 times faster. A
synthetic check of the arithmetic: a wheel at a constant speed with one 30 ms cycle makes the replica's
output jump by 19.5 %, the size seen. It is **not white sampling noise**: it decays through the
replica's own filter, whose time constant is 0.01 / 0.05 = 0.2 s, about the speed of the step response
itself (its peak comes 0.3 to 0.4 s after the step in the failing corner). Late-window statistics of the
kept runs: sample-to-sample standard deviation 0.0014 to 0.0023 m/s (1.2 to 3.4 % of the speed), lag-1
autocorrelation of the residual 0.31 to 0.83, largest sample 4 to 20 % above the late mean.

### Why not a moving average

The brief suggested judging the maximum of a short moving average. Measured on the same kept runs, no
window works, because the artefact decays as slowly as the response it sits on:

| Run (raw overshoot) | 0.05 s | 0.1 s | 0.15 s | 0.2 s | 0.3 s |
| --- | --- | --- | --- | --- | --- |
| nominal, sweep run 3 (32.6 %) | 25.2 | 22.4 | 20.3 | 19.6 | 17.9 |
| nominal, trial run (37.9 %) | 37.0 | **36.0** | **33.1** | 29.8 | 24.8 |
| `d+` (38.4 %) | **35.7** | **33.3** | **31.0** | 27.2 | 21.8 |
| corner `m+h+k−d+`, 3 runs (48.9-50.9 %) | 47.1-47.8 | 43.0-44.0 | 36.5-37.8 | *28.6-29.6* | *16.9-21.1* |

A window short enough to keep the real corner failure (0.15 s or less) still fails two of the noisy
nominal-model runs; at 0.2 s the corner **passes** (28.6 to 29.6 %), which would hide a real failure.

### What changed instead (`sim/scenario_checks.py`)

V's overshoot is now the peak of the **mean measured velocity recomputed with each cycle's real length**:
`velocity_from_true_periods()` applies the replica's own formula but divides each cycle's travel by the
time between the two plant samples' stamps (the controller manager's clock, the same period SimPlant
integrated over) instead of 0.01 s. It needs the wheel travel, so V now records the exported `position`
state (and the right wheel's velocity) as well; `samples.csv` has two more columns. It works on the
difference only: it carries how far the replica's filter reads above the true-period filter and
subtracts that from the exported value, so with exactly 10 ms cycles the value is the exported one.

Safeguards, each tried on synthetic samples in the container:

- **One cycle at a time.** For every pair of samples it replays one replica cycle from the previous
  sample; it must give the exported value within 1e-6 m/s. If not, samples were dropped in between
  (this script's subscription loses messages when it stalls: 1 pair in about 880 in the runs below, once a
  0.34 s hole). Then the error is only decayed as over that many nominal cycles and nothing else is
  removed, so an artefact inside a hole stays in: the value can only look larger, never hide an
  overshoot (synthetic: a 30 ms cycle inside a 15-sample hole leaves 0.10775 of the 0.10775 spike).
  The first version demanded a perfect replay of the whole run and gave `COULD NOT JUDGE` on 2 of 9
  runs with a dropped sample; it was changed before any of the runs counted below.
- **The replica must be the one in the code.** If more than 5 % of the pairs are not one cycle apart,
  V is `COULD NOT JUDGE` (synthetic: a replica with alpha 0.1 instead of 0.05 misses 200 of 200 pairs).
  The constants (0.01 s, 0.05) are copied from `SimPlantModel.hpp` and named in the script.
- The highest **exported** sample is still computed and printed, and written to `metrics.json`
  (`exported_overshoot_pct`), next to the judged `overshoot_pct`.

**What did not change:** the 30 % limit (`V_OVERSHOOT_PCT`) and every other threshold in
`verify_sim.sh`; the rise time (still on the exported velocity); U, I, S2b and AB (they read only the
first four columns of a sample).

### Before and after, on the same recorded runs

The kept runs of stages 5c-1 and 5c-2 recorded only the mean velocity, not the travel, so for them the
same correction was made offline by inverting the replica (the mean is 0.975 × the left output, which
gives each cycle's travel back); scratch script, not committed:

| Kept run | Raw (judged then) | Recomputed with real cycle lengths |
| --- | --- | --- |
| nominal, sweep run 3 | 32.6 % FAIL | 17.8 % |
| nominal, trial run before the sweep | 37.9 % FAIL | 23.7 % |
| `h−` | 33.8 % FAIL | 16.7 % |
| `d+` | 38.4 % FAIL | 20.4 % |
| `m−h−k+d−` | 30.7 % FAIL | 17.1 % |
| stage 5c-1 iteration 5 (other gains) | 31.6 % FAIL | 28.2 % |
| **corner `m+h+k−d+`, 3 runs** | **48.9, 50.9, 49.1 % FAIL** | **48.9, 47.3, 47.7 % FAIL** |
| `m+h+k+d+` and `m−h−k−d+` (gamepad watchdog fired mid-step) | 88.3, 63.4 % FAIL | 81.3, 58.1 % FAIL |

The last row is a different cause, not touched here: the fake gamepad's commands stopped for longer than
the 0.2 s watchdog (see section 2: the same container stalls), DiffDriveController zeroed the reference
mid-step and the robot really did lurch. V judges the motion it got.

### Results with the new metric (container, `verify_sim.sh scenarios V`)

| What | Runs | Judged overshoot | Highest exported sample (the old metric) | Verdict |
| --- | --- | --- | --- | --- |
| **nominal** | **20 consecutive** | 15.9 to 24.5 %, median 17.9 % | 18.6 to 30.7 %, median 20.9 % | **20 PASS**; the old metric would have failed run 10 (30.7 %, judged 18.2 %) |
| **corner `m+h+k−d+`** (`SIM_PLANT_OVERRIDES="body_mass_kg:=2.6 com_height_m:=0.26 motor_stall_torque_Nm:=0.5423075 imu_delay_ms:=26"`) | 3 | **46.7, 46.8, 46.5 %** | 46.3, 46.0, 50.5 % | **3 FAIL**, `overshoot ... (limit 30.0 %)`: the real failure is still caught |
| corner, an earlier version of the code (dropped samples gave could-not-judge) | 3 | 47.8 (from its `samples.csv`), 45.9, 43.8 % | 47.2, 45.3 % | 1 COULD NOT JUDGE, 2 FAIL |

In 9 of the 20 nominal runs `command_timed_out` went True while the gamepad was driving (the gamepad
stall above); all 9 passed. 6 of the 20 had one or two pairs with dropped samples; all passed.

Then `verify_sim.sh scenarios U I S2b`: `U: PASS` (|pitch| ≤ 0.063 deg, |v| ≤ 0.0016 m/s from 5 s),
`I: PASS` (back under 2 deg 0.761 s after the push, peak 6.4 deg), `S2b: ok` (fell 0.149 s after
release, limit 1.0 s), exit 0: the same as stage 5c-1 (U 0.048-0.050 deg, I 0.77 s, S2b 0.142 s).
Tier 0: `python3 -I -m py_compile sim/scenario_checks.py` ok; `python3 -I sim/check_sim_config.py`
(host) all `ok`.

**What this says, at most:** V's verdict no longer depends on how late the container ran a control cycle.
It says nothing new about the model or the robot: on the robot `QuadEncoders.py` has the same fixed
0.01 s divisor (finding U-2), so a late cycle there gives the same false jump, and that is a property of
the robot's code that the sim copies on purpose, not something this metric fixes.

## 2. Check C3

In the next commit (this section is written there).

## Commits

- The V metric commit: `sim/scenario_checks.py` (V only), this entry's section 1, the links from the
  stage 5c-1 and 5c-2 entries, the ladder's V paragraph and row 2.3 of next-steps.

