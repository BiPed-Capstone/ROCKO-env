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

## 2. Check C3: two causes, both delays C3 could not see

### Evidence

The failure message was `command_timed_out went True only 0.15-0.16 s after a command, sooner than the
0.2 s timeout`. Since the [C3/FF fix](2026-11-03-c3-ff-flaky-checks.md) C3 has timed everything by
**send** stamps (DDS source timestamps, all on the system clock), and the node's watchdog counts from a
command's **arrival** on a monotonic clock. The two clocks are not the problem (the node's log stamps
and its messages' source stamps agree to 0.1 ms); the gap between a command's send stamp and the node
taking it off its queue is.

- **The four kept failures of 2026-11-04** all have the early True inside a long gap between two
  commands (0.285 to 0.353 s), and the False 8 to 12 ms after the command that ended the gap.
- **An instrumented copy** (scratch, not committed) recorded, for each command, when `publish()`
  returned, and ran an independent listener process on the gamepad topic, the flag, the plant and (in
  the second batch) the node's references. 16 runs: **2 failures, two different causes.**
  1. *A `publish()` call blocked after the stamp.* Run 8: command 2 has send stamp 0.0525 s, but its
     `publish()` call returned at 0.3521 s (0.2996 s later); the independent listener received it
     0.3004 s after the stamp, and the listener's plant stream had no gap above 14 ms, so the listener
     was not stalled. DiffDriveController timed out at 0.208 s, 0.208 s after command 1, the last one it
     could have had (correct), and logged `resumed` at 0.3569 s. The checker saw a True 0.156 s after
     command 2's stamp. In the other 15 runs no `publish()` took over 2 ms.
  2. *The node under test stalled.* Run 4: every command left within 0.3 ms and reached the listener
     within 1.7 ms, yet DiffDriveController logged `timed out` at 1.6377 s and `resumed` 1.4 ms later.
     The True came 0.236 s after the last command sent before 1.4377 s, so the node's own 10 ms timer did
     not run between ~1.60 and 1.637 s: the node itself was stopped, and on waking ran its timer before
     the four commands waiting in its queue. The checker saw a True 0.03 s after the latest command.
- **Reproduced on purpose:** freezing DiffDriveController with `SIGSTOP` for 0.3 s, 1.0 s into the
  drive (scratch script, no code change), gives the old check `FAIL: command_timed_out went True only
  [0.0039] s after a command` (True, then False 9 ms later: the run-4 signature).
- 24 more instrumented runs passed; there the node's longest gap between references was 0.031 s and the
  longest `publish()` call 2.4 ms.

So the stalls are the same family as in the C3/FF journal (about a third of a second, inside the
ROS 2 / Fast DDS stack, cause not found), now seen in two more places: inside a `publish()` call
**after** the middleware has stamped the message, and in DiffDriveController's own process. Neither is
a watchdog fault, and C3 can tell them apart from one, because both leave traces it can see.

### The fix: count a command only once the node certainly had it (`sim/scenario_checks.py`, `check_teleop`)

- The checker records when each `publish()` call **returned** (the command had left by then).
- DiffDriveController publishes its velocity reference once in **every** 10 ms timer cycle, so the
  references' send stamps show when the node was running.
- A command counts as handled by time t only if the node ran **two** cycles after its `publish()`
  returned and before t (`NODE_CYCLES_TO_HANDLE`; two because when the timer and a waiting command are
  ready together, rclpy runs the timer first). A True must not come sooner than 0.19 s after the latest
  handled command, at any point in the run.
- Every certain silence longer than 0.3 s (from a command's `publish()` returning to the next command's
  send stamp; the final silence is one) must hold a True by the node's first cycle 0.3 s after the
  command left. This replaces both the old per-gap window and the separate final-latency window; when
  nothing stalls the bounds are the same to within one 10 ms cycle.

**Why it is not looser where it matters:** the allowance is not a fixed tolerance (no "early by less than
0.1 s is fine"). It applies only while a command had demonstrably not left, or the node demonstrably was
not running. A watchdog that fires early while the node runs normally is still caught by any margin:
the 0.03 s break fails, and a 0.15 s timeout would fail at the final silence. The printout now shows the
longest `publish()` call and the longest gap between the node's cycles, so a run that used either
allowance says so.

Offline, run 8's numbers under the new rule: command 2's `publish()` had not returned when the True came
(0.352 > 0.208 s), so the True is judged against command 1: 0.208 s, allowed.

### Results (container, `verify_sim.sh C3`, the committed code)

| What | Runs | Result |
| --- | --- | --- |
| C3 as it was (instrumented scratch copy, same judging) | 16 + 24 | 38 pass, **2 fail** (runs 4 and 8 above) |
| C3 as it was, node frozen 0.3 s mid-drive (`SIGSTOP`) | 1 | `FAIL: ... went True only [0.0039] s after a command` |
| new C3, the same freeze | 1 | `PASS`; the printout shows the node's longest cycle gap, 0.315 s |
| **new C3** | **20 consecutive** | **20 pass**: timed out 0.202 to 0.211 s after the last command. Three runs used an allowance and were judged correctly: runs 2 and 17 had a `publish()` call block 0.303 and 0.327 s (the cause-1 signature: the node timed out mid-drive about 0.15 s after the blocked command's stamp, which the old rule would have failed), run 5 had a 0.338 s gap between commands (judged as a silence, the node timed out in it) |
| break `command_timeout_s:=1000` (scratch copy of `verify_sim.sh`) | 1 | `FAIL: command_timed_out never went True in the 2.0 s after the last command`, exit 1 |
| break `command_timeout_s:=0.03` | 1 | `FAIL: command_timed_out went True only [0.0405, 0.0397, ...] s after a command the node had processed, sooner than the 0.2 s timeout`, exit 1 (a natural 0.367 s stall in this run too) |
| break `command_timeout_s:=0.5` | 1 | `FAIL: the gamepad went silent for longer than 0.3 s at [1.95] s into the drive and command_timed_out was not True within 0.3 s of it (first True after the last command: 0.510 s)`, exit 1 |
| default `verify_sim.sh` (S1 S2a H4 C3 C2 H6 FF) | 1 | `PASS (simulation: S1 S2a H4 C3 C2 H6 FF)`, exit 0 (C3 timed out 0.203 s after the last command) |

**Not shown by a run:** cause 1 cannot be injected (nothing outside Fast DDS can stop a `publish()`
between its stamp and its send); the new rule's handling of it is shown by runs 2 and 17 above and by
run 8's numbers offline. How long the node may be stalled before C3 calls it late is not bounded: a
node that published nothing at all after a silence would fail (`deadline is None`), but one stalled for
seconds and then timing out at once would pass. That is a stall, not a watchdog fault, but it is not
reported as anything except the printed longest cycle gap.

## Not verified, and what a human should know

- **Anything about the robot.** On the robot `DiffDriveController.py` is the same Python node: if a Pi
  shows stalls like these, a gamepad command waiting in its queue can still be counted as "late" and
  the robot is told to stand still for a moment (the safe direction). Whether a Pi stalls is unknown;
  the procedure in the [C3/FF journal](2026-11-03-c3-ff-flaky-checks.md) (count `teleop command timed
  out` lines while driving tethered) also catches this.
- **What causes the stalls** (now seen inside a `publish()` call and in a whole node process): not found.
- **The ±30 % sweep was not re-run** with the new V metric; its verdicts in the stage 5c-2 journal were
  made with the old one. The offline recomputation above covers only the kept (failing) runs.
- **V runs with the gamepad watchdog firing mid-step** are still judged on the motion they produced;
  in the sweep two such runs failed (88.3, 63.4 %). Not changed: the robot really did lurch.
- Review: the reviewer agents were not run in this task (`explainability-reviewer` for
  `scenario_checks.py`, `docs-reviewer`). No §6 critical-path file changed.
- Tier 3 is **not** called built.

**Why the run stopped here:** both verifiers are fixed with their evidence and checks recorded; the
remaining items need a human or a later task.

## Commits

- `70cb870`: the V metric: `sim/scenario_checks.py` (V only), this entry's section 1, the links from
  the stage 5c-1 and 5c-2 entries, the ladder's V paragraph and row 2.3 of next-steps.
- The C3 commit: `check_teleop`, the C3 comment in `verify_sim.sh`, this section, the ladder's C3 rows
  and row 2.3.

