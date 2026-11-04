# Stage 5: find why the model drifts off, and write the balance scenarios before any tuning

- **Date / commit:** 2026-11-03, built on `23c1117`. Stage 5 of the run that started in
  [2026-10-31-sim-plant-and-critical-fixes.md](2026-10-31-sim-plant-and-critical-fixes.md) (stages 2
  to 4 are there). Commits: (a) the diagnosis, this entry's first version, finding H-12 and proposal
  0006; (b) the scenario code with its thresholds; (c) the results below. Each is named under
  [Commits](#commits).
- **Branch:** `ai-forward`. Local commits only: this stage was told not to push.
- **Environment:** laptop (macOS, Python 3, standard library) for the linear analysis;
  `rocko-env-dev` container (`ros:jazzy`, arm64 on Docker Desktop) for the build and `verify_sim.sh`.
  `./check_container_mount.sh` printed `ok    the container sees this checkout` at the start.

**Everything in this entry is about a model with estimated, unmeasured parameters and assumed signs
([sim-plant.md §4](../architecture/sim-plant.md)).** Nothing here says the real robot balances, or that
any gain is good or bad.

## Goal and success criteria

The person's goal for stage 5b, "scenarios first": establish why the simulated robot drifts off and
falls about 10 s after release with the hardware gains (next-steps row 2.7), record it as a finding
and a proposal without hiding it, and commit the three balance scenarios with numeric thresholds
**before any tuning**.

1. **Diagnosis recorded.** The cause, its numbers, how sure we are and what is not established are in
   this entry; a finding (H-12) in [balance-loop-findings.md](../claude-analysis/balance-loop-findings.md);
   next-steps row 2.7 updated; [proposal 0006](../proposals/0006-velocity-loop-sign-check.md) asks a
   human for the robot sign check, and is in the proposals index.
2. **The sim's sign assumption is explicit.** The scenarios run with `encoder_sign:=-1`, labelled as an
   assumption every time it is printed; one characterisation scenario, AB, keeps the as-built signs
   (`encoder_sign:=+1`) and is expected to fall, with its time to fall recorded.
3. **Three scenarios exist as checks:** U (hold upright 10 s), I (recover from a push), V (follow a
   velocity step), each with numeric thresholds in one place (the scenario block at the top of
   `verify_sim.sh`), each threshold's reason, unit, frame and sign in a comment.
4. **S2b:** U with the pitch loop's sign flipped fails, and for the right reason (the pitch diverges,
   not a launch error).
5. **The scenarios are a separate mode** (`verify_sim.sh scenarios`) with a meaningful exit status;
   the default `verify_sim.sh` still runs S1 to FF and still gates.
6. **Thresholds committed before tuning:** commit (b)'s SHA recorded here; the scenarios then run once
   with the unchanged gains, and the raw results recorded. No gain is touched in this stage.

## Plan

1. Commit the diagnosis script `sim/cascade_linearisation.py` (written in the diagnosis step, before
   this entry) with this entry, finding H-12 and proposal 0006. Tier: the script's own self-checks
   against sim-plant.md §7.1 (laptop).
2. Add the scenarios to `sim/scenario_checks.py` (subcommands `upright`, `impulse`, `velocity-step`)
   and a `scenarios` mode to `verify_sim.sh`. Use only launch arguments that already exist
   (`encoder_sign`, `pitch_sign`, `impulse_time_s`, `impulse_angular_velocity_deg_per_s`,
   `initial_pitch_deg`); no change to `SimPlant`, no change to any YAML, so `sim/check_sim_config.py`
   needs no new rule. Tier 3 (the model).
3. Commit (b). Then run `verify_sim.sh scenarios` once and the default `verify_sim.sh` once; record.
4. Deliberate breaks, for each new check, done after commit (b) so they cannot shape a threshold.

Risks: a runner bug found only by running (fixed in its own commit, thresholds untouched, so git shows
it); and the sign assumption being read as a fix (every printout and this entry label it).

## Diagnosis: the outer velocity loop is positive feedback under the sim's default signs

**Cause (in the model).** With the sim's default signs (`pitch_sign = −1`, `encoder_sign = +1`) and
every hardware gain positive, the outer velocity loop is **positive feedback at DC**. It has a real
closed-loop pole in the right half-plane, so the measured velocity runs away in one direction, and the
pitch loop leans the robot ever further to chase it until it falls. That is the slow drift seen in
stages 3 and 4. It is the run-away that [sim-plant.md §2.4](../architecture/sim-plant.md) predicted
in writing; stage 5 shows that the slow pole is that run-away and puts numbers on it.

**How it was shown.** `python3 -I sim/cascade_linearisation.py` (laptop, standard library, about 7 s).
It linearises `SimPlant` about upright, reads the gains and the D-term wiring from the two YAML files,
and closes both loops exactly as `rocko_sim_controllers.yaml` wires them: continuous time first, then
discrete time as the sim runs (100 Hz, zero-order hold, the 20 ms IMU delay on pitch, no delay on
`pitch_rate`, the `QuadEncoders.py` replica of finding U-2, and `pid_controller`'s update order). It
checks itself against sim-plant.md §7.1 #4 (braked open-loop poles +6.9306, −2.0304, −12.1916 s⁻¹) and
§4's inertia and motor constants, and stops with an assertion if they differ.

**Numbers (estimated model; run 2026-11-03 on `23c1117`'s YAML files):**

| What | `encoder_sign = +1` (sim default, the authors' intent) | `encoder_sign = −1` |
| --- | --- | --- |
| §2.4 eq. (1) reduction, outer loop roots [1/s] | **+0.854**, −0.402: constant term < 0, so a real positive root for **any** velocity `i` > 0 | +0.087 ± 0.571j: negative feedback at DC, but unstable because `k·Kpv < c` (needs `Kpv` > 4.60 deg/(m/s); the YAML has 2.0) |
| Whole cascade, continuous, slowest mode [1/s] | **+0.805** (real) | +0.109 ± 0.573j |
| Whole cascade, discrete as the sim runs (H-6, 20 ms), slow mode / fast pair | s = **+0.816 /s** (z = 1.0082 per cycle) / \|z\| = 0.9896 (decays) | s = +0.104 ± 0.576j / \|z\| = **1.0183** (the ~3.5 Hz pitch oscillation **grows**, about 1.8 /s) |
| Scaling all three velocity gains by m = 0, 0.5, 1, 2, 4, 8 (discrete) | slow pole moves **right**: +0.31, +0.63, +0.82, +1.10, +1.51, +2.07: more gain, faster run-away. That is what positive feedback does | slow pole moves left (to −0.34 at m = 8) while the fast pair grows (\|z\| 1.004 to 1.109) |

The parent run's diagnosis summarised this as "real pole +0.82 /s"; the discrete figure is +0.816 /s.

**With the other encoder sign, the hardware gains still do not hold the model.** `encoder_sign = −1`
removes the run-away at DC, but two things remain, both in the inner loop's gains, not in a sign:

- The pitch loop alone is marginal at the estimated 20 ms IMU delay: its fast pair has \|z\| = 1.0042
  per cycle (unstable), and scaling the pitch loop's output down to about 0.88 of today's would be
  needed to bring it to 1 (section D of the script: ×0.75 gives 0.9948, ×1.0 gives 1.0042), a gain
  margin of about **−1.1 dB**. (Stage 4's runs, with the run-away present, did not show this growing
  oscillation; the run-away tipped the model first.)
- The velocity loop's D term takes damping away from that pair when the signs are right (section C2:
  D alone moves \|z\| from 1.004 to 1.017 with `encoder_sign = −1`).

The parent diagnosis observed this in the sim as a growing oscillation of about 2.5 Hz with a fall at
about 10.8 s. The linear model puts the pair at about 3.5 Hz (22 rad/s); the difference was not
investigated. Section C3 of the script maps where nearby gains would be stable: with `encoder_sign =
−1` a band exists (pitch `d` ≥ 0.2 with velocity `p` 6 to 8, wider at larger `d`); with `encoder_sign =
+1` **no** point of the map is stable, because the DC sign problem holds for any positive gains. **The
map is not a gain recommendation**; no gain was changed.

**Confidence.**

- High, for the model: three independent views agree (the closed-form slow-time-scale reduction, the
  continuous 5-state cascade, the discrete 10-state cycle-by-cycle model), the script checks its plant
  against the unit-tested numbers of §7.1, and the observed drift time (9.3 to 10 s from 2 deg) is the
  size a +0.8 /s pole gives.
- **Not established for the robot.** Whether the real robot has the same sign problem cannot be known
  from code: it depends on how the encoders are wired relative to the motor driver's direction pin. Only
  the two-step check of sim-plant.md §2.4 settles it (tank mode on a stand, then a hand tilt of the
  IMU). [Proposal 0006](../proposals/0006-velocity-loop-sign-check.md) asks a human for it. If the
  robot's encoder reads positive when a positive command drives it (the authors' intent), the real
  outer loop is positive feedback too, and the fix is a sign change in one of two places (the
  proposal lists both; a human chooses). If it reads negative, the robot's signs are right and the
  sim's default `encoder_sign = +1` is the wrong guess.
- **Not established either:** that the inner loop is marginal on the robot. The −1.1 dB margin rests on
  the estimated 20 ms IMU delay and the estimated plant; the robot's delay has not been measured.

**Why this is a finding and not a tuning job.** The person's instruction for this run: a structural
cause is a finding to fix, not a gain to tune around. Under the default signs no positive gain set
stabilises the model (C3), so tuning against `encoder_sign = +1` would mean picking negative velocity
gains, which encodes the same sign assumption in a less visible place (sim-plant.md §2.4). So the sim
now states its assumption (`encoder_sign:=-1` for the scenarios) out loud, and keeps the as-built signs
as a scenario that is expected to fall (AB), so the fault stays visible on the ladder.

## What changed

Commit (a), the diagnosis:

- `sim/cascade_linearisation.py` (new): the linear analysis above. Laptop, standard library only.
- This entry; `docs/journal/README.md` index; a pointer from the stage 2 to 4 journal.
- [balance-loop-findings.md](../claude-analysis/balance-loop-findings.md): finding **H-12** (row and section).
- [next-steps.md](../next-steps.md) row 2.7: cause established under the model, robot check pending,
  fix lands as a proposal.
- [proposal 0006](../proposals/0006-velocity-loop-sign-check.md) (new) and the proposals index.
- [sim-plant.md §7.2](../architecture/sim-plant.md): the "not yet explained" paragraph now points at H-12.

Commit (b), the scenarios (thresholds before tuning):

- `verify_sim.sh`: a `scenarios` mode (`verify_sim.sh scenarios [U I V S2b AB]`), the scenario
  thresholds block (the one place they are written), and the five scenario runs. The default run is
  unchanged: no argument still means `S1 S2a H4 C3 C2 H6 FF`.
- `sim/scenario_checks.py`: subcommands `upright`, `impulse`, `velocity-step`. Each records
  `/dynamic_joint_states` from SimPlant's release stamp (read from the launch log, so t = 0 is the same
  zero SimPlant's impulse time uses), writes `samples.csv` and `metrics.json`, and exits 0 / 1 / 2.
- [verification-ladder.md](../architecture/verification-ladder.md): the scenarios mode and what it
  proves.

No change to `SimPlant`, to any YAML or to any launch file: the scenarios use launch arguments that
already existed. So `sim/check_sim_config.py` needed no new drift rule.

## The scenario thresholds (as committed in (b))

All in the scenario block at the top of `verify_sim.sh`, each with its reason in a comment. Pitch is the
exported pitch the controllers read (limits on its absolute value). "Measured velocity" is the exported
`velocity` state, the mean of the two wheels: about half the true speed (U-2), signed by `encoder_sign`.
Times count from SimPlant's release.

| Scenario | Launch | Pass when |
| --- | --- | --- |
| **U** hold upright | `teleop:=false`, zero velocity reference, `initial_pitch_deg:=2.0`, `encoder_sign:=-1` (assumption) | for 10 s: \|pitch\| never reaches 35 deg; from 5 s to 10 s \|pitch\| ≤ 2.0 deg and \|measured velocity\| ≤ 0.05 m/s |
| **I** recover from a push | `teleop:=false`, upright start, `impulse_time_s:=2.0`, `impulse_angular_velocity_deg_per_s:=100`, `encoder_sign:=-1` | the push shows as a ≥ 80 deg/s jump in `pitch_rate`; \|pitch\| never reaches 35 deg; within 3.0 s of the push \|pitch\| is back under 2.0 deg and stays there to 8 s |
| **V** velocity step | `teleop:=true` (the real `DiffDriveController.py`), a fake gamepad at 20 Hz: `linear.x` 0 for 1 s, then 0.1 m/s for 8 s; `encoder_sign:=-1` | measured velocity reaches 90 % of 0.1 m/s within 3.0 s of the step; overshoot ≤ 30 %; \|pitch\| < 10 deg; never 35 deg |
| **S2b** (verifier check) | U with `pitch_sign:=1` (the pitch loop's sign flipped) | "ok" only if U **fails** with \|pitch\| reaching 35 deg within 1.0 s of release and the wiring up |
| **AB** (characterisation, as-built signs) | U with `encoder_sign:=1` (sim default), 20 s | "ok" if it **falls**, no sooner than 3.0 s after release (the slow run-away); the time is printed. Not falling is a FAIL |

Exit status of `verify_sim.sh scenarios`: 0 only if U, I and V pass and S2b and AB are "ok"; 1 if any is
not; 2 if a run could not be judged and none failed. While `SCENARIOS_EXPECTED_RED=1` a failing U, I or V
prints `FAIL (expected until the sim gains are tuned)`, but the exit status is still 1, so the mode
cannot read green while the model falls.

Why S2b uses `pitch_sign:=1` and not a scratch copy of the gain file: flipping the sign of the pitch the
controllers read is the same, to the loop, as negating both pitch gains. A copy of the gains would need
the launch file to load a second YAML, which CLAUDE.md §3 asks launch files not to do, and this stage
must not touch `rocko_sim_controllers.yaml`.

## Verification

**Thresholds committed before tuning: commit `0b0e39b`** (b). No scenario had been run when it was
made. One later commit touched the runner, `9101fc0`, and only its printouts (the per-second pitch list
skipped seconds; fall times printed with 16 decimals); no threshold or pass rule changed (`git diff
0b0e39b 9101fc0`). No gain was changed in this stage; `rocko_sim_controllers.yaml` and
`rocko_controllers.yaml` are as they were at `23c1117`.

| Tier | Command | Result |
| --- | --- | --- |
| — laptop | `python3 -I sim/cascade_linearisation.py` | ran, self-checks passed (braked poles +6.9306, −2.0304, −12.1916); numbers in the diagnosis above |
| 0 static | `verify_urdf.sh` (container) | `PASS: all seven expansions ... and the sim config has not drifted.` exit 0 (`check_sim_config.py`'s six checks `ok`; it reads `TELEOP_TIMEOUT_S` and `FF_AGE_S` from `verify_sim.sh`, still found) |
| 0 static | `.claude/hooks/test-guard-protected-files.sh` (host) | 81 cases, pass (no hook change; run as a baseline) |
| 1 build | `build_ros.sh` (container) | `Summary: 2 packages finished`, exit 0 (no C++ changed in this stage) |
| 2 mock | `verify_mock.sh`, `verify_mock.sh tank` | not run: no ROS code, launch file, URDF or YAML changed |
| 3 sim, scenarios | `verify_sim.sh scenarios` at `0b0e39b` | **exit 1, as expected.** U, I, V `FAIL (expected until the sim gains are tuned)`; S2b `ok`; AB `ok`. 78 s. Details below |
| 3 sim, default | `verify_sim.sh` (no argument) at `9101fc0` | **exit 1: `C3` failed, the other 8 launches `ok`** (156.6 s). `C3` then run alone 3 times: pass, **fail**, pass. The C3 code is unchanged since `23c1117`; see *C3 is flaky* below |

**Raw scenario results, first run (`0b0e39b`, hardware gains, `encoder_sign:=-1` assumed except AB).**
Times are seconds after release; pitch is the exported pitch; velocity the mean measured velocity.

| Scenario | Verdict | What happened |
| --- | --- | --- |
| U | FAIL | No fall in 10 s, but a sustained oscillation of about **2.4 Hz** (pitch zero crossings) with amplitude about 23 deg: pitch at 1 s, 2 s, ...: −13.9, 21.2, −23.0, 23.4, −23.7, −19.9, 23.0, 11.9, 1.7 deg (the first printout skipped one second). Reason printed: `\|pitch\| reached 30.252 deg between 5.0 and 10.0 s (limit 2.0 deg)`. Velocity reached 2.05 m/s, 1.41 m/s at 10 s |
| I | FAIL | The push **was applied**: a 98.7 deg/s jump in `pitch_rate` at 2.016 s (the first end-to-end run of SimPlant's impulse). Then an oscillation of about 2.6 Hz grew and the robot fell: `\|pitch\| reached 35.0 deg 4.514 s after it` (6.514 s after release). Before the push the pitch was exactly 0 (exact equilibrium) |
| V | FAIL | Step to 0.1 m/s at 1.043 s; an oscillation of about 2.7 Hz grew; peak mean velocity 2.08 m/s (overshoot 1977 %); `\|pitch\| reached 35.0 deg 6.694 s after release (5.651 s after the step)` |
| S2b | ok | U with `pitch_sign:=1`: `\|pitch\| reached 35.0 deg 0.194 s after release` (limit 1.0 s); the linear model's ~0.2 s |
| AB | ok | As-built signs: the slow drift, pitch at 1 s, 2 s, ...: −0.03, 0.17, 0.75, 0.88, 1.14, 2.05, 4.25, 9.10, 21.2 deg, then lying at 60 deg; **fell at 9.400 s** (stage 4 saw 9.34 to 9.42 s). Velocity −0.65 m/s at 8 s, −1.46 m/s at 9 s: it runs away in one direction |

So with `encoder_sign:=-1` the failure is the growing ~2.4-2.7 Hz pitch oscillation the linear model
predicts (it puts the pair at ~3.5 Hz; the difference is not explained), and U, at 10 s, is a large
bounded-looking oscillation rather than a fall (the diagnosis run before this stage, with the same
signs, fell at about 10.8 s; U's record ends at 10 s). With the as-built signs the failure is the slow one-way run-away of H-12.
Two different failures, each where the analysis put it.

**Deliberate breaks** (scratch copies in the container's `/tmp/brk`, never in the repo; each run alone
with `verify_sim.sh scenarios <name>`):

| Break | Result |
| --- | --- |
| S2b without the flip (`pitch_sign:=-1`) | `SCENARIO S2b: FAIL: U failed, but not by a fast fall within 1.0 s (fell_at_s: None)`, exit 1: S2b refuses a failure for the wrong reason |
| AB with `disable_torque:=true` | `SCENARIO AB: FAIL: fell 0.449 s after release, sooner than 3.0 s: not the slow run-away of H-12`, exit 1 |
| I with the push size 0 | `SCENARIO I: FAIL ...: no push seen: the largest pitch_rate jump near 2.0 s was 0.0 deg/s (need >= 80)`, exit 1 |
| V with `command_timeout_s:=0.001` (the watchdog zeroes nearly every command) | `FAIL ...: overshoot 572.4 %`: **not a clean break** (a few commands still got through, and with these gains any kick grows), so it was replaced by the next row |
| V with the fake gamepad on a topic nobody reads (a scratch copy of `scenario_checks.py`) | `SCENARIO V: FAIL ...: the mean measured velocity did not reach 0.09 m/s within 3.0 s of the step`, exit 1 (the robot stood exactly still) |
| U from exactly upright (`U_START_DEG=0.0`) | `SCENARIO U: PASS`, `SCENARIOS: PASS (exit 0)`: the pass branch and exit 0 work, and a start at exactly 0 passes trivially, which is why U starts at 2 deg |

U's own red needs no break: it is red now. The pass branches of I and V, and the `COULD NOT JUDGE` /
exit 2 path of the scenarios mode, were **not** exercised.

**C3 is flaky (an observation, not caused by this stage).** In the default run `C3` failed with
`command_timed_out went True while commands were still arriving` (the flag went True 1.03 s into the 2 s
drive and False again 8 ms later). Run alone 3 more times it passed, failed (`True 0.003 s after the last
command`, with 40 commands taking 2.15 s instead of 1.95 s), and passed. Both failures fit a gap of 200
ms or more in the fake gamepad's publishing or its delivery, the same kind of gap check `FF` already notes
for its publishers. The C3 code is unchanged since `23c1117` (`git diff 23c1117 -- sim/scenario_checks.py`
shows additions only). Not investigated; recorded in the ladder. So the default run is **not** green on
this commit, for a reason outside this stage's change.

## Review rounds

None in this stage: the person's task list for stage 5b did not include the reviewer agents, and this
run was told not to start any. The diff is waiting for `explainability-reviewer` (code), `docs-reviewer`
(docs) and `ros2-control-reviewer` (it touches a verify script, not ROS code; optional). No §6
critical-path file changed, so `realtime-auditor` is not required.

## Not verified — needs a human

- **The robot's signs** (proposal 0006): the two-step check of sim-plant.md §2.4. Until it is done, the
  scenarios' `encoder_sign:=-1` is an assumption, and AB's fall says nothing about the robot.
- Everything the model omits (sim-plant.md §6) and every estimated parameter (§4).
- The pass branches of scenarios I and V (only U's was seen, from an exactly upright start), and the
  scenarios mode's `COULD NOT JUDGE` / exit 2 path.
- Why `C3` is flaky in the container (2 of 4 runs failed on 2026-11-03). Found and fixed later the same day: [2026-11-03-c3-ff-flaky-checks.md](2026-11-03-c3-ff-flaky-checks.md).

## Reproduce

On the laptop, from the repo root:

```bash
python3 -I sim/cascade_linearisation.py          # the diagnosis, ~7 s, standard library only
```

In the dev container (start it from this checkout first: `docker compose -f docker/compose.yml up -d`,
then `./check_container_mount.sh` must print `ok`):

```bash
docker compose -f docker/compose.yml exec -T ros2 bash -c 'cd /ros2_ws && src/ROCKO-env/build_ros.sh'
docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_sim.sh scenarios
docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_sim.sh
```

## Commits

- (a) diagnosis, H-12, proposal 0006: the commit that adds this file. It already carries the plan for
  (b) and the threshold table above, written before (b) was committed; the code in (b) implements that
  table.
- (a) is `6ea1355`.
- (b) scenarios and thresholds: **`0b0e39b`, thresholds committed before tuning.**
- `9101fc0`: printouts only (see *Verification*).
- (c) the results above, the ladder, next-steps row 2.3 and the two skills' status lines: the commit
  that adds this section.

**Why the run stopped here:** the stage's task list is done (scenarios first, no tuning). Next, for
whoever picks it up: the reviewers listed under *Review rounds*; the C3 flake; then tuning the **sim**
gains against U, I, V under the labelled `encoder_sign:=-1` assumption, inner loop first
(`tune-gains`), with the ±30 % sweep and held-out scenarios before any claim.
