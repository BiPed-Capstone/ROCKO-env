# Final report, stages 4 to 6: what was checked, what was not, and what a human does next

- **Date / commit:** 2026-11-04, written on top of `77664f4`. The commit that adds this file is the
  one after it. This is a **report**: it adds no behavior and cites the journals that hold the
  evidence.
- **Branch:** `ai-forward`. Local commits only for stages 5c and this report (those runs were told
  not to push).
- **Environment:** the evidence was gathered in the `rocko-env-dev` container (`ros:jazzy`, arm64,
  Docker Desktop on macOS; `controller_manager` 4.48.1, `pid_controller` 4.42.1) and on the laptop
  (Python 3, standard library). **Nothing ran on a robot or a Raspberry Pi.**

> **STATUS AT WRITING (the orchestrator edits this box).**
> - **Stage 6 (finding C-1, the blocking service calls in the 100 Hz loop) is partly done
>   (2026-11-07): helper + test + proposal; the plugin is unchanged.** Landed: the BNO055 UART framing
>   as an unwired header (`Bno055UartProtocol.hpp`) with a host test, and
>   [proposal 0010](../proposals/0010-c1-imu-uart-and-encoders-in-cpp.md) for the rest
>   ([journal](2026-11-07-c1-bno055-uart-framing.md)). `ICM20948.cpp` and the encoder reads still make
>   the blocking service calls; nothing that runs on the robot changed. Lines below that say stage 6
>   is not started were true when written.
> - **Tier 3 is now marked built** (2026-11-07, the orchestrator's decision after review) in the
>   [verification ladder](../architecture/verification-ladder.md), which is the only home of that status. **Tier 3 is
>   built: a model with ESTIMATED, unmeasured parameters (row 2.2/proposal 0007 not done), `encoder_sign:=-1`
>   ASSUMED (proposal 0006 not checked), scenarios U/I/V pass with sim-tuned gains that differ from the hardware
>   gains (candidate only: proposal 0008), S2b fails as it must; in the ±30 % sweep one corner fails V overshoot
>   (`m+h+k−d+`, about 46 %), four are marginal, one case could not be judged (clock step), and one real V failure
>   (31.5 %) followed a late control cycle; it says nothing about whether the robot balances.** Text below that says
>   "not called built" was true when written (2026-11-04).
> - Proposals 0003 to 0009 are all **Open**. No gain, threshold or version pin was changed.
> - Review status of the last code: see (b) and (e) below; this box does not claim a clean review.
> - Anything in this report that disagrees with the ladder or [next-steps](../next-steps.md): they win.
> - Updated 2026-11-07: the sweep numbers, V's verdict noise and C3's flake in (a), (b) and (c) follow
>   the later journals ([V metric and C3](2026-11-07-c3-v-verifier-noise.md),
>   [sweep re-run](2026-11-07-stage5c2-sweep-rerun.md), [review fixes](2026-11-07-stage5c-review-fixes.md)).
>   Tier 3 is still not marked built and stage 6 is still not started (true on 2026-11-04; superseded by the bullets above).
> - Updated 2026-11-08: stages 4 and 5 pushed (4374d5c); stage 6 partly done (framing helper, host test
>   389 checks, proposal 0010; plugin unchanged); H4/C3/FF/V verifiers made robust (journals
>   2026-11-03..11-07); Tier 3 built with limits; robot-side items all need a human (proposals 0006-0010).
> - Still open, 2026-11-08: check S2a (no torque, passes 35 deg within 0.48 s) failed once in a default
>   `verify_sim.sh` run at 0.749 s, then passed 6 of 6 alone at 0.430 to 0.440 s. Its margin is about 0.04 s,
>   and a 0.3 s container stall explains the one failure; the cause was not shown and the check was not
>   changed. It is the fifth timing-based check seen to fail at random (C3, FF, V, H4, S2a): until the
>   stalls are understood, a single red in the default run is not proof of a regression.

> **The strongest claim in this whole report is "balances in simulation under an estimated
> model".** The balancing of the real robot was not checked and cannot be by an agent (CLAUDE.md §4).

## (a) The highest tier reached, per claim

[What each tier proves](../architecture/verification-ladder.md); the evidence is in the journals named.

| Claim | Highest tier | Evidence and limit |
| --- | --- | --- |
| The tree builds (both packages, no motor plugin) | Tier 1 | `build_ros.sh` exit 0 in [stage 5c-2](2026-11-03-stage5c2-sweep.md). The motor plugin is **not** compiled in the container; `check_motor_plugin_compiles.sh` type-checks it only (syntax and types, stub headers) |
| The URDF expands in all modes; the sim config has not drifted | Tier 0 | `verify_urdf.sh` PASS; `sim/check_sim_config.py` six `ok`; hook tests 81 cases ([5c-1](2026-11-03-stage5c1-sim-gain-tuning.md)) |
| The controllers load, claim interfaces and activate (mock) | Tier 2 | Last passed in the stage 4 round 3 preflight ([stage 4 journal](2026-10-31-sim-plant-and-critical-fixes.md)); **not rerun in stage 5** (no file the mock loads changed). Says nothing about physics |
| The tipover cutoff, teleop watchdog, IMU outage handling, gyro-rate wiring and feedforward age limit behave as designed | Tier 3, **the model** | Checks H4, C3, C2, H6, FF of `verify_sim.sh`. C3 was **flaky** in the container (2 of 3 default runs red in 5c-2); its cause was found and C3 fixed on 2026-11-07, 20 of 20 green since ([journal](2026-11-07-c3-v-verifier-noise.md)). Whether any of these work on the robot: not checked |
| The real cascade with the hardware gains holds the model up | -- | **False in the model**: it does not (finding H-12, [stage 5](2026-11-03-stage5-balance-in-sim.md)), with either encoder sign |
| The cascade with the **sim-tuned gains** holds the model up, recovers from a push and follows a step (U, I, V) | Tier 3, **an estimated model, `encoder_sign:=-1` assumed** | [5c-1](2026-11-03-stage5c1-sim-gain-tuning.md). V's overshoot 23.5 to 29.1 % against a 30 % limit with the metric of the time; much of that spread was the verifier's noise (a late control cycle read as a faster wheel), and with the corrected metric (2026-11-07, limit unchanged) nominal V passed 20 of 20, 15.9 to 24.5 % ([journal](2026-11-07-c3-v-verifier-noise.md)) |
| The same, under ±30 % of mass, centre-of-mass height, motor gain and IMU delay | Tier 3, estimated model | [Sweep re-run with the corrected V metric](2026-11-07-stage5c2-sweep-rerun.md): **PASS 18, MARGINAL 4, FAIL 2, COULD NOT JUDGE 1** of 25 cases; no fall; U and I pass in all 25; one corner (`m+h+k−d+`) fails V in all 3 runs (45 to 47 %), one `m+` run failed on a gamepad-watchdog lurch caused by the harness, and `d+` could not be judged in 2 runs (the container's clock stepped back under the samples; [review fixes](2026-11-07-stage5c-review-fixes.md#3-why-v-could-not-judge-two-d-runs-the-containers-system-clock-stepped-back)). Historical: the first sweep, with the old metric, gave PASS 14, MARGINAL 4, FAIL 7 ([5c-2](2026-11-03-stage5c2-sweep.md)). See the [gap page](../architecture/sim-to-real-gap.md) for what the sweep did not cover |
| Anything about the robot: balancing, a gain being better, the signs, the safeguards | **not reached (Tier 4 is a human)** | Procedures: [0006](../proposals/0006-velocity-loop-sign-check.md), [0008](../proposals/0008-candidate-gains-and-tethered-test.md) |

## (b) What failed, and what was done about it

| What failed | Where | What was done |
| --- | --- | --- |
| A failed IMU read reported pitch 0.0 (C-2) | stage 2 | Fixed in the node and the plugin (hold the last value, count failures, `ERROR` past a limit); check C2 in the sim; a check that goes red if the failed read exports 0.0 |
| Review round 3 of stage 4 came back **dirty** (H-A, M-A, M-B, plus docs findings) | [stage 4 journal](2026-10-31-sim-plant-and-critical-fixes.md) | The run stopped at the cap (CLAUDE.md §2 step 6). The person authorised a round 4; findings H-11 (feedforward age limit), M-7 (gyro read gated) and M-8 (log lines) were fixed in code. **Round 4's fixes were not re-reviewed** by the journal's account; confirm before relying on them |
| With the hardware gains the model drifts off and falls (found 2026-11-02) | [stage 5](2026-11-03-stage5-balance-in-sim.md) | Diagnosed as finding H-12: the outer loop is positive feedback unless the encoder reads negative when a command drives the robot. Not a code fix: [proposal 0006](../proposals/0006-velocity-loop-sign-check.md) |
| Scenarios U, I, V failed with the hardware gains (the first scenarios run) | stage 5 | Sim gains tuned in 6 iterations ([5c-1](2026-11-03-stage5c1-sim-gain-tuning.md)); thresholds, committed before tuning, were not changed |
| Checks C3 and FF were flaky | [C3/FF journal](2026-11-03-c3-ff-flaky-checks.md) | Fixed to judge send times, not when the checker ran. **C3 then flaked again** in 5c-2 (flag set 0.15 s after a command); not fixed in this report's runs, **fixed on 2026-11-07** (a blocked `publish()` or a stalled node, neither a watchdog fault; [journal](2026-11-07-c3-v-verifier-noise.md)) |
| Scenario V's overshoot verdict was noisy (5c-1, 5c-2) | [V/C3 journal](2026-11-07-c3-v-verifier-noise.md) | 2026-11-07: the metric changed (velocity recomputed with each cycle's real length), the 30 % limit did not; the sweep was re-run with it ([re-run](2026-11-07-stage5c2-sweep-rerun.md)) |
| The sweep's override mechanism refused `motor_stall_torque_Nm` (regex allowed lower case only) | 5c-2 | Found by a deliberate break before the sweep ran; fixed |
| The drift check let every gain differ unchecked | 5c-1 | `check_sim_config.py` now allows only the named tuned gains to differ and pins their hardware values; 11 deliberate breaks shown red. Since 2026-11-07 a tuned gain must also be on every joint in both files ([review fixes](2026-11-07-stage5c-review-fixes.md)) |

## (c) Findings declined, and why

| Finding | Why declined |
| --- | --- |
| A host gtest for `AddFeedforwardController`'s NaN and stale-reference logic (stage 2 round 2 and 3) | It is exercised end to end by the sim checks once the fault scenarios exist; recorded as not covered by a unit test in the stage 4 journal, section "Not verified" |
| Changing V's overshoot threshold or metric, though V's verdict is noisy (5c-1, 5c-2) | A threshold change after seeing results is a separate human decision and must precede further tuning; it was not made. Reported as narrow and noisy instead. (Later, 2026-11-07, the **metric** was changed once the noise was shown to be a verifier artefact; the threshold still was not: (b)) |
| Retuning after the sweep's one systematic FAIL corner (`m+h+k−d+`) | The task was a sweep, not retuning; reported only |
| A stub motor plugin for off-Pi builds (stage 1) | The error text is documented instead; the wheels cannot be driven by it |
| Marking Tier 3 built | A call for the orchestrator; made on 2026-11-07: the ladder now says built, with the limits in the box at the top |
| The stage 5 reviewer agents (`explainability-reviewer`, `docs-reviewer`, `ros2-control-reviewer`) | Not run in 5b, 5c-1 or 5c-2: those task lists did not include them. **Waiting.** This is not a declined finding but an unrun review, listed so it is not mistaken for a clean one. (Later: findings of a `ros2-control-reviewer` and an `explainability-reviewer` pass over stage 5c were fixed on 2026-11-07, [review fixes](2026-11-07-stage5c-review-fixes.md); those fixes were not re-reviewed in that run) |

## (d) Proposals filed

| # | What | Status |
| --- | --- | --- |
| [0003](../proposals/0003-finish-tipover-refactor-and-guard-new-keys.md) | Finish the tipover refactor (a hook-refused edit); guard the new safety keys | Open |
| [0004](../proposals/0004-safety-thresholds.md) | Confirm the proposed safety thresholds (checked complete against the code on 2026-11-04) | Open |
| [0005](../proposals/0005-amend-adr-0003-simulpant-first.md) | Amend ADR 0003: `SimPlant` first, Gazebo later | Open |
| [0006](../proposals/0006-velocity-loop-sign-check.md) | Check the robot's velocity-loop sign on a stand, then flip one sign if wrong | Open |
| [0007](../proposals/0007-measure-robot-parameters.md) | Measure the robot's physical parameters | Open |
| [0008](../proposals/0008-candidate-gains-and-tethered-test.md) | Candidate gains and a tethered test | Open |
| [0009](../proposals/0009-controller-manager-minimum-version.md) | The `controller_manager` minimum version | Open |

## (e) What was not verified

- **Anything on a robot**: balancing, any gain, the encoder and pitch signs, the gyro axis, sign and
  unit, the tipover cutoff, the teleop watchdog, what the motors do after an `ERROR` (finding H-7),
  the real write path's behavior. All of it is Tier 3 at best.
- **The model's parameters**: every one is an estimate ([sim-plant.md §4.1](../architecture/sim-plant.md)).
  Wheel radius has four disagreeing values (issue #16) and none was picked.
- **What the model omits** ([gap page](../architecture/sim-to-real-gap.md)): backlash, PWM aliasing,
  battery sag, stiction, gyro noise and rate delay, slip, quantisation, CPU jitter, DDS delays.
- **The sweep's gaps**: wheel radius, friction, encoder sign, inertia separately from mass, IMU
  bias, initial pitch; the `tune-gains` held-out set.
- **The motor plugin at run time** (not compiled in the container); **C-1** (the blocking service
  calls) is unfixed; only an unwired framing helper, its host test and proposal 0010 landed (2026-11-07).
- **Container timing versus the Pi**: nothing was measured on a Pi.
- **Reviews**: stage 4 round 4's fixes and everything in stage 5 are not reviewed by the agents
  (stage 5c was reviewed later, in part: see (c)).
- **The `COULD NOT JUDGE` path** of the scenarios mode, and the pass branches of I and V from a
  deliberately broken input (5b), per the stage 5 journals.
- **The default `verify_sim.sh` was not reliably green** because of C3 at the time of writing (C3
  fixed 2026-11-07, see (b)).

## (f) The tethered robot procedure

The full procedure is in [proposal 0008](../proposals/0008-candidate-gains-and-tethered-test.md); the
order, in a paragraph:

1. **First, [0006](../proposals/0006-velocity-loop-sign-check.md):** on a stand, find the two signs
   (does the encoder read positive or negative when a command drives the robot; does pitch go up or
   down when tilted that way). Without this the sim results do not apply.
2. **Measure the robot** ([0007](../proposals/0007-measure-robot-parameters.md),
   [measure-inertia.md](../guides/measure-inertia.md)); the agent reruns the scenarios and the sweep.
3. **Confirm the safety thresholds** ([0004](../proposals/0004-safety-thresholds.md)), and verify the
   gyro axis, sign and unit (0008, "Gyro check").
4. **Wheels off the ground**: launch `ros2 launch rocko_env rocko.launch.py`, tilt by hand at most 3
   degrees, check the wheels turn the way that rolls under the lean.
5. **Tether**, a person holding, power cut within reach, no gamepad: release gently, 5 seconds first.
   Abort on the cutoff (35 deg), a human limit of 15 deg (proposed), full-speed wheels, a growing
   oscillation, creep, buzz or heat.
6. **Log** the [reproducibility block](../guides/reproducing-results.md) and a bag; **do not
   conclude** that a gain is better from one run.

## (g) What the simulation omits, and what each omission does to a gain

[The gap page](../architecture/sim-to-real-gap.md) has each one (backlash, PWM aliasing, battery sag,
IMU latency beyond the whole-ms model, gyro noise, motor dead band and stiction, wheel slip, friction
and braking, encoder quantisation, CPU jitter, DDS delays, a wrong upright reference), with the loop it
hits and the direction, as **reasoning, not measurement**. In short: nearly every omission makes the
sim optimistic; the two that matter most for the candidates are **gyro noise** (pitch d of 1.0 is ten
times the file's and the sim's rate is exact and undelayed) and the **encoder sign** assumption.

## What a human does next

1. Read the box at the top, then [next-steps](../next-steps.md).
2. Do proposal 0006's two-step check; it takes minutes on a stand and gates everything else.
3. Decide 0003, 0004, 0005 and 0009; do 0007; only then consider 0008.
4. Run (or ask for) the unrun reviews listed in (c) and (e).

## Verification of this report

Docs only; no code or config changed in this commit. Checked: a relative-link scan over `docs/`,
`README.md` and `CLAUDE.md` (the command and result are in the commit message that adds this file);
the facts are copied from the journals cited, not re-run. Not run: any tier.

**Why the run stopped here:** the documentation deliverables of stages 4 to 6 are written; stage 6
and the human steps above are what remain.
