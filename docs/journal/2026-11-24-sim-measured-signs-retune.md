# The simulation runs the robot's measured signs, and its outer loop is retuned for the true wheel speed

- **Date / commit:** 2026-11-24, built on `f9a612b` (the owner's "update gains": the hardware velocity gains are now
  negative). The commit that adds this file holds the record.
- **Branch:** ai-forward
- **Environment:** the dev container (Tier 3 is a model with ESTIMATED parameters), laptop for the config check and the
  linear model. Nothing here touches the robot.

## Goal and success criteria

The robot's signs are measured: pitch goes **down** when it leans toward the direction a positive command drives it
(2026-11-23), and the encoder reads **positive** under a positive command (2026-11-24, [stand check](2026-11-24-stand-sign-check.md)).
The owner chose [proposal 0006](../proposals/0006-velocity-loop-sign-check.md) option A (negative velocity gains in the
hardware YAML). The sim had assumed `encoder_sign:=-1` with positive gains, and since the U-2 fix
([journal](2026-11-23-encoder-service-u2.md)) V and AB failed. Done when:

1. The sim's default `encoder_sign` is the robot's and no scenario still overrides it for the old assumption.
2. The sim's velocity gains are negative (option A), with the reason written next to them.
3. AB still characterises the as-built (unflipped) loop and still expects a fall; S2b still fails as it must.
4. U, I, V and K pass with their thresholds **unchanged**, at the true (not halved) wheel speed.
5. `repeat_check.sh "scenarios V" 10` and `scenarios 3` are reported as they came out.

## Plan

Change only the sign plumbing and one gain, in this order: (a) verify_sim.sh / verify_tools.sh, (b) the sim YAML's sign,
run the scenarios to get the new "before", (c) test the first hypothesis, halving the outer `p` (the loop gain doubled
when the encoder replica stopped halving the speed), (d) repeat checks, ladder, docs. Risk: the sign change alone should
not change U, I, S2b (it is the same loop, see below), so those are a built-in cross-check.

## The sign mapping, and how it was confirmed

- The sim's `encoder_sign = +1` means "exported position and velocity = +1 x motion toward sim-forward"
  (`SimPlantModel.hpp`: `left_velocity = encoder_sign * r * (phi_dot - psi_dot)`), and sim-forward is the direction a
  positive wheel command drives ([sim-plant.md §2.4](../architecture/sim-plant.md)). The robot's encoder is positive
  under a positive command, so the robot is `encoder_sign = +1`. `pitch_sign = -1` (a positive command makes the
  exported pitch go up, because it tips the body back) is the measured pitch sign too.
- The default was already +1 in all three places `sim/check_sim_config.py` keeps equal (xacro `default="1"`,
  `rocko_sim.launch.py` `"1"`, `SimPlantParams` `+1.0`), which that script checked (`ok ... 13 launch-file defaults
  match the xacro`). Only `verify_sim.sh` (`SCENARIO_ENCODER_SIGN=-1`) and `verify_tools.sh` (the dashboard launch) overrode it.
- **Cross-check that negating the gains is the old loop in new clothes.** With `encoder_sign = -1` and positive gains,
  the velocity loop's feedback is `+K v`; with `+1` and negated gains it is also `+K v` (only the sign of the velocity
  reference differs). The first run on the new signs reproduced the 2026-11-23 journal's numbers for the scenarios that
  use a zero reference: U (late `|pitch|` under 0.1 deg), S2b (35 deg at 0.140 s vs 0.142 s), AB (35 deg at 2.470 s vs
  2.468 to 2.472 s in its 3 repeats). That is the evidence that the mapping holds in the nonlinear sim as well as on paper.

## What changed

| File | Change |
| --- | --- |
| `verify_sim.sh` | `SCENARIO_ENCODER_SIGN=1` (measured, was -1 "ASSUMPTION"); new `AB_ENCODER_SIGN=-1`; AB launches with it; scenario text, banner and result header say "measured". Thresholds untouched. Kept: the explicit `encoder_sign:=` argument on U/I/V/K/S2b (one named place, and `parameter_sweep.py` reads the variable) |
| `verify_tools.sh` | dashboard launch no longer overrides `encoder_sign` (it would have run the unstable loop against the negated gains) |
| `rocko_env/rocko_env/bringup/config/rocko_sim_controllers.yaml` | outer loop gains `p 25 -> -12.5`, `d 0.2 -> -0.2`, `i` stays 0; comments say why. Pitch loop untouched |
| `sim/check_sim_config.py` | SIM_TUNED_GAINS pins follow the owner's hardware gains (`p -2.0`, `i -5.0`); the sim's `d -0.2` equals the hardware's, so `d` left the list |
| `sim/cascade_linearisation.py` | scenario encoder sign +1; the gains regex accepts the multi-line gains the hardware YAML now uses; docstring says `--sweep` and numeric `--candidate` still assume positive velocity gains (not updated) |
| docs | `sim-plant.md` §2.4 and the parameter table, `verification-ladder.md` Tier 3 row and scenario paragraph, `sim-to-real-gap.md`, `next-steps.md` rows 2.3 and 2.7, the `build-verify` and `tune-gains` skills, `verify-in-container.md` |

AB: the launch file has no gain-file argument, so AB cannot load positive gains. At a zero velocity reference negating
the gains is the same as negating the encoder, so AB runs `encoder_sign:=-1` against the negative gains, which is the
positive-gain (as-built) loop with the measured sign. Its comment in `verify_sim.sh` says so. The AB threshold
`AB_MIN_FALL_S = 3.0` is unchanged.

## Tuning record (Tier 3, the model)

Scenario numbers, one run each unless stated. "Before" is the first run with the signs measured and the old magnitude.
Conditions: sim gains as listed, `encoder_sign:=1` (AB: -1 against negative gains), true wheel speed (U-2), 20 ms IMU
delay, estimated parameters.

| Scenario | 2026-11-23 (assumed sign, p +25) | Before: measured sign, p -25 | After: p -12.5 |
| --- | --- | --- | --- |
| U | PASS, late `\|pitch\|` <= 0.098 deg | PASS, <= 0.078 deg, `\|v\|` <= 0.0025 m/s | PASS, <= 0.048 deg, `\|v\|` <= 0.0019 m/s |
| I | PASS, peak 11.985 deg, back under 2 deg in 0.571 s | PASS, peak 11.705 deg, 0.570 s | PASS, peak 4.490 deg, 0.910 s |
| V | FAIL, overshoot 47 to 49 % (limit 30 %) | **FAIL**, overshoot 34.1 % | PASS, 90 % in 1.777 s, overshoot -5.2 % (peak below the step), peak pitch 1.281 deg |
| K | PASS, 90 % in 1.488 s, 17.2 % | PASS, 1.547 s, 7.3 % | PASS, 2.016 s, 1.5 % (limit on rise: 5.0 s) |
| S2b | ok, 35 deg at 0.142 s | ok, 0.140 s | ok, 0.150 s |
| AB | FAIL, 35 deg at 2.468 to 2.472 s (floor 3.0 s) | **FAIL**, 2.470 s | ok, 35 deg at 4.111 s |

Gain changes (the tune-gains skill: inner loop first, one change at a time, reasons written down):

1. **Sign of the outer loop's p and d, +25 -> -25 and +0.2 -> -0.2** (option A; `i` was 0). Reason: the measured
   signs. Effect: same loop as the assumed sign (cross-check above), V and AB still red, so nothing else had changed.
2. **Outer `p`, -25 -> -12.5** (the first hypothesis, nothing else moved). Reason: the U-2 fix doubled the speed the
   outer loop sees, so its effective gain doubled; halving `p` restores the loop gain the thresholds were met with in
   stage 5c-1. Effect: V and AB turn green and K, U, I, S2b stay green. The inner loop was not touched (its U, I and
   S2b results show it is still fine), and `d` did not need to change, so no second hypothesis was tried.

I and K moved too: I's peak fell from 11.7 to 4.5 deg because a weaker outer loop leans less hard against the push, and
it takes 0.34 s longer to settle (0.910 s against a 3.0 s limit). K's rise slowed from 1.55 s to 2.02 s. V's overshoot
is negative: with `i = 0` the speed settles a little under the reference (a P-only loop), so the "overshoot" is the peak
speed compared with the step, which stays under it. That makes V's overshoot limit easy to meet; it is not a sign
the limit was loosened (it was not).

## Stability evidence (Tier 3, the model)

Final gains, `38cde08` checkout, `tools/repeat_check.sh`:

| Command | Result |
| --- | --- |
| `tools/repeat_check.sh "scenarios V" 10` | pass 10, fail 0, could not run 0. V: 90 % in 1.553 to 1.918 s, overshoot -5.5 to +2.5 %, peak pitch 1.28 to 1.34 deg |
| `tools/repeat_check.sh scenarios 3` | pass 3, fail 0, could not run 0. Per run: U late `\|pitch\|` 0.045 to 0.065 deg; I peak 4.4 to 4.6 deg, 0.89 to 0.90 s; K 2.02 to 2.04 s; S2b 0.139 to 0.151 s; AB falls at 4.110 to 4.121 s |

(An earlier attempt of both commands printed "fail" for every run because `LOG_DIR` did not exist, so no run was made;
it is not a tally of anything and was repeated with the directory created.)

## Verification

| Tier | Command | Result |
| --- | --- | --- |
| 0 host | `python3 -I sim/check_sim_config.py` | ok (defaults equal in the three places; the sim differs from the hardware gains in pitch p, pitch d, velocity p and i only) |
| 3 | `verify_sim.sh scenarios` and the repeats above | U, I, V, K PASS; S2b and AB ok |
| laptop | `python3 -I sim/cascade_linearisation.py --candidate yaml` | the linear model, with `encoder_sign +1` and these gains: largest per-cycle `\|z\|` 0.99334 (stable), gain margin x3.2 up, delay margin 49 ms over 20 ms; time sim U, I, V pass. It agrees with the nonlinear sim in sign and rough size, not in V's rise time (2.76 s there, 1.78 s in the sim) |
| ladder | `tools/run_ladder.sh` (once, final gains) | **FAIL**: 3 red steps, none a scenario; see "Ladder result" below (resolved in "Follow-ups") |
| ladder | `tools/run_ladder.sh`, re-run after the follow-ups (S1 at 0.2 deg, frozen hook-test YAML, the owner's one-line gains) | **PASS on every step**: Tiers 0 to 3, all host tests, hook tests, drift check, link scan; the dashboard check passed too |

## Review rounds

Not run (no C++ or real-time file changed; scripts, a sim YAML and docs only).

## Not verified, needs a human

- **Nothing here says the robot balances.** The model's parameters are still estimates (mass, centre-of-mass height,
  motor torque, 20 ms IMU delay: row 2.2, proposal 0007). The sim's gains are candidates, not results.
- The ±30 % sweep (`sim/parameter_sweep.py`) was **not** re-run with these gains; its results describe the assumed sign
  and the halved speed signal. `cascade_linearisation.py --sweep` and numeric `--candidate` still assume positive
  velocity gains and were not updated.
- The sim's `p -12.5` differs from the hardware's `-2.0` by a factor 6; whether any sim gain suits the robot is the question of
  [proposal 0008](../proposals/0008-candidate-gains-and-tethered-test.md), which needs a tethered test.
- The true speed the real robot reports at 100 Hz (balancing) has not been checked against a stopwatch (see the
  [stand check](2026-11-24-stand-sign-check.md)).

## Ladder result (`tools/run_ladder.sh`, once, final gains, `f9a612b` plus these edits, 13 dirty files) (resolved below)

PASS: container mount, Tier 1 build (0 compiler warnings), Tier 0 `verify_urdf.sh`, Tier 2 `verify_mock.sh` balance and
tank, `verify_supervisor.sh`, compile-only motor plugin, **Tier 3 `verify_sim.sh scenarios` (U I V K S2b AB)**, all 13 host
tests, sim config drift check, doc link scan. FAIL, three steps:

1. **Tier 3 `verify_sim.sh` (S1 S2a H4 C3 C2 H6 FF): S1-command, a real finding, not fixed.** S1 adds a 5 rad/s feedforward
   to both wheels and requires pitch to move at least `S1_CHANGE_DEG` = 0.5 deg ("it changes because the wheels moved").
   With the cascade holding the model up it moved **0.443 deg** (baseline S1 still 0.0000: the check can still see a
   response, just a smaller one than its threshold). It is deterministic, and it grows only slowly with the outer
   loop's `p`: 0.449 deg at `p -12.5, d -0.1`, 0.475 deg at `p -15` (where V peaks at 1.6 deg and AB falls at 3.57 s, still
   passing everything else). I did not chase the threshold with the gain (reaching 0.5 would mean about `p -20`, closer to
   V's overshoot failure at `p -25`, to satisfy a wiring check) and did **not** change `S1_CHANGE_DEG`: the instruction
   was to keep U/I/V thresholds, and loosening another one is a decision for the orchestrator or the owner. The comment
   above it ("far below what the command produces: several degrees") was true for the stage 3 cascade and is not for
   this one. Options: lower `S1_CHANGE_DEG` to about 0.3 deg (still 30x the still threshold) with a written reason, or
   raise the feedforward. S2a, H4, C3, C2, H6 and FF passed.
2. **Tier 3 `verify_tools.sh` (dashboard): "only 38 history points overlap the 6.0 s this script listened" (needs 40).**
   A count of history points in a listening window, not a pitch comparison (the check also reported nothing else wrong).
   Re-run alone three times on the same files: 99, 117 and 112 points, all ok, plus the record and Foxglove parts. So 1
   red in 4 runs, cause not found, and it reads as the container's known stalls (bootstrap-sim, "Designing a check that
   does not fail at random"); it is **not** shown to be independent of this change. The dashboard launch lost its
   `encoder_sign:=-1` override in this change, which is why it is on this list at all.
3. **Hook tests: 81 cases, the gain-guard cases fail with "TEST SETUP ERROR: 'p: 10.0, i: 0.0' is not in
   .../rocko_controllers.yaml (changed? update the test)".** The owner's `f9a612b` ("update gains") reformatted the hardware
   YAML's gains onto several lines, so the test's search strings no longer match. Not caused by this change and not
   touched here (`.claude/hooks/` is a guardrail; a human updates its test). Note that those cases not running means the
   gain guard is untested until then.

Also found while doing this: sim YAML gains must be written as floats (`-15.0`, not `-15`): an integer made
`velocity_pid_controller` fail to initialise (every scenario "could not judge: the wiring was not right"), the scenarios
reported it correctly.

## Follow-ups by the orchestrator, 2026-11-24

- **S1's threshold lowered from 0.5 to 0.2 deg** (`verify_sim.sh`, `S1_CHANGE_DEG`, reason in its comment). S1 proves
  that pitch responds to the wheels; the baseline reads exactly 0. With the measured signs and the retuned outer loop
  the cascade resists the feedforward kick, so the command moved pitch 0.443 deg instead of several degrees. Made to
  fail first: with `disable_torque:=true` the command run moved pitch 0.0000 deg and S1 went red; restored, green.
- **Hook tests:** the owner's reformat of `rocko_controllers.yaml` broke 20 cases, which edit exact text in the file.
  `.claude/hooks/test-guard-protected-files.sh` now takes that file frozen at `a8e3c38` instead of the working tree
  (owner approved the guardrail edit). 81 cases pass. The hook itself is unchanged.
