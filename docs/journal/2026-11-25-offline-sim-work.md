# Work without the robot: the S2a failure explained, the floor test reproduced in a model, proposal 0008 brought up to date

- **Date / commit:** 2026-11-25, built on `49edea5` (this entry is added by the commit that makes the change)
- **Branch:** `ai-forward`
- **Environment:** laptop and the Docker dev container only. The robot was switched off and away; nothing here ran on
  it.

## Goal and success criteria

The owner, leaving for the day: "what can be done that will improve the robot, codebase, documentation that does not
need the robot". The owner then chose items 1 to 4 of the agent's list:

1. **S2a.** The full ladder's one red check (`S2a-no-torque`, 0.622 s against a 0.48 s limit) has a found cause, and
   the check either is fixed or keeps failing for a reason that is written down. A red check stays red until then.
2. **The full ladder** passes on the final code of this change, or each red step is reported.
3. **The floor test in a model.** The owner's 2026-11-25 floor test (the wheels spun one way whatever the lean) is
   reproduced with the hardware gains, and proposal 0017's ±5 deg clamp is tried against it. Done when there is a
   reproducible command that shows both, and a Tier 3 run shows whether the clamp breaks a simulation scenario.
4. **Proposal 0008.** Its candidate gains, evidence and preconditions describe today's state: the measured signs, the
   gyro D term, the C++ encoder filter, the ±30 % sweep rerun with today's candidates.

## Plan

Rerun S2a 10 times and look at the failing run's own files before touching anything. Fix the offline linear model's
encoder (it still copied the removed `QuadEncoders.py`) before using it for anything. Model the floor test offline,
because `SimPlant` cannot hold the body at a lean. Try the clamp in the nonlinear sim by editing the sim YAML for
one run and reverting it (the sim YAML is free to edit, CLAUDE.md §3; the hardware YAML is not touched). Run the
sweep (about an hour) while writing the docs, then the full ladder once on the final code. Risk: the sweep and the
ladder share the one container, so they run one after the other.

## 1. S2a: a container stall, not the robot

| Command | Result |
| --- | --- |
| `tools/repeat_check.sh S2a 10` | pass 10, fail 0. Fall times 0.429 to 0.433 s (the model's own fall time is 0.4273 s, unit test #6) |
| the failing run's `samples.csv` and `launch.log`, still in the container (`/tmp/tmp.FfWZFeuoO2`) | one 0.222 s gap between two pitch samples (28.6 deg, then 35.9 deg), and `controller_manager`: `Overrun detected! ... The loop took 193.181209 ms`, then `SimPlant: a 199.0 ms control period was cut to 50 ms of physics` |

Cause: a 193 ms stall in the container (the full ladder ran while the agent was building in the same container). The
fall is timed in wall-clock stamps, but `SimPlant::read()` integrates at most 50 ms of physics per cycle and drops the
rest, so 149 ms of fall went missing and the fall read 0.19 s too long. Nothing in the code under test was wrong.
(An earlier `repeat_check.sh` attempt printed "fail" 10 times because its `LOG_DIR` did not exist, so no run was made;
it was repeated with the directory created. The 2026-11-24 retune journal records the same mistake.)

Fix, `verify_sim.sh` `run()`: when the probe's timed check (mode `exceed`, i.e. S2a) fails and the launch log has
`SimPlant`'s "control period was cut to" line, the run is COULD NOT JUDGE (exit 2) with the two log lines printed,
not a failure. Exit 2 is not a pass: `repeat_check.sh` counts it as "could not run".

| Made to fail first | Result |
| --- | --- |
| the detector on the failing run's `launch.log`, and on two passing runs' | matches 1, 0, 0 |
| `FALL_LIMIT_S` set to 0.40 (a real failure, no stall), `verify_sim.sh S2a` | `FAIL (S2a-no-torque)` at 0.431 s, exit 1: a real failure is not hidden. Restored to 0.48 |

## 3. The floor test in a model, and the clamp

**The offline model first.** `sim/cascade_linearisation.py` still modelled the encoder as `QuadEncoders.py` did (two
filter updates per cycle, a steady gain of 0.5). Since 2026-11-24 the motor plugin filters once per cycle over the
real period (`WheelEncoder.hpp`), and `SimPlant` already copies that. The model now does the same (steady gain 1,
time constant 195 ms). With the sim YAML's gains: largest |z| 0.97748 per cycle, gain margin x3.203 up and x0.123
down, stable for an IMU delay from 0 to 70 ms, time sim of U, I, V passes (the 2026-11-24 journal had 0.99334 and a
49 ms margin with the old encoder).

**`--floor-test`, new** (section 7 of the same file): the hardware gains, the body held still at a lean while the
cascade runs, on the floor (the wheels roll the whole robot) and on a stand. A person holding the body removes the body
dynamics, which is what makes the velocity loop negative feedback on a free robot, so the wheels simply obey the
command. The clamps are modelled as `pid_controller` describes its parameters (legacy anti-windup: the integral term
clamped, then the output), not from its source. The result (`python3 -I sim/cascade_linearisation.py --floor-test`):

- **Held at one lean, the wheels follow its sign**, with or without the clamp. So the wind-up alone does not make
  "one direction whatever the lean".
- **Once the wheels have turned, a later lean cannot reverse them** with the YAML as it is: the integral winds the
  target lean up to about ±160 deg in the direction they already turn. Enabled level, leaned back 10 deg, then forward
  10 deg: with the old offset (level read +10.1) -5.2 m/s after all three holds; with the new offset +5.2 m/s after
  leaning back and still +5.2 m/s after leaning forward.
- **With both clamps at ±5 deg the wheels reverse with the lean** (new offset: +5.2, then -5.2 m/s; the target stays
  at ±5), and under a 5 deg lean the speed is bounded at about 2.5 to 3.1 m/s instead of 5.2.

That fits the owner's report, with the old offset starting the wheels at enable. It is a model of a held robot with
estimated parameters: evidence for proposal 0017's reasoning, not a measurement of the robot.

**The clamp in the nonlinear sim (Tier 3).** Both clamps at ±5 put in `rocko_sim_controllers.yaml` (the installed
copy is a symlink to the source, checked), `tools/repeat_check.sh scenarios 1`, then the file restored from a copy
(`git diff` empty for the config folder):

| Scenario | With the clamp | Without (2026-11-24, 3 runs) |
| --- | --- | --- |
| U | PASS, late \|pitch\| <= 0.216 deg, \|v\| <= 0.0035 m/s | 0.045 to 0.065 deg |
| I | PASS, peak 4.534 deg, settled 0.898 s | 4.4 to 4.6 deg, 0.89 to 0.90 s |
| V | PASS, 90 % in 1.639 s, overshoot -4.9 %, peak pitch 1.299 deg | 1.55 to 1.92 s |
| K | PASS, 2.042 s, overshoot 1.5 % | 2.02 to 2.04 s |
| S2b | ok, 0.141 s | 0.139 to 0.151 s |
| AB | ok, falls at 6.869 s | 4.110 to 4.121 s |

The clamp acted (AB, the as-built loop that must fall, falls later because its run-away target lean is capped; U's
late pitch is larger), and nothing failed. One run only.

## 4. Proposal 0008

Rewritten in place: the candidates are now exactly the sim YAML's gains (pitch p 15, d 1.0 on the gyro; velocity
p -12.5, i 0, d -0.2), the preconditions say which are done (signs, gyro, the D-term wiring) and which are open (0007's
measurements, 0004's thresholds, 0017 and the balance-point pitch), and the evidence table has the 2026-11-24
scenarios, the clamp run above, the sweep below and the linear margins with the C++ encoder. Two step texts that named
removed things (the IMU service, the launch-file gyro edit) are marked superseded.

**The ±30 % sweep** with these gains (`sim/parameter_sweep.py run --stem sim/results/2026-11-25-measured-signs-sweep`, about 75 minutes, in the container; results page [sim/results/2026-11-25-measured-signs-sweep.md](../../sim/results/2026-11-25-measured-signs-sweep.md)): 25 cases, 125 runs: PASS 24, FAIL 1, no fall anywhere; U and I pass in all 25. The one FAIL is one V run of the corner `m-h-k-d+` (light, short, weak motor, 26 ms): 90 % reached at 3.107 s against a 3.0 s limit; its other runs took 2.08 to 2.09 s, and 5 reruns 1.96 to 2.21 s (7 of 8 pass). That run had a 137 ms container stall, which explains about 0.09 s, not the 1 s; cause not found, kept red. The old corner failure (`m+h+k-d+`, V overshoot about 46 % with the 2026-11-04 gains) now passes, overshoot at most -5.7 %. The runs were made with the docs and `sim/cascade_linearisation.py` edited but not committed (the page says `49edea5+dirty`); nothing the sim loads was changed (the clamp experiment had been reverted before). Evidence for the red: `SIM_PLANT_OVERRIDES="body_mass_kg:=1.4 com_height_m:=0.14 motor_stall_torque_Nm:=0.5423075 imu_delay_ms:=26" tools/repeat_check.sh "scenarios V" 5`: pass 5 of 5.

`sim/parameter_sweep.py` wrote `encoder_sign:=-1 ASSUMED` into the page's header and launch lines, copied by hand from before 2026-11-24; the runs themselves use `verify_sim.sh`'s `SCENARIO_ENCODER_SIGN=1` (the page's threshold line shows it, and negative gains with -1 would run away). The text now says +1, measured, and the page was rewritten with `parameter_sweep.py report`.

## Also fixed (stale text found on the way)

- `sim-plant.md` unit test row 13 still gave the encoder numbers from before the U-2 fix (left 0.5128 v, right
  0.4872 v); the test checks that both read v. The test's own comment named `QuadEncoders.py`.
- `verification-ladder.md`: the Tier 3 row said the hardware YAML still had positive velocity gains (applied
  2026-11-24), and the V-metric paragraph described the U-2 encoder in the present tense.

## Verification

| Tier | Command | Result |
| --- | --- | --- |
| 3 | `tools/repeat_check.sh S2a 10` | pass 10 of 10 (above) |
| 3 | `verify_sim.sh S2a` with `FALL_LIMIT_S=0.40` | FAIL, as it must; restored |
| 3 | `tools/repeat_check.sh scenarios 1` with the clamp in the sim YAML | pass (above); reverted |
| 3 | `sim/parameter_sweep.py run` | PASS 24, FAIL 1 (one V run of `m-h-k-d+`, 3.107 s against 3.0 s; 5 reruns pass); **red kept, cause not found** |
| laptop | `python3 -I sim/cascade_linearisation.py --candidate yaml` and `--floor-test` | as above |
| all | `tools/run_ladder.sh` on the code after round-2 review fixes (only docs and one docstring changed after it) | **PASS on every step**: container mount, Tier 1 build (0 warnings), Tier 0, Tier 2 mock (both modes) and supervisor, compile-only motor plugin, Tier 3 `verify_sim.sh` (S1 S2a H4 C3 C2 H6 FF), `verify_tools.sh`, `scenarios`, all 15 host tests, hook tests, sim config drift check, doc link scan |

## Review rounds

| Round | Reviewer | Findings | Fixed / declined (why) |
| --- | --- | --- | --- |
| 1 | explainability-reviewer | the S2a downgrade matched any stall and any `exceed` failure, and hid the probe's reason; unnamed constants; the clamp reader's assumptions undocumented; a command-sign sentence asserted, not computed; a fragile time lookup; one long report function; also a stale 0.5 encoder gain in section A2 | all fixed: the downgrade needs the probe's late-fall line AND the stall line, and prints both; section 7 rewritten (output unchanged); A2's `k` fixed |
| 1 | docs-reviewer | the encoder sign still called an assumption (ladder, gap page); 0017's stand-test guess overstated; the model claim stated too firmly (0017, next-steps); the clamp run's commit wording; 0008 precondition 4 unsourced | all fixed |
| 2 | explainability-reviewer | clean (one long docstring line) | wrapped |
| 2 | docs-reviewer | the old sweep still presented as current (README, a ladder sentence); "assumed sign" left in next-steps row 2.3 and `sim-plant.md` | all fixed |
| 3 | docs-reviewer | three older sentences still said "assumed encoder sign" (README, next-steps' Tier 3 summary) or cited the removed `QuadEncoders.py` for the encoder filter (`sim-plant.md` parameter list); none of them added by this change | all fixed; checked by grep and the link scan, not by a fourth review round (CLAUDE.md allows three) |

## Not verified, needs a human

Everything on the robot. The floor-test model is a held robot with estimated parameters; whether the clamp fixes the
real one-direction spin is proposal 0017's test. The 0008 candidates balance a model, nothing more.
