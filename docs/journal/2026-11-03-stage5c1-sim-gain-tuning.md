# Stage 5c-1: tune the SIM gains until scenarios U, I, V pass, against the thresholds of `0b0e39b`

- **Date / commit:** 2026-11-03, built on `93f7a82`. Follows
  [stage 5](2026-11-03-stage5-balance-in-sim.md), which wrote the scenarios and their thresholds
  (commit `0b0e39b`) before any tuning. Commits of this stage are listed under [Commits](#commits).
- **Branch:** `ai-forward`. Local commits only: this stage was told not to push.
- **Environment:** laptop (macOS, Python 3, standard library) for the linear analysis;
  `rocko-env-dev` container (`ros:jazzy`, arm64 on Docker Desktop) for `verify_sim.sh` and
  `verify_urdf.sh`. `./check_container_mount.sh` printed `ok    the container sees this checkout` at
  the start. This run was the only user of the container.

**Everything in this entry is about a model with estimated, unmeasured parameters
([sim-plant.md §4](../architecture/sim-plant.md)) and an assumed encoder sign.** U, I, V run with
`encoder_sign:=-1`, an **ASSUMPTION** nobody has checked on the robot
([proposal 0006](../proposals/0006-velocity-loop-sign-check.md), finding H-12), and with the modelled
20 ms IMU delay. The strongest claim this entry can make is "**balances in simulation under an
estimated model**". It says nothing about the robot, and no gain here is a recommendation for
`rocko_controllers.yaml`; that needs a candidate-gains proposal (a later task, not written here).

## Goal and success criteria

The person's goal: make scenarios U, I, V of `verify_sim.sh scenarios` pass by changing **sim gains
only**, in `rocko_env/rocko_env/bringup/config/rocko_sim_controllers.yaml`, against the thresholds
committed in `0b0e39b`, unchanged.

1. **The drift check allows exactly the tuned gains to differ.** `sim/check_sim_config.py` lets the
   sim YAML differ from `rocko_controllers.yaml` only in the gains it lists by name, still fails on
   structure, wiring, interface names and `update_rate`, fails on any other gain change in either
   file (including a change to a tuned gain's hardware value), and is shown red on deliberate breaks.
   Its own commit, before any gain changes.
2. **Candidates come from a script, not from guessing.** `sim/cascade_linearisation.py` gains a
   `--sweep` (grid over the five gains, linear stability, then a time simulation of U, I, V with
   SimPlant's write quirks) and a `--candidate` report (gain margins up and down, IMU delay range).
3. **Each change is confirmed in the nonlinear sim**, one gain at a time, inner loop first, at most
   8 iterations per scenario, each logged below as a row (change, metrics before and after, result).
4. **U, I and V pass** (`verify_sim.sh scenarios` exit 0 with S2b and AB "ok"), no threshold
   changed; or, if 8 iterations per scenario run out, a report of which fail and why.
5. **If they pass:** S2b still fails *for the right reason* with the tuned gains (a sign-flipped pitch
   loop falls within 1 s), AB still falls as H-12 predicts, the default `verify_sim.sh` (S1..FF)
   passes, `check_sim_config.py` and the host tests pass.
6. **Margins reported, not just verdicts:** the final gains' linear gain margins and delay margin,
   and each scenario's distance from its thresholds in the nonlinear sim.
7. The hardware YAML is untouched (`git diff 93f7a82 -- rocko_env/rocko_env/bringup/config/rocko_controllers.yaml` is empty).

## Plan

1. Extend `sim/check_sim_config.py` (criterion 1), with docs, in its own commit. Tier 0 (laptop and
   container); deliberate breaks on scratch copies with `--root`.
2. Extend `sim/cascade_linearisation.py` (criterion 2) without changing its existing report (the
   default output must be byte-identical before and after; checked with `diff`). Laptop.
3. Pick a target from the sweep, then walk to it one gain at a time in the nonlinear sim
   (`verify_sim.sh scenarios U I V`), inner loop (pitch) first, recording each run. Tier 3.
4. Commit the gains with this entry's results, separate from step 1. Then run S2b, AB, the default
   `verify_sim.sh`, `verify_urdf.sh` and the host tests.

Risks: the screen (a linear plant with the write quirks) and the nonlinear sim disagree, which costs
iterations; a threshold that cannot be met by any gain (then stop and report it, do not widen it);
a structural problem (journal it as a finding, do not tune around it); gains that pass the tuning
scenarios only because of a quirk of the model (the 30 ms held-out column and the margins are there
for that).

## 1. The drift check: only the tuned gains may differ

Before this stage `check_gains` compared only the gain-map keys that are not numbers
(`angle_wraparound` and similar). Every `p`, `i`, `d` and clamp could differ between the two files
unchecked, in either file: a gain edited in `rocko_controllers.yaml` passed silently, and so did an
accidental clamp change in the sim file.

Now (`SIM_TUNED_GAINS` in `sim/check_sim_config.py`):

- every number under `gains` must be equal in the two files, with its type, **except** the gains
  listed in `SIM_TUNED_GAINS`, as `(controller, key)`;
- each entry pins the **hardware** value it was tuned away from. If `rocko_controllers.yaml` changes
  that gain, the check fails: the sim gains were tuned against a cascade the robot's file no longer
  describes, and a hardware gain change is a human decision anyway;
- in the sim file a tuned gain must be a finite number and equal on both joints (the linear analysis
  treats the two wheels as one loop; two values would be a different controller);
- the wiring check (names, types, `dof_names`, interfaces, `update_rate`) is unchanged.

**Why a list of keys, not "any p, i, d":** each listed gain is one place where the sim tests a
cascade the robot does not run. A short, named list makes that visible in review, and makes an
accidental edit anywhere else red. The list was written from the sweep (section 2) before any gain
was changed: pitch `p` and `d`, velocity `p` and `i`. It does not have to differ (the check cannot
know whether a tuning run is in progress), so a listed gain that ends up equal to the hardware value
should be removed from the list.

**Deliberate breaks** (each on a fresh scratch copy of the tree outside the repo, run with
`python3 -I sim/check_sim_config.py --root <copy>`):

| Break | Result |
| --- | --- |
| Control: sim pitch `d` 0.1 → 0.8 on both joints (a listed gain) | all `ok`, exit 0 |
| Sim velocity `d` 0.2 → 0.3 (not listed) | exit 1: `gains setting differs at /velocity_pid_controller/gains/pitch_pid_controller/left_wheel_joint/d: rocko_controllers.yaml has 0.2, rocko_sim_controllers.yaml has 0.3` (and the right joint) |
| Sim pitch `i_clamp_max` 150 → 100, left joint only | exit 1: `gains setting differs at .../left_wheel_joint/i_clamp_max` |
| Sim pitch `i` 0.0 → 0.5 (not listed) | exit 1, both joints named |
| Sim pitch `d` 0.8 on the left joint only | exit 1: `pitch_pid_controller \`d\` differs between joints (...: 0.8, ...: 0.1); one loop, one gain` |
| Sim velocity `p` written as the string `"20"` | exit 1: `... is '20', not a finite number` |
| Hardware pitch `d` 0.1 → 0.2 (a listed gain) | exit 1: `rocko_controllers.yaml changed pitch_pid_controller \`d\` ...: 0.2, but the sim gains were tuned against 0.1 (SIM_TUNED_GAINS) ...` |
| Hardware velocity `d` 0.2 → 0.4 (not listed) | exit 1: `gains setting differs at .../d: rocko_controllers.yaml has 0.4, rocko_sim_controllers.yaml has 0.2` |
| Sim `update_rate: 100` → `200` | exit 1: `wiring differs at /controller_manager/ros__parameters/update_rate` |
| Sim `pitch_rate` renamed `pitch_rat` | exit 1: the allowed wiring difference no longer matches, and `wiring differs` |
| Sim velocity loop reading `["pitch"]` instead of `["velocity"]` | exit 1: `wiring differs at /velocity_pid_controller/ros__parameters/reference_and_state_interfaces` |
| Sim `angle_wraparound: true` added to one entry | exit 1: `gains setting differs at .../angle_wraparound` |

The real tree, after the change and before any gain change: six `ok` lines, exit 0, the gains line
ending `sim differs from hardware in: none`.
