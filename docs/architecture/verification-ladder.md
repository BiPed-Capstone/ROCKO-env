# The verification ladder

When an AI changes this repo, something has to check the change. This page says what can be checked
today, what each check proves, and — more important — what it cannot. **It is the single place that
records the status of each tier.** Every other page links here instead of restating it.

> Last verified 2026-10-27 on the `ai-forward` branch, in the `rocko-env-dev` container
> (`ros:jazzy`, **arm64** on Docker Desktop for macOS; `pid_controller` 4.42.1). An x86_64 container
> has not been tried. Evidence is in the
> [run journal](../journal/2026-10-27-ai-forward-bootstrap.md). On 2026-10-29, after the container
> was recreated for the [mount check](../journal/2026-10-29-container-mount-check.md), these all
> passed again: the hook tests, a clean `build_ros.sh`, `verify_urdf.sh`, and `verify_mock.sh` in both
> modes. The `pid_controller` version was not read again. On 2026-10-31, on code built on `3882790` plus
> the stage 2 commit, `build_ros.sh`, `verify_urdf.sh`, `verify_mock.sh` and
> `verify_mock.sh tank` passed again (the hook tests were not rerun: no hook file changed), and the new
> [compile-only check](#the-compile-only-check-for-the-motor-plugin-not-a-tier) passed. All of these
> were run again after the fixes from that stage's first review round, and passed, together with the
> new [host unit test](#the-host-unit-test-of-the-imu-fault-tracker-not-a-tier). After the fixes from
> review round 2 (including a new `write()` in `Motor12VoltQuadEncoder.cpp`), `build_ros.sh` (exit 0,
> no compiler warnings), `verify_urdf.sh`, `verify_mock.sh`, `verify_mock.sh tank` and the compile-only
> check (on the edited file) passed, and the host unit test was run by hand and passed. Evidence:
> [stage 2 journal](../journal/2026-10-31-sim-plant-and-critical-fixes.md). Later on 2026-10-31, with
> the stage 3 code (the `SimPlant` plant) (committed as the stage 3 commit), `build_ros.sh`
> (exit 0, no compiler warnings), `verify_urdf.sh` (7 expansions, after the stage 3 review round 2 fixes; the host plant unit test then had 70 checks), `verify_mock.sh`,
> `verify_mock.sh tank`, the compile-only check, the hook tests (81 cases) and the new
> `verify_sim.sh` (exit 0) all passed. Evidence:
> [stage 3 journal](../journal/2026-10-31-sim-plant-and-critical-fixes.md#stage-3--the-plant-simplant-tier-3-partly-built).
> On 2026-11-02, on the stage 4 code (the tipover cutoff, the teleop watchdog, the pitch rate and the
> checks for them; uncommitted when this paragraph was written, on top of `85bdc31`), the integrator
> ran `build_ros.sh` (exit 0, 0 compiler warnings), `verify_urdf.sh` (PASS, now 14 interfaces in
> balancing mode and one allowed wiring difference), `verify_mock.sh` in both modes (PASS), the
> compile-only check (PASS), the hook tests (81 cases), the host tests (listed
> [below](#the-host-tests-of-the-stage-4-safeguards-not-tiers)) and `verify_sim.sh`, which now makes
> 8 launches: PASS. They were all re-run after the code fixes made following the first docs pass
> (a clean build with 0 warnings; every host test passing). On 2026-11-03 (stage 4 review round 4, authorised by
> the person; the code is done and its review is still to run) the feedforward age limit (finding H-11) added check
> `FF` and a host test: `verify_sim.sh` now makes 9 launches and passed all 7 checks (`S1 S2a H4 C3 C2 H6 FF`)
> in 157 s. Evidence:
> [stage 4 journal](../journal/2026-10-31-sim-plant-and-critical-fixes.md#stage-4--safeguards-against-the-sim-h-4-c-3-c-2-downstream-h-6).

## Why a ladder

An agent that writes code and checks it is a feedback loop, and its checks are the sensor. A sensor
that can't report a problem is dangerous in exactly the way finding **C-2** describes: an IMU that
returns pitch = 0.0 when it fails tells the controller the robot is upright at the moment it is
blind. A check that always passes does the same to the agent.

So each tier below has to be able to **fail**, and the tiers are ordered by how much they can see.
A change is only as verified as the highest tier that actually ran.

## The tiers

The exact commands, expected output and failure modes are in the
[`build-verify` skill](../../.claude/skills/build-verify/SKILL.md); a walk-through for a person is
[Verify a change without the robot](../guides/verify-in-container.md).

| Tier | Name | Script | Status |
| --- | --- | --- | --- |
| 0 | Static | `verify_urdf.sh` (container); `.claude/hooks/test-guard-protected-files.sh`, `.githooks/test-git-hooks.sh` (laptop) | **Works.** Needs a Tier 1 build first (`verify_urdf.sh`), which now checks seven expansions and the sim config's drift. The git hooks it tests are not installed in this clone until [proposal 0002](../proposals/0002-unattended-run-settings.md) item E |
| 1 | Build | `build_ros.sh` (container) | **Works** off-Pi, *without* the motor plugin (which only the [compile-only check](#the-compile-only-check-for-the-motor-plugin-not-a-tier) looks at); exits 1 when the build fails. **The Pi build (flag `OFF`) has not been run** since the CMake change |
| 2 | Mock | `verify_mock.sh`, `verify_mock.sh tank` (container) | **Works**; shown to fail on deliberate breaks |
| 3 | Simulation | `verify_sim.sh` (container; needs a Tier 1 build first). The plant is `rocko_env/SimPlant`, described in [sim-plant.md](sim-plant.md) | **Tier 3 is built: a model with ESTIMATED, unmeasured parameters (row 2.2, [proposal 0007](../proposals/0007-measure-robot-parameters.md), not done), `encoder_sign:=-1` ASSUMED ([proposal 0006](../proposals/0006-velocity-loop-sign-check.md), not checked), scenarios U, I and V pass with sim-tuned gains that differ from the hardware gains (a candidate only: [proposal 0008](../proposals/0008-candidate-gains-and-tethered-test.md)), S2b fails as it must; in the ±30 % sweep one corner fails V's overshoot (`m+h+k−d+`, about 46 %), four cases are marginal, one case could not be judged (a clock step), and one real V failure (31.5 %) followed a late control cycle; it says nothing about whether the robot balances.** Marked built on 2026-11-07 by the orchestrator's decision, after the review loop. What exists: two of the checks that define "built" exist and pass: S1 (the real cascade runs against the simulated robot and pitch responds to the wheels) and S2a (with no motor torque the simulated robot falls). Four checks of the stage 4 safeguards also exist and pass: H4 (the tipover cutoff), C3 (the teleop watchdog), C2 (an IMU outage, and what `controller_manager` does after the resulting `ERROR`) and H6 (the pitch PID's D term reads the gyro rate). The upright, impulse and velocity-step scenarios and S2b (the upright scenario with the pitch loop's sign flipped must fail) **exist since stage 5 (2026-11-03)**, in a separate mode, `verify_sim.sh scenarios`, with thresholds committed before any tuning. With the hardware gains the three scenarios fail (first run 2026-11-03: U oscillates at about 2.4 Hz up to 30 deg, I and V fall; [stage 5 journal](../journal/2026-11-03-stage5-balance-in-sim.md)). **Since stage 5c-1 (2026-11-03) U, I and V pass with tuned sim gains** in `rocko_sim_controllers.yaml` (pitch p 15, d 1.0; velocity p 25, i 0), thresholds unchanged, S2b and AB still `ok`; V only just (old V metric: overshoot 23.5 to 29.1 % in 6 runs against a 30 % limit, nominal 15 to 24 % under the corrected metric; the previous gain set failed 1 run in 4), and AB falls at 3.8 s against its 3.0 s floor ([stage 5c-1 journal](../journal/2026-11-03-stage5c1-sim-gain-tuning.md)). **The ±30 % sweep (stage 5c-2, `sim/parameter_sweep.py`)** on body mass, centre-of-mass height, motor gain and IMU delay (25 cases: nominal, one at a time, 16 corners; U and I once, V three times) found no fall: U and I pass in all 25 cases, with the old metric V's overshoot failed in all 3 runs of one corner (heavy, tall, weak motor, 26 ms; 49-51 %, the linear model predicts 44.8 %) and in single runs of 6 other cases, including the nominal one, which look like the verifier's noise ([stage 5c-2 journal](../journal/2026-11-03-stage5c2-sweep.md)). That noise was found and V's overshoot metric changed on 2026-11-07, limit unchanged: nominal V 20 of 20 pass, the corner still fails 3 of 3 ([journal](../journal/2026-11-07-c3-v-verifier-noise.md)). **The sweep re-run with the new metric** (2026-11-07, same 25 cases, `03238ca`; [journal](../journal/2026-11-07-stage5c2-sweep-rerun.md)): PASS 18, MARGINAL 4, FAIL 2, COULD NOT JUDGE 1; still no fall, U and I pass in all 25; V fails in all 3 runs of the same corner (45-47 %), in one run of `m+` (68 %, the gamepad watchdog fired mid-step: a harness delivery stall) and could not be judged in 2 runs of `d+` (a plant sample stamp went backwards; cause found 2026-11-07: the container's system clock, which stamps `controller_manager`'s messages, stepped back about 16 ms while the cycles and the physics run on the steady clock; the refusal is correct; 9 re-runs of `d+` V all judged, 8 pass and 1 fails at 31.5 % after a 46 ms late cycle made the model really surge; [review fixes](../journal/2026-11-07-stage5c-review-fixes.md#3-why-v-could-not-judge-two-d-runs-the-containers-system-clock-stepped-back)). Tier 3 is called built with the limits stated at the start of this row: every physical parameter is an estimate, the encoder sign is an assumption, and the `tune-gains` held-out set (another start, ±20 % mass and friction, one more control period of delay) has not been run in the nonlinear sim (a 30 ms IMU delay was checked on the linear screen only). A fifth scenario, AB, keeps finding H-12 (the outer loop's sign problem under the as-built signs) visible as an expected fall. The plant's parameters are estimates, not measurements, so every result is about the model and says nothing about the robot. [ADR 0003](../decisions/0003-simulation-target.md) is still `Proposed` (it names Gazebo; `SimPlant` was built first) |
| 4 | Robot | A person, tethered | **Human only.** Not something an agent can run or observe |

Before any tier that uses the container, `check_container_mount.sh` (run on the laptop) says whether
the container is looking at your checkout. It is a precondition, not a tier: it proves one thing and
leaves out others (see the blind spots).

### The compile-only check for the motor plugin (not a tier)

`check_motor_plugin_compiles.sh` (repo root, runs in the container, after a Tier 1 build because it
needs the `rocko_interfaces` headers) runs `g++ -std=c++17 -fsyntax-only -Wall -Wextra` on
`Motor12VoltQuadEncoder.cpp` against stub `wiringPi.h` and `softPwm.h` in `tools/compile_check/stubs/`.
Run it from the repo root on the laptop:

```bash
docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/check_motor_plugin_compiles.sh
```

Exit 0 passes, 1 is a compile error (it prints `FAIL (compile-only)`), 2 means it could not run. An
optional first argument names another `.cpp` to check. It is **not a tier**: it sits beside Tier 1,
checks one file Tier 1 leaves out, and never links, never runs and writes no object file. The stubs
hold declarations only and `#error` unless `-DROCKO_COMPILE_CHECK_ONLY` is passed. See what it proves
in the next table.

### The host unit test of the IMU fault tracker (not a tier)

`tools/imu_fault_tracker_test.cpp` tests `ImuFaultTracker.hpp`, the header-only counter that decides
when the IMU plugin returns `ERROR` (finding **C-2**, next-steps row 0.1). It needs only a C++17
compiler and the one header, no ROS, so it runs on the laptop or in the container. From the repo root:

```bash
g++ -std=c++17 -Wall -Wextra -I rocko_env/rocko_env/hardware/sensors/icm20948/include \
    tools/imu_fault_tracker_test.cpp -o /tmp/imu_fault_tracker_test && /tmp/imu_fault_tracker_test
```

Success is the line `all ImuFaultTracker tests passed` and exit status 0; a failed check aborts with
a non-zero status. It is **not a tier**: it checks the counting rules of one class (the consecutive
limit, the 10-of-20 window, a good read clearing the run, the total never resetting) and nothing
about `ICM20948.cpp`, the sensor, timing or a robot. It is not run by any script here, so nothing
makes anyone run it.

### The simulation check, `verify_sim.sh` (Tier 3, built with the limits in the table above)

`verify_sim.sh` (repo root, runs in the container, after a Tier 1 build) launches
`rocko_sim.launch.py` once per check and judges what it saw. Run it from the repo root on the laptop:

```bash
docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_sim.sh
```

With no arguments it runs every check: 7 checks in 9 launches, about 2.5 minutes in total (157 s on 2026-11-03; one
measurement in the arm64 container; not a promise). To run only some, name them (any of `S1 S2a H4 C3
C2 H6 FF`, in any order), for example `verify_sim.sh H4 C3`. An unknown name exits 2.

Exit 0 prints `PASS (simulation: S1 S2a H4 C3 C2 H6 FF)` (the names that ran), then a line for each; 1 is
a failure (it says which run and why); 2 means it could not run (no build, a `controller_manager`
already running, an unknown check name) or `COULD NOT JUDGE (simulation: ...)`: a probe saw no messages or no
movement, so read its reason and the kept run directory. On anything but a pass the run directory is
kept and its path printed. The runs, with the thresholds it holds (each is explained next to its
definition in the script):

| Check | Launch | What it does | Passes when |
| --- | --- | --- | --- |
| S1 baseline | `teleop:=false` | Upright start, no wheel command beyond the cascade's own | Pitch stays within 0.01 deg of its start |
| S1 command | the same | The same, plus a 5 rad/s feedforward on both wheels | Pitch moves at least 0.5 deg: it changes **because** the wheels moved |
| S2a | `disable_torque:=true`, `initial_pitch_deg:=2.0` | With no motor torque the robot must fall | `\|pitch\|` passes 35 deg within 0.48 s of starting to move |
| H4 | `initial_pitch_deg:=40` | Finding H-4. Starts 5 deg past the tipover limit; reads the wheel commands from `controller_manager`'s introspection topic (`/controller_manager/introspection_data/full`, best-effort QoS); after the trip it publishes **one** re-arm message, `ros2 topic pub --once -w 2 /tipover_rearm std_msgs/msg/Empty {}`, while the robot is still past the limit (both wheel controllers subscribe to that topic; `-w 2` because plain `--once` re-arms only one wheel, as discovery of the second subscriber takes about 3 s). It records for `TIPOVER_RECORD_S` = 16 s (the refusal came 11.4 s after the first write in a slow container) | Both `left_` and `right_add_feedforward_controller` log `TIPOVER CUTOFF TRIPPED` before the robot is released (so in the first cycle they write); every command written while `\|pitch\| > 35 deg` is exactly `0.0` on both wheels (at least 50 such samples, or it says `COULD NOT JUDGE`); **both** controllers log `Tipover re-arm refused` (each within 5 s of the message) and neither logs `RE-ARMED`; the commands are watched for at least 1 s after the refusal; the controllers are still active at the end |
| C3 | `teleop:=true` | Finding C-3. The real `Joystick.py` and `DiffDriveController.py` run. A fake gamepad publishes `TwistStamped` (`linear.x` = 0.05 m/s) on `/diffbot_base_controller/cmd_vel` at 20 Hz for 2 s and then stops | `command_timed_out` goes `False` while it drives and `True` between 0.2 s and 0.3 s after the last command; the velocity reference is non-zero while driven and exactly `[0, 0]` after the timeout; both log lines appear, and `teleop commands resumed` comes **before** the first `teleop command timed out` (a time-out logged before any command had arrived would not show the watchdog firing); the controllers are still active and `/dynamic_joint_states` keeps arriving. Every time is a **send** time (the DDS source timestamp of each message, including the check's own commands, which it hears back), so a stall of the checking script does not move what it measures. If the script stalls and sends a command late, the watchdog must fire in that gap too: no `True` may come sooner than 0.2 s (less 10 ms) after the latest command, and in every silence longer than 0.3 s the node must be timed out by 0.3 s after it began. Since 2026-11-07 a command counts only once its `publish()` call has returned and `DiffDriveController` has run two 10 ms cycles since (seen from the reference it publishes every cycle), because in the container a `publish()` call and the node itself have each been seen to stall about 0.3 s ([journal](../journal/2026-11-07-c3-v-verifier-noise.md)) |
| C2 short | `initial_pitch_deg:=2.0`, IMU outage of 0.03 s (3 reads) starting 1 s after release | Finding C-2 downstream | No `ERROR`: `controller_manager` does not deactivate the hardware; the wiring is intact afterwards; no exported pitch sample is exactly `0.0` or NaN after the first finite one |
| C2 long | the same, outage of 0.2 s (20 reads) | | `SimPlant`'s `read()` returns `ERROR` and `controller_manager` logs `Deactivating following hardware components as their read cycle resulted in an error ... SimRocko`; no exported pitch sample is exactly `0.0` or NaN. The wiring is **not** required to stay up. The script prints what `controller_manager` did next, but does not judge it |
| H6 | `initial_pitch_deg:=2.0` | Finding H-6. Reads `/pitch_pid_controller/controller_state` and the plant's exported `pitch_rate` | `feedback_dot` is finite, reaches at least 1 deg/s, equals the exported `pitch_rate` of the same cycle (tolerance 1e-9 deg/s), and `error_dot` is `-feedback_dot` |
| FF | `teleop:=false` | Finding H-11 (the feedforward age limit). Two `ros2 topic pub` processes publish 5 rad/s on `/left_feedforward` and `/right_feedforward` at 100 Hz (the rate `DiffDriveController.py` uses) for 4 s, then are killed | A listener (`scenario_checks.py feedforward-sends`) records when every message was really sent (its DDS source timestamp). If a time-out was logged in the last `FF_LOG_GAP_S` (1 s), the publishers keep going (up to `FF_EXTEND_MAX_S`, 5 s more) until 1 s has passed without one, so the controller's one-line-per-second log limit cannot hide the line FF waits for. Each wheel's controller logs `Feedforward on ... ms old` exactly once after that wheel's **last message sent**, between 0.19 s and 0.5 s after it (`FF_AGE_S` less one 10 ms cycle, and `FF_AGE_S` + `FF_MARGIN_S`), never before the publisher started; the controllers stay active (only the feedforward term goes to 0.0). A time-out logged while the publisher was still running is printed as a note with the send gaps the listener saw, not a failure; if it came less than 1 s before the last message and no line follows, FF exits 2 (could not judge). `FF_AGE_S` (0.2) must equal `kMaxFeedforwardAgeNs`: `sim/check_sim_config.py` compares them in Tier 0 |

Each run also checks the wiring unless the check says otherwise: the four cascade controllers and
`joint_state_broadcaster` active, the `SimPlant` component active, all 14 interfaces present, both
wheel commands claimed. The robot is held still until the cascade first commands both wheels (see the
*release hold* blind spot below), so the clock of a run starts there. In S1 command and S2a the robot
falls past 35 deg, so the tipover cutoff trips; the run prints a note, and neither check depends on it,
because the fall is already under way when it trips.

#### The balance scenarios: `verify_sim.sh scenarios` (a separate mode)

The checks above ask whether the plumbing and the safeguards work. The scenarios ask whether the
controllers **hold the simulated robot up** (`bootstrap-sim` step 7). They are a separate mode, never
part of the default run, so a red scenario cannot hide a red plumbing check and a green default run
never claims balance:

```bash
docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_sim.sh scenarios
docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_sim.sh scenarios U S2b   # only some
```

Five launches: **U** (hold upright 10 s from a 2 deg start), **I** (recover from a 100 deg/s push to the
lean rate), **V** (follow a 0.1 m/s velocity step sent through the real `DiffDriveController.py`), **S2b**
(U with the pitch loop's sign flipped, `pitch_sign:=1`: it must fail by the pitch diverging within 1 s)
and **AB** (U with the as-built signs, `encoder_sign:=1`: a characterisation of finding H-12, expected to
fall by the slow run-away, no sooner than 3 s; its time to fall is printed). U, I, V and S2b run with
`encoder_sign:=-1`, an **assumption** printed with every result (proposal 0006 asks a human for the real
sign). The thresholds, and why each is what it is, are in one place, the scenario block at the top of
`verify_sim.sh`; the [stage 5 journal](../journal/2026-11-03-stage5-balance-in-sim.md) has them as a
table, committed before any tuning. Each run writes `samples.csv` (pitch, pitch rate, mean measured
velocity, wheel travel and the right wheel's velocity against time since release) and `metrics.json`.

Each scenario prints `SCENARIO <name>: PASS`, `FAIL` or `COULD NOT JUDGE` (S2b and AB print `ok` when they
behave as expected). Exit 0 only if U, I and V pass and S2b and AB are `ok`; 1 if any is not; 2 if a run
could not be judged and none failed. Until the sim gains were tuned, a failing U, I or V printed
`FAIL (expected until the sim gains are tuned)` (the switch is `SCENARIOS_EXPECTED_RED` in the script, 0 since
stage 5c-1), and the exit status was 1 either way. With the hardware gains U, I and V fail (seen 2026-11-03,
exit 1; stage 5 journal); with the sim gains tuned in stage 5c-1 the mode exits 0 (U, I, V `PASS`, S2b and AB
`ok`; [stage 5c-1 journal](../journal/2026-11-03-stage5c1-sim-gain-tuning.md), with each scenario's distance from its thresholds).
**V's overshoot metric (changed 2026-11-07, limit unchanged):** the measured velocity comes from the
encoder replica, which divides each cycle's travel by an assumed 10 ms (finding U-2), so a control cycle
the container runs late (26-30 ms seen) reads as a 15-20 % jump that decays over 0.2 s. Judged on the
largest exported sample, that failed nominal V runs at random (32.6 and 37.9 % against 30 %). V now judges
the peak of the same velocity recomputed with each cycle's real length (from the plant samples' stamps);
the highest exported sample is still printed. With it: 20 of 20 nominal runs pass (15.9-24.5 %; exported
18.6-30.7 %) and the corner `m+h+k−d+` still fails 3 of 3 (46.5-46.8 %)
([journal](../journal/2026-11-07-c3-v-verifier-noise.md)).
U starts at 2 deg on purpose: from exactly upright the model's equilibrium is exact, and U passes
trivially (seen, in a scratch copy; see the table of deliberate breaks below).

### The host unit test of the plant model (not a tier)

`tools/sim_plant_model_test.cpp` tests `SimPlantModel.hpp`, the physics inside `SimPlant`, against the
numbers in [sim-plant.md §7.1](sim-plant.md). It needs only a C++17 compiler, no ROS. From the repo
root:

```bash
g++ -std=c++17 -Wall -Wextra -I rocko_env/rocko_env/hardware/sim/include \
    -I rocko_env/rocko_env/hardware/sensors/icm20948/include \
    tools/sim_plant_model_test.cpp -o /tmp/sim_plant_model_test && /tmp/sim_plant_model_test
```

Success is the line `all SimPlantModel tests passed` and exit status 0 (80 `ok` lines on 2026-11-02; every
check prints `ok` or `FAIL`). It is **not a tier** and **no script runs it**, so nothing makes anyone run it. It
checks that the model computes what its specification says, with the estimated parameters. It cannot
check that the specification matches the robot.

### The host tests of the stage 4 safeguards (not tiers)

Five more tests need no ROS and run on the laptop or in the container, from the repo root. Each
prints its own success line, exits non-zero when a check fails, and **no script runs any of them**, so
nothing makes anyone run them.

```bash
g++ -std=c++17 -Wall -Wextra -I rocko_env/rocko_env/controllers/add_feedforward/include \
    tools/tipover_latch_test.cpp -o /tmp/tipover_latch_test && /tmp/tipover_latch_test      # all TipoverLatch tests passed
python3 -I tools/teleop_watchdog_test.py                                                    # 22 of 22 checks
python3 -I tools/computer_joystick_heartbeat_test.py                                        # 5 of 5 checks
python3 -I tools/imu_node_test.py                                                           # 7 of 7 checks
g++ -std=c++17 -Wall -Wextra -I rocko_env/rocko_env/controllers/add_feedforward/include \
    tools/feedforward_timeout_test.cpp -o /tmp/feedforward_timeout_test && /tmp/feedforward_timeout_test   # all FeedforwardTimeout tests passed
```

| Test | Checks | Cannot show |
| --- | --- | --- |
| `tools/tipover_latch_test.cpp` (12 test functions) | `TipoverLatch.hpp`, the trip and re-arm decision of finding H-4: trips above the limit and not at it; a NaN pitch neither trips nor re-arms; no automatic re-arm; a re-arm needs a request **and** `\|pitch\|` at or below the re-arm band (`rearm_max_pitch_deg`, 10 deg in the test), so a pitch between the band and the limit is refused; `is_valid_rearm_max` accepts only a value above 0 and below the limit; a request is used up by the next update; a limit of 0 or NaN disables it | That `AddFeedforwardController` calls it correctly (the sim check H4 looks at that), the IMU, the motors, the robot |
| `tools/teleop_watchdog_test.py` (22 checks) | `CommandWatchdog` and the command path of `DiffDriveController.py` and `Joystick.py`, loaded with stand-ins for `rclpy` and a fake clock: age counted from a command's arrival on a monotonic clock; the velocity reference and feedforwards are published as zero past the timeout and publishing never stops; log and `Bool` topic change only on a transition; `Joystick.py` publishes only commands that really arrived and survives a clean disconnect, and drops a connection that sends a non-finite or out-of-range value | ROS timing, DDS, a real TCP link from the laptop, the robot |
| `tools/computer_joystick_heartbeat_test.py` (5 checks) | `send_loop()` in `joystick/computer_joystick.py`, over a socket pair with a fake clock and no gamepad: an unchanging stick is resent every 0.05 s, a change is sent at once, a packet is still 40 bytes | Network timing, a real gamepad, pygame |
| `tools/feedforward_timeout_test.cpp` (7 test functions) | `FeedforwardTimeout.hpp`, the age decision of finding H-11: nothing arrived gives 0.0 and is not a time-out; a value is used up to and at the 200 ms limit and is 0.0 one nanosecond past it, logged once (`just_timed_out()` is true for one update only); a fresh message after a time-out is used at once; a steady 100 Hz stream never times out; a stamp a little later than the loop's `now` counts as age 0; a real value of 0.0 is not confused with "none" | That the controller calls it, that its subscription callback stamps the message, DDS delivery, the clock on the Pi, that 200 ms is a good limit |
| `tools/imu_node_test.py` (7 checks) | `ICM20948.py`'s service reply (findings C-2, H-6 and M-7), loaded with stand-ins for `rclpy`, the sensor libraries and the service type, and a fake BNO055: started with no parameters (as the launch files do), `read_pitch_rate` is false, the node makes one sensor read (the Euler angles, no gyro read), sends `pitch_rate_deg_s` as NaN and warns about nothing; with `read_pitch_rate` true: good angles and gyro give `success` and `rate = sign * degrees(gyro[axis])`; a bad gyro (None, an exception, NaN) makes only `pitch_rate_deg_s` NaN while the angles and `success` stay good; so does a rate beyond +/-2000 deg/s (`MAX_ABS_PITCH_RATE_DEG_S`, which must equal the plugin's `kMaxAbsPitchRateDegS`), and the node logs it; a still gyro (0, 0, 0) is a real reading, not a failure; bad angles give `success = false` and NaN angles whatever the gyro says | The real driver's units or timing, the I2C bus, the C++ plugin's separate rate handling, the robot |

### What a pass proves, and what it cannot show

| Tier | A pass proves | It cannot show |
| --- | --- | --- |
| 0 | The URDF expands and parses in seven expansions (balancing and tank, mock off and on, plus balancing with `use_sim_plant:=true`, without and with `use_mock_hardware:=true`, where the sim must win, plus tank mode asked for `use_sim_plant:=true`, which tank mode ignores by design), and each holds the right hardware: the two real motor plugins (and the IMU, in balancing mode) when the mock is off, exactly one `MockRocko` system when it is on, exactly one `SimPlant` system (`SimRocko`) and nothing else for the sim, and still the two real motor plugins and no `SimPlant` for the tank request; no plugin of any other kind; and the mock exports exactly the same interfaces (joints, sensors and gpio; a duplicate counts as a difference) as the real hardware, and so does the sim: 14 in balancing mode (the 12 of before plus a `pitch_rate` state on each wheel joint, finding H-6), 6 in tank mode. A pass prints 12 `ok` lines at the left margin (four more, indented, come from the config check below) and `PASS`. `sim/check_sim_config.py` also checks that `rocko_sim_controllers.yaml` equals `rocko_controllers.yaml` in everything except the `gains` maps and one allowed difference named in the script (`INTENTIONAL_WIRING_DIFFERENCES`: the sim's `pitch_pid_controller` has `reference_and_state_interfaces: ["pitch", "pitch_rate"]`, the hardware file `["pitch"]`; the entry fails if either file stops having exactly those values) (values compared with their type, so `100` and `100.0` differ; 17 wiring values equal plus 1 allowed difference on 2026-11-02); that in both files each controller's `gains` keys are exactly its `dof_names` and every `gains` key other than the tunable numbers (`p`, `i`, `d`, the clamps, `feedforward_gain`) is equal; that every simulator setting's default is the same in `SimPlantModel.hpp`, `rocko.urdf.xacro` and (for the settings in its `SIM_ARGUMENTS`) `rocko_sim.launch.py`, that each `<param>` in `rocko.ros2_control.xacro` is wired to the xacro arg of the same name, and that an `int` setting is written as an integer; and that the IMU fault limit the sim copies equals `ICM20948.hpp`'s. The guard hook blocks what it should and passes what it should. The git hooks in `.githooks/`, installed by copy in a throwaway repository, refuse and allow what they should | That anything compiles or runs. That the git hooks are installed in your clone |
| host unit test (not a tier) | The counting logic of `ImuFaultTracker`: it trips at exactly the consecutive limit and not before; a good read clears the run; a sensor that alternates 4 bad and 1 good reads, which never reaches the consecutive limit, trips the 10-of-20 window; the window clears after 20 good reads; the total count is never reset; one bad read in ten never trips it; a custom window limit is honoured | That `ICM20948.cpp` calls it correctly, that the limits are right for the robot, or anything about the sensor, timing or physics |
| compile-only (not a tier) | `Motor12VoltQuadEncoder.cpp` parses, every name resolves, every call matches a declared signature, and the ROS 2 headers it uses exist in Jazzy. First run, 2026-10-31, on the unmodified file: pass. Rerun on the edited file (the new `write()`, review round 2): pass. It was not made to fail on the new `write()` | That the real WiringPi signatures match: the stubs are copied by hand, so a stub that differs from the real header would hide a mistake. That it links, runs, behaves, is fast enough, or builds on a Pi |
| 1 | `rocko_interfaces` and `rocko_env` compile and link; declared dependencies install | Anything about **`Motor12VoltQuadEncoder.cpp`** (left out off-Pi: it needs WiringPi; only the compile-only check above type-checks it). Anything at runtime |
| 2 | Every controller **the launch file spawns** (the four cascade controllers in balancing mode; the diff-drive controller and joint-state broadcaster in tank mode) loads against the mock, claims interfaces the hardware layer exports, chains, activates and stays active; `Joystick.py` and `DiffDriveController.py` have not crashed. It refuses to run while another controller manager is up, so it cannot be fooled by a leftover one | **Physics.** The mock echoes each command back as a state and holds pitch at 0.0, so a controller that would drop the robot on the floor passes just the same. It does not start every controller named in the YAML, and it skips the IMU and encoder nodes |
| host unit test of the plant (not a tier) | `SimPlantModel` computes what [sim-plant.md §7.1](sim-plant.md) says: the falling pole is 8.59149 s^-1 (expected 8.5915); the fall from 2 to 35 deg with no torque takes 0.427297 s (expected 0.4273); through the 100 Hz loop with a 20 ms IMU delay it takes 0.45 s; plus the motor, delay-buffer, `write()` copy, IMU-fault and parameter-validation rules, in 80 checks, including the stage 4 pitch-rate rules (the rate is the current lean rate with no IMU delay, and a failed read holds the rate like the angle) | That the specification matches the robot. Every physical parameter is an estimate, so a pass says the code does what the document says, not that either is true |
| host unit test of the tipover latch (not a tier) | `TipoverLatch` makes the H-4 decision as specified, in 12 test functions: trips for `\|pitch\|` above 35 deg and not at it, forward or backward; a NaN pitch neither trips nor re-arms; it stays tripped after the pitch comes back; a re-arm is refused while past the limit or between the 10 deg re-arm band and the limit, and accepted inside the band; the re-arm band must be above 0 and below the limit; a request sent while armed is not kept for a later trip; a limit of 0 or NaN disables it; `reset()` re-arms and drops a pending request | That the controller calls it correctly, that 35 deg is a good angle, anything about the IMU, the motors or the robot |
| host tests of the watchdog and the heartbeat (not tiers) | `CommandWatchdog` counts age from a command's arrival on a monotonic clock, treats "no command yet" and a NaN timeout as expired, and `DiffDriveController` then publishes a zero reference and zero feedforwards without ever stopping, logging and publishing the `Bool` only on a change (22 of 22 checks, with stand-ins for `rclpy`); `Joystick.py` publishes only commands that arrived, survives a clean disconnect and drops a connection that sends a garbled message; the laptop sender resends an unchanged stick every 0.05 s and a change at once (5 of 5 checks); a separate test (`imu_node_test.py`, 7 of 7) checks that by default (`read_pitch_rate` false, finding M-7) the gyro is never read and the rate is NaN, that with it on a bad gyro value or a rate beyond +/-2000 deg/s makes only the rate NaN, and that a still gyro is a real reading | Real ROS timing and DDS (the C3 sim check looks at those), a real TCP link and gamepad, the laptop-to-Pi path, the robot |
| 3 (built, with the limits in the status table) | `verify_sim.sh`: the four cascade controllers and `joint_state_broadcaster` load against `SimPlant` unchanged and go active with all 14 interfaces; with no wheel command the simulated pitch does not move (0.0000 deg measured, limit 0.01); with a 5 rad/s feedforward it moves at least 0.5 deg (it moved 0.06 s after release, by 60.23 deg in the latest stage 3 run; that figure varies from run to run and only the >= 0.5 deg threshold matters, so the pitch responds to the wheels); with the motors producing no torque it passes 35 deg within 0.48 s of starting to fall (0.440, 0.432 and 0.430 s measured; the model alone gives 0.4273 s). **Stage 4 checks, in the model only:** *H4* started at 40 deg, both controllers log `TIPOVER CUTOFF TRIPPED` in the first cycle they wrote, all 801 of 801 commands written while `\|pitch\| > 35 deg` were exactly `0.0`, a re-arm sent at 60.1 deg was refused (the first version of the check; it now sends one `/tipover_rearm` message and requires both controllers to refuse it, and it passed that way), and the controllers stayed active. *C3* with a fake gamepad at 20 Hz for 2 s: `command_timed_out` went `False` 0.014 s after the first command and `True` 0.207 s after the last (allowed window 0.2 to 0.3 s); the velocity reference was `[0.05, 0.05]` in 195 of 195 messages while driven and exactly `[0, 0]` in 170 of 170 afterwards; the controllers stayed active and 170 plant samples arrived after the timeout. *C2* from a 2 deg start: a 0.03 s outage (3 reads) gave no `ERROR` and no `0.0` or NaN pitch sample (a held value shows as a run of 5 identical samples); a 0.2 s outage made `SimPlant`'s `read()` return `ERROR` and `controller_manager` 4.48.1 then did what the paragraph after this table describes, with 0 samples of `0.0` or NaN pitch. *H6* over 600 pairs, `feedback_dot` equalled the exported `pitch_rate` exactly (largest 46 deg/s) and `error_dot` was `-feedback_dot`. *FF* (2026-11-03, round 4; timed from each wheel's last message sent since the flakiness fix below, 0.19 to 0.5 s): after the publishers were killed both wheels logged the time-out once and the controllers stayed active. **C3 and FF flakiness, cause found (2026-11-03):** the checking processes (Python ROS nodes) sometimes stop for about a third of a second in the container (0.327 to 0.372 s measured; in 5 of 40 FF runs two separate publisher processes stopped at the same moment; a ROS-free loop never did; cause inside the ROS 2 / Fast DDS stack, not found). The old C3 timed what it saw by *arrival*, so a stall looked like a watchdog fault: as it was, it failed 1 of 10 runs, and 1 of 10 against a build with the H-11 age limit off. Both checks now judge DDS *send* timestamps, C3 judges every gap between its commands, and FF times from each wheel's last message sent: C3 20 of 20 and FF 20 of 20 green, both red on their deliberate breaks. **C3 flakes again (seen 2026-11-04, stage 5c-2):** 4 failures in 15 runs, a new way: `command_timed_out went True only 0.15-0.16 s after a command, sooner than the 0.2 s timeout`; 1 of 8 with `verify_sim.sh` and the sim launch file as they were before stage 5c-2, so not caused by it. Cause not shown, not fixed ([stage 5c-2 journal](../journal/2026-11-03-stage5c2-sweep.md#3-after-the-sweep)). **Cause found and fixed (2026-11-07):** two more places the same ~0.3 s stalls happen, neither a watchdog fault: a `publish()` call that blocks after the middleware stamped the command (0.30 s measured; an independent listener got the command 0.30 s after its stamp), and `DiffDriveController` itself stopping (its timer did not run, then it ran before the waiting commands). C3 now counts a command only once its `publish()` returned and the node has run two cycles since; 20 of 20 green (two of them with a 0.30-0.33 s `publish()` block the old rule would have failed), red on the three breaks ([journal](../journal/2026-11-07-c3-v-verifier-noise.md#2-check-c3-two-causes-both-delays-c3-could-not-see)). Details: [journal](../journal/2026-11-03-c3-ff-flaky-checks.md). Whether a Pi shows such stalls is not known | **Everything the model omits** (backlash, PWM aliasing, battery sag, the floor, turning; see [sim-plant.md §6](sim-plant.md)), and **everything that was estimated**: mass, centre-of-mass height, inertia, motor constants, IMU delay, wheel radius. Nothing measured. Whether the real robot balances. The scenarios that ask whether a controller *holds* the simulated robot up (`verify_sim.sh scenarios`) pass with the tuned sim gains of stage 5c-1, which only says the cascade balances in simulation under an estimated model and an assumed encoder sign, V narrowly; S2b shows a sign error in the pitch loop is caught in the model, not on the robot. For the safeguards: that 200 ms is a good feedforward age limit (a proposal), whether `DiffDriveController.py` really dies in practice, and what zeroing a turn's feedforward does to a leaning robot; nothing about the Pi, the motor plugin or the real IMU node; that 35 deg, 10 deg and 0.2 s are right (all are proposals); the 10 to 35 deg re-arm band (host test only); an accepted re-arm; what the cutoff does to the integrators upstream of it (finding H-10); that the cutoff can trip at all if the IMU node stops answering (C-1); a frozen IMU that holds a safe-looking value (H-9); the plugin's separate handling of the gyro rate (the sim ties the rate to the pitch); the sim sends H6's rate with no delay, so it shows the wiring and not the benefit. A pass is a statement about a model |
| 4 | Whether the robot actually balances | — |

**Measured, not assumed:** in the mock the pitch controller's `feedback` is exactly `0.0` (not NaN)
and it computes every cycle — Jazzy's `GenericSystem` zero-fills non-standard state interfaces. To
see it: start `rocko.launch.py use_mock_hardware:=true` in the container, then run
`ros2 topic echo /pitch_pid_controller/controller_state --once` and read `feedback`.

**What `controller_manager` did after a long IMU outage (check C2 long, observed 2026-11-02, Jazzy
`controller_manager` 4.48.1, in the container, with `SimPlant`).** `read()` returned `ERROR`, and the
log then held `Deactivating following hardware components ... [ SimRocko ]`, so the whole hardware
component was deactivated. All five controllers, `joint_state_broadcaster` included, were deactivated
too, with `Not acceptable command interfaces combination` and `Error while attempting mode switch
when deactivating controllers in read cycle!` logged along the way. The end state: every controller
`inactive`, `SimRocko` `unconfigured`, and not one exported pitch sample of `0.0` or NaN. This answers a
question that three review rounds had left open ("what does `controller_manager` do after an `ERROR`?")
**for this one component in this one version**. It does **not** show what the motors do on the robot:
there the IMU is its own component (the IMU plugin) and the motor plugin is another, so what the
manager deactivates after an IMU `ERROR`, and whether the motor plugin keeps calling `write()`, is not
shown by this run (finding [H-7](../claude-analysis/balance-loop-findings.md#h-7--the-motors-keep-their-last-command-when-the-cascade-stops)
stays partly mitigated and unconfirmed on hardware). Which controllers `controller_manager` deactivates
when only the IMU component fails, and whether `AddFeedforwardController::on_deactivate` then writes
`0.0` to a live motor plugin, can only be seen with the real hardware layout (a Pi) or a sim that splits
the plant in two (neither exists).

## Evidence that the tiers can fail

A tier is not "built" until it has been seen to fail on something deliberately broken, **for the right
reason**. (Several of the controls below first failed for the *wrong* reason, or did not fail at all, and were
fixed; see the journal.)

| Tier | Break | Result |
| --- | --- | --- |
| 2 | Delete the `pitch` state interface from the mock block | `FAIL`, exit 1: `state interface 'left_wheel_joint/pitch' is not exported`, all four controllers `inactive` |
| 2 | Remove the executable bit from `Joystick.py` | `FAIL`, exit 1: `node Joystick.py crashed` |
| compile-only | Break a copy of `Motor12VoltQuadEncoder.cpp` in the container's `/tmp` and pass it as the script's argument: put a string where an `int` is expected; delete a `;`; remove `#include <softPwm.h>`; misspell `HW_IF_VELOCITY` as `HW_IF_VELOCIT` | each `FAIL (compile-only)`, exit 1, for the right reason: `invalid conversion from const char* to int`; `expected ';' before 'return'`; `softPwmCreate was not declared`; `not a member of hardware_interface`. The unmodified file passes with exit 0 |
| host unit test | Three deliberate breaks to `ImuFaultTracker.hpp`, one at a time: disable the windowed trip; change `>=` to `>` in a limit test; make a good read stop clearing the consecutive run | each went red on a named assertion, and the unbroken header passes with `all ImuFaultTracker tests passed` |
| 1 | `build_ros.sh` with the flag `OFF` where there is no WiringPi | CMake `WiringPi not found`, then `ERROR: colcon build failed`, **exit 1**, no `Done!`; the next normal run recovers (its explicit `-D` overrides the cached value) |
| 1 | Skip flag `ON` while a (fake) `libwiringPi` is present | CMake stops: `ROCKO_SKIP_WIRINGPI=ON, but WiringPi IS installed here`, exit 1 |
| 1 | Skip flag `OFF` with a (fake) `libwiringPi` present — the Pi path | Configures with 0 CMake errors and compiles `Motor12VoltQuadEncoder.cpp`, which then fails on the missing `wiringPi.h` as it must off-Pi. This shows the Pi branch is taken; it is **not** a Pi build |
| 0 | Swap the mock block's `xacro:if` for `xacro:unless` | `verify_urdf.sh` `FAIL`, exit 1: the real-mode description holds a mock (`mock=1`) and the mock-mode one holds nothing (`mock=0`) |
| 0 | Delete one `yaw` state from the mock block | `verify_urdf.sh` `FAIL`, exit 1, from the parity check: `< left_wheel_joint state yaw` (real has it, mock does not) |
| 0 | Give only the mock a `<sensor>`; make the real hardware export an interface twice; put a third plugin in the mock block | each `FAIL`, exit 1 (`> sensor mock_only state orientation`; `< joint left_wheel_joint state velocity`; `plugins=2`) |
| 2 | Start a mock launch, then run `verify_mock.sh` | `ERROR: a controller_manager (ros2_control_node) is already running`, exit 2 |
| pre | Run `check_container_mount.sh` from a real git worktree while the container mounts the main checkout | `FAIL`, exit 1, with the fix. After the printed `up -d --force-recreate` from the worktree: `ok` from the worktree and `FAIL` from the main checkout. After the same from the main checkout: `ok` again |
| pre | Delete that worktree and make a new one at the same path, so the container still mounts the deleted folder | `FAIL`, exit 1. Plain `up -d` prints `Running` and the check still fails; only `up -d --force-recreate` fixes it |
| pre | Run it with no container (`COMPOSE_PROJECT_NAME=zzz`), with no `docker` on `PATH`, and through a symlink | exit 2 each time, with a message that names the real cause |
| pre | Send SIGINT, SIGTERM and SIGHUP to the script's process group while a stand-in `docker` hangs; then SIGKILL | the marker file is removed for the first three; SIGKILL leaves it, which `.gitignore` covers |
| 1 | Append a comment to `AddFeedforwardController.cpp`, build, restore the original with its older timestamp, build again | the second build recompiles nothing: build output from another checkout can survive a switch, so `check_container_mount.sh` printing `ok` is not enough |
| 0 | Reintroduce, one at a time, 28 bugs into a copy of the hook (among them `fault` matching `default`, a crash instead of failing closed, no case-folding, no symlink resolution, CRLF not normalised, comments not stripped, YAML compared case-insensitively or with its indentation ignored, a gain judged as a token instead of a whole line, an unmatched edit waved through, the Edit tool's newline-eating not modelled, a quadratic comment scan) | The hook test suite goes red for each: 28 of 28. It has 80 hook cases (79 on a case-sensitive filesystem such as Linux or WSL2, which skips the one case about differently-cased paths) plus a check that `settings.json` wires the hook up, and asserts the exit status as well as the decision. Rerun the mutation check with `.claude/hooks/mutation-check-guard.py` |
| 0 | Break each of twelve checks in `.githooks/pre-commit` and `pre-push` one at a time in a copy, with `.githooks/mutation-check.py`, which first runs the unbroken test and stops if that is red (any ref name allowed, a ref matched by its ending, any remote allowed, a deletion not recognised, no fast-forward check, a refusal only when we have the remote's commit, a refusal that forgets its `exit 1`, the person's `ROCKO_HUMAN=1` escape removed, or any value of it accepted, in each file as it applies; `main` not detected) | `.githooks/test-git-hooks.sh` goes red for each (between 1 and 11 cases). Several things first passed for the wrong reason and were fixed: a push that *git* refused counted as the hook's work, the mutation check printed PASS with a red baseline, and the test would have acted on a real repository had `GIT_DIR` (or `git -c` settings) been set (without the fix, in a copy, pointed at a stand-in repository, it committed to it, added a remote to its config and pushed several refs to its remote). 24 cases in a throwaway repository and remote. The hooks are installed by copying into `.git/hooks`: `core.hooksPath .githooks` stops working on a branch without the folder (a commit and a push of `main` went through). git 2.48.1 on macOS |
| host unit test of the plant | Two deliberate breaks to `SimPlantModel.hpp`, one at a time: flip the sign of the body torque; drop gravity | `tools/sim_plant_model_test.cpp` goes red on 9 checks and on 22 checks respectively; restored, it passes with `all SimPlantModel tests passed` |
| 3 | In the `SimPlant` plugin, freeze the pitch it reports | `verify_sim.sh` `FAIL`, exit 1: the S1 command run moved 0.0000 deg (needs 0.5), and S2a never reached 35 deg. Restored: `PASS`, exit 0 |
| 3 | In the plugin, ignore `disable_torque` | `verify_sim.sh` `FAIL`, exit 1: S2a never reached 35 deg (the pitch peaked at 2.92 deg), and 16 checks in the unit test failed. Restored: `PASS` |
| 0 | Leave the `roll` state out of the sim's `<ros2_control>` block | `verify_urdf.sh` `FAIL`, exit 1, from the sim-versus-real interface comparison |
| 0 | Put the real block's old condition back (so the real motors and IMU are also in the `use_sim_plant:=true` description) | `verify_urdf.sh` `FAIL`, exit 1: `plugins=4` in the `rocko sim=true` expansion, where exactly 1 is expected |
| 0 | Break the sim config in copies, one at a time: rename a `gains` key in the sim YAML; delete a `gains` map; add `angle_wraparound: true` to one; write `update_rate: 100.0`; wire `<param name="imu_delay_ms">` to `$(arg imu_delay_max_ms)`; give `belt_ratio` a literal; write `imu_delay_ms` as `20.0` in the xacro or the launch file | `sim/check_sim_config.py` `FAIL`, exit 1, each time with a line naming the cause. Changing only `p`, `i`, `d` and the clamp numbers in the sim YAML still passes (exit 0) (true when written, 2026-10-31; since 2026-11-03 only the `SIM_TUNED_GAINS` entries may change, next row) |
| 0 | 2026-11-03, the gains check (stage 5c-1), in scratch copies (`--root`): in the sim YAML change velocity `d` (not tuned), one `i_clamp_max`, pitch `i`, pitch `d` on one joint only, velocity `p` to the string `"20"`; in the hardware YAML change pitch `d` (tuned in the sim) and velocity `d`; plus `update_rate: 200`, `pitch_rate` renamed, the velocity loop reading `pitch`, `angle_wraparound: true` | `sim/check_sim_config.py` `FAIL`, exit 1, each with a line naming the key (the hardware pitch `d` line says the sim gains were tuned against 0.1). Control: pitch `d` 0.8 on both joints of the sim YAML passes (exit 0). Commands and output: [stage 5c-1 journal](../journal/2026-11-03-stage5c1-sim-gain-tuning.md) |
| 0 | 2026-11-07, a tuned gain missing on one joint, in scratch copies: delete `p:` from only the right wheel's entry of `pitch_pid_controller` in the sim YAML; the same in the hardware YAML; delete velocity `i:` from only the right wheel in the sim YAML. The 12 breaks of the row above re-run on the current files | Each of the three: `FAIL ... \`p\` (SIM_TUNED_GAINS) is missing for right_add_feedforward_controller/right_wheel_joint`, exit 1 (the check before this change passed the first two, exit 0). The 12 older breaks: as before (11 red, control green). Real tree: six `ok`. [Review-fixes journal](../journal/2026-11-07-stage5c-review-fixes.md) |
| 0 | Make the tank description hold a `SimPlant` plugin (a comment containing it, in the tank `<ros2_control>` macro); rename `struct SimPlantParams` so the checker cannot read it | `verify_urdf.sh` `FAIL`, exit 1, with `got sim=1` in the `rocko_tank_mode sim=true` line; and `sim config check could not run (exit 2; this is not drift)` |
| 3 | Give `released_problem` (in `verify_sim.sh`) a hand-made launch log whose release stamp is 9.5 s, 2.5 s or -1.2 s (after) before the last sample in `metrics.json`; and a `metrics.json` with no `last_stamp_s` | 9.5 s: no problem. 2.5 s and -1.2 s: `less than 3 s before the probe's last sample`. No `last_stamp_s`: says so. Run on the function alone, not in a full `verify_sim.sh` run |
| host unit test of the plant | Weaken four of the `invalid_reason()` checks in a copy of `SimPlantModel.hpp` (allow `imu_delay_max_ms` up to 100000; drop the `battery_voltage_V`, `body_depth_m` and mass-matrix checks) | 5 of the #17 checks go red; the real header passes all 70 |

| 3 (H4) | Launch the check with the cutoff off (`tipover_pitch_deg` 0, which the controller logs as `TIPOVER CUTOFF DISABLED`) | `verify_sim.sh H4` went red: the commands written past the limit were -400 rad/s (548 of 548 non-zero) and no controller logged a trip. Restored: `PASS` |
| 3 (C3) | Launch with `command_timeout_s:=1000`, so the watchdog never fires | `verify_sim.sh C3` went red (the failure message was not recorded in this page). Restored: `PASS` |
| 3 (C3, 2026-11-03, send-time version) | In a scratch copy, launch with `command_timeout_s:=1000`; `:=0.03`; `:=0.5` | red each time: `never went True in the 2.0 s after the last command`; `went True only [0.0377, ...] s after a command, sooner than the 0.2 s timeout`; `the gamepad went silent for longer than 0.3 s ... and command_timed_out did not go True within [0.2, 0.3] s of it`. Restored: `PASS` 20 of 20 |
| 3 (C3, 2026-11-07, "handled" version) | Scratch copies of `verify_sim.sh` launching with `command_timeout_s:=1000`; `:=0.03`; `:=0.5` | red each time: `never went True in the 2.0 s after the last command`; `went True only [0.0405, 0.0397, ...] s after a command the node had processed`; `the gamepad went silent for longer than 0.3 s at [1.95] s ... (first True after the last command: 0.510 s)`. The committed check: `PASS` 20 of 20 |
| 3 (C3 node stall) | `DiffDriveController.py` frozen with `SIGSTOP` for 0.3 s, 1 s into the drive (no code change) | the previous check: `FAIL: ... went True only [0.0039] s after a command` (the second flake, reproduced); the new check: `PASS` |
| 3 (C3 stall) | In scratch copies, a 0.35 s `sleep` before the checker's 40th command | the old check: `FAIL: ... went True 0.001 s after the last command` (the flaky failure, reproduced); the new check: `PASS`, judging the time-out in the gap. Also injected at command 21 (0.35 s and 0.25 s): `PASS` |
| 3 (C2) | Make a failed IMU read export `0.0` instead of holding the last value | `verify_sim.sh C2` went red with 4 exported pitch samples of exactly `0.0`. Restored: `PASS` |
| 3 (H6) | Set the sim's `reference_and_state_interfaces` back to `["pitch"]` | `verify_sim.sh H6` went red (the D term then has no rate to read; the failure message was not recorded in this page). Restored: `PASS` |
| 3 (S2b) | The S2b scenario itself: U with the pitch loop's sign flipped (`pitch_sign:=1`) | U failed by the pitch diverging, 35 deg 0.194 s after release (limit 1.0 s): `SCENARIO S2b: ok` (2026-11-03) |
| 3 (S2b) | In a scratch copy of `verify_sim.sh`, S2b without the flip (`pitch_sign:=-1`) | `SCENARIO S2b: FAIL: U failed, but not by a fast fall within 1.0 s`: U's failure was the slow oscillation, so S2b refused it as the wrong reason; exit 1 |
| 3 (AB) | In a scratch copy, AB with `disable_torque:=true` | `SCENARIO AB: FAIL: fell 0.449 s after release, sooner than 3.0 s: not the slow run-away of H-12`; exit 1 |
| 3 (I) | In a scratch copy, I with the push size 0 | `SCENARIO I: FAIL ...: no push seen: the largest pitch_rate jump near 2.0 s was 0.0 deg/s (need >= 80)`; exit 1. (The real run saw a 98.7 deg/s jump: the first end-to-end run of SimPlant's impulse) |
| 3 (V) | In a scratch copy of `sim/scenario_checks.py`, the fake gamepad on a topic nobody reads | `SCENARIO V: FAIL ...: the mean measured velocity did not reach 0.09 m/s within 3.0 s of the step`; exit 1 |
| 3 (U) | In a scratch copy, U from exactly upright (`U_START_DEG=0.0`) | `SCENARIO U: PASS`, `SCENARIOS: PASS (exit 0)`: the pass branch and exit 0 work, and a start at exactly 0 would make U meaningless (why it starts at 2 deg). The pass branches of I and V have **not** been exercised |
| host unit test of the tipover latch | Two deliberate breaks to `TipoverLatch.hpp` (made when the test had 10 cases) | each went red on a named case (the two breaks are not listed here); the unbroken header passes with `all TipoverLatch tests passed` |
| host test of the watchdog | Four deliberate breaks to `DiffDriveController.py` / `Joystick.py` (made when the test had 13 checks) | each went red (the four breaks are not listed here); unbroken: 22 of 22 now |
| 0 | `tools/feedforward_timeout_test.cpp`: two deliberate breaks of `FeedforwardTimeout.hpp` (which two was not passed to the author of this page) | the test went red on both (2 of 2). Restored: passes 7 of 7 |
| 0 | `tools/feedforward_timeout_test.cpp` again, after it was changed to assert on `just_timed_out()` and the returned value (round 4 fixes, 2026-11-03), compiled against scratch copies of the header: (a) `just_timed_out_` true on every timed-out cycle (edge detection removed); (b) `update()` returns the stored value even when stale | red on both: (a) `!f.just_timed_out()` in `test_old_value_becomes_zero_and_logs_once`; (b) the `== 0.0` check one nanosecond past the limit in the same test. Real header untouched: 7 of 7 |
| 0 | `tools/imu_node_test.py`, run against a scratch copy of `ICM20948.py` with the +/-2000 deg/s check disabled (2026-11-03) | `test_out_of_range_rate_is_nan_and_logged` went red (`gyro 40.0 rad/s: the rate must be NaN`), 6 of 7. Real node: 7 of 7 |
| 3 | Run check `FF` with the age limit infinite, and again with it set to 50 ms (`kMaxFeedforwardAgeNs` changed in a scratch build; `FF_AGE_S` left at 0.2) | red both times: with an infinite limit no time-out is ever logged after the kill; with 50 ms it is logged sooner than `FF_EARLIEST_S`. Restored: `PASS` |
| 3 (FF, 2026-11-03, timed from the last message sent) | The same two scratch builds (limit 1000 s; 50 ms) | red both times: `never logged 'Feedforward on ... ms old' after its last message`; `logged the time-out less than 0.2 s after the last message was sent` (0.057 s, 0.055 s). Restored: `PASS` 20 of 20 |
| 0 | In a scratch copy of the tree (`--root`), change `FF_AGE_S` in `verify_sim.sh` from 0.2 to 0.5 | `sim/check_sim_config.py` prints `FAIL  the feedforward age limit differs: kMaxFeedforwardAgeNs ... is 0.2 s, FF_AGE_S in verify_sim.sh is 0.5 s`, exit 1; the real tree was not touched and prints `ok` (2026-11-03) |
| (not shown to fail) | The later additions: the latch test's re-arm band cases, the watchdog test's 3 added checks, the heartbeat test's 5th, `imu_node_test.py` (including its new `read_pitch_rate` default test: not recorded), `H4`'s rule that both controllers refuse the one message, and `C3`'s rule that `resumed` comes before `timed out` | whether these were made to fail was not passed to the author of this page; treat them as unproven until it is |
**Also checked:** the real-hardware `<ros2_control>` elements (`use_mock_hardware:=false`) are
semantically identical to those at `HEAD` — 3 of 3 blocks in balancing mode, 2 of 2 in tank mode —
so the mock did not change what the robot loads.

## What the ladder has already caught

The first run of this ladder, while it was being built, found real defects. They are recorded here
because they show what each tier is for.

| Found by | Defect |
| --- | --- |
| Tier 2 (first launch) | `pid_controller` and `diff_drive_controller` were never declared in `package.xml`, so a fresh machine could not launch either mode |
| Tier 2 | The launch passed the URDF to `robot_state_publisher` as an unwrapped substitution, so any `: ` in an xacro comment made `launch_ros` fail with a YAML parse error |
| Tier 2 | Five Python executables were committed without the executable bit, so under `--symlink-install` they could not be run |
| Tier 1 (compiler warning) | `AddFeedforwardController` ignores the result of `set_value()` — finding **M-5** |
| Review of the hook | The guard hook failed *open*: a crash made the harness skip it. It now fails closed |
| Docs review, then a test | `build_ros.sh` could not fail: no `set -e`, ends in `echo`, so a failed build said `Done!` and exited 0. Tier 1 now exits 1 |
| Testing the fix | The container's `/ros2_ws/build_ros.sh` was a stale copy that no longer matched the edited host file (see the blind spots) |
| Asking whether the repo was ready for a new session | The container mounts one fixed folder, so a session in a git worktree would have run every tier against the main checkout's files and passed (see the blind spots) |

## Blind spots, stated plainly

- **The motor plugin is type-checked off-Pi, not built.** A change to `Motor12VoltQuadEncoder.cpp`
  gets Tier 0, the compile-only check (syntax and types, against hand-copied stub headers) and a
  reviewer's eyes, until it is built on a Pi. Tier 1 still leaves it out. The 2026-10-31 `write()`
  (findings H-2, M-1, M-3: always write the speed pin, a deadband, a clamp) is in exactly this state:
  compile-checked only, awaiting a robot.
- **Most of the controller stop logic still has no automated test.** The tipover latch (finding H-4)
  has a host unit test and the sim check H4, so its decision and its effect on the commands are
  checked, in the model. What stays unchecked is the rest of `AddFeedforwardController`: the
  NaN-to-zero rule, the 3-cycle hold, the refusal of non-finite commands and the `on_deactivate` stop
  (finding H-7). They are built by Tier 1 and loaded by Tier 2. Check C2 long deactivated the
  controller, so `on_deactivate` was called, but no check reads what it wrote (and the plant had
  already been deactivated by then). A gtest was declined in review round 2.
- **The IMU and encoder nodes are not exercised.** The mock launch skips `ICM20948.py` and
  `QuadEncoders.py` (they need I²C and GPIO), so the C++ plugins that call them are untested here.
  `ICM20948.cpp` is compiled by Tier 1, but the mock replaces that plugin, so no tier runs its code,
  including the C-2 failure path and the new gyro read (`ICM20948.py` reading `.gyro` once, the `pitch_rate` export and its 2000 deg/s plausibility limit, finding H-6). Only the counting rules of `ImuFaultTracker` have a test (the
  [host unit test](#the-host-unit-test-of-the-imu-fault-tracker-not-a-tier)); how `read()` uses it is
  not run anywhere. `SimPlant` applies the same counting rule to its own simulated IMU (a copy of the
  decision, not `ICM20948.cpp`), so it does not cover that file either. A sensor that freezes or resets
  while still returning valid-looking numbers is not detected by the code either (finding **H-9**).
- **Timing is not measured.** Container and mock timings say nothing about the Pi's 10 ms budget.
- **The simulator is a model with invented numbers.** Every physical parameter of `SimPlant`
  (masses, centre-of-mass height, inertia, motor constants, IMU delay, wheel radius) is an estimate,
  labelled `ESTIMATE, NOT MEASURED` in the code and in [sim-plant.md](sim-plant.md) §4. Nothing was
  measured on the robot, and no tier can check these numbers. The wheel radius has four disagreeing
  values in the repo (issue #16) and the sim picks none as right; its radius handling is tested only
  inside the header class, not through the plugin. A result from `verify_sim.sh` is about the model.
- **The sim holds the robot still until the controllers first command both wheels (the "release
  hold").** The hardware component activates seconds before the controllers are spawned, and a plant
  that fell in that gap would start every run already lying down. So `SimPlant` freezes the physics until
  both wheel commands are finite, and every sim time and scenario time counts from that moment. The
  robot has no such hold: its hardware is live as soon as it activates. This is a deliberate
  difference, and nothing has checked that it hides no problem; the release moment is only measured
  indirectly (the pitch starts to move about 0.06 s after it in the S1 command run).
- **The impulse parameters are not exercised end to end.** `impulse_time_s` and
  `impulse_angular_velocity_deg_per_s` are covered by the host unit test of the model, but no launch
  has run them and `verify_sim.sh` has no run that uses them. The recovery-from-an-impulse scenario does not
  exist. (The IMU-fault parameters `imu_fault_start_s` and `imu_fault_duration_s` are now run by check
  C2.)
- **The sim has one hardware owner; the robot has two.** `SimPlant` is one component that provides the
  wheels and the IMU. When its IMU rule trips and `read()` returns `ERROR`, the whole plant (wheels
  included) stops with it: check C2 long saw `controller_manager` deactivate `SimRocko` and all five
  controllers (see the paragraph under the proves table). On the robot the IMU and the motor plugin are
  separate components, and the motor plugin may keep driving its last command after the IMU fails
  (finding H-7). So this sim cannot show the H-7 behaviour, and a "safe" outcome after an injected IMU
  fault in the sim would not carry over.
- **The plant carries copies of robot code, which can drift, and only some are guarded.** The duty map
  in `SimPlantModel.hpp` is a copy of the new `Motor12VoltQuadEncoder::write()` (deadband, stiction
  offset, clamp), including its integer-division `MAX_RAD_PER_SEC` (119.38 rad/s, finding U-3 in
  [sim-plant.md](sim-plant.md), L-7 in the findings doc) on purpose; the encoder-node halving of speed
  is a copy of `QuadEncoders.py` (switch `model_encoder_node`). Guarded: a `static_assert` fails the
  build if `kWriteMaxRadPerSec` is "tidied" to 120 or more (it does not read the plugin, so a changed
  plugin constant still leaves the copy as it was), and `sim/check_sim_config.py` compares the IMU
  fault limits with `ICM20948.hpp`. Not guarded: the `write()` duty map (the deadband, the stiction
  offset, the clamp) and the encoder-node copy (its filter constant and assumed period). Nothing
  checks that those still match the files they copy.
- **`rocko_sim_controllers.yaml` is a copy of the hardware controller wiring, and its gains may
  differ on purpose.** `sim/check_sim_config.py`, run by `verify_urdf.sh`, fails if the wiring
  (controller names, types, `dof_names`, interfaces, `update_rate`) or any `gains` entry differs from
  `rocko_controllers.yaml`, or if a controller's `gains` keys are not its `dof_names`. The only gains
  allowed to differ are the ones listed in its `SIM_TUNED_GAINS` (since 2026-11-03, stage 5c-1: before
  that every gain number and clamp could differ unchecked); each entry pins the hardware value, so a
  change to that gain in the hardware file fails too, and each must be present for every joint in
  `dof_names` in both files (since 2026-11-07). It cannot say whether the sim gains are sensible;
  only the scenarios can, and only about the model.
- **All the hardware plugins, `SimPlant` included, override `on_init(const HardwareInfo &)`, which the
  stage 3 author reports as deprecated in `ros2_control`.** The Tier 1 build prints no compiler warning
  for it, and this page's author did not check the installed headers, so whether a later release
  removes it is unknown. Not changed.
- **`/ros2_ws/build_ros.sh` can be a stale copy.** `docker/compose.yml` mounts that one file by itself;
  a file mounted alone is pinned when the container starts, so replacing it on the host while the
  container runs leaves the container with an older version. Run `src/ROCKO-env/build_ros.sh`, which
  is always current. Not fixed in `compose.yml`.
- **The container serves one checkout at a time.** `docker/compose.yml` mounts `..` (the checkout
  holding that compose file) under a fixed container name, and every checkout's compose file is in a
  folder called `docker`, so compose treats them all as one project. A git worktree or a second clone
  is therefore verified against whichever checkout last ran `up -d`, and passes. `up -d` is itself the
  switch, and it takes the container away from a session in another checkout. A container per checkout
  (a project name per checkout and no fixed `container_name`) would remove the problem; `compose.yml`
  is not changed.
- **`check_container_mount.sh` closes only part of that.** It says whether the container sees your
  files. It does not say the build output is yours (measured: it is not recompiled when the files are
  older), so rebuild from clean after a switch. A recreated container also loses the ROS packages
  `build_ros.sh` installed, so that rebuild needs the network. Nothing makes anyone run the script,
  though it has been on the allow-list since 2026-10-30
  ([proposal 0001](../proposals/0001-allow-container-mount-check.md)). Only Docker Desktop
  for macOS was tried: Linux and WSL2 were not.
- **No tier can test a permission rule.** Whether a `Bash(...)` string in `settings.json` matches
  the command an agent types, and whether the guard hook still denies and asks under each permission
  mode, can only be seen in a live session, by a person (the session transcript's per-call
  `permissionDecision` records then say why each call was accepted or refused). The check is
  [proposal 0002](../proposals/0002-unattended-run-settings.md)'s *Test for a human*, and it must be
  run outside `auto` mode, where a classifier can approve a call whose rule does not match. The
  project default is `acceptEdits` since 2026-10-30 and nothing has been run in it yet. The git
  hooks in `.githooks/` are tested on a throwaway repository and are inactive in this one until that
  proposal's item E is done.
- **`sim/check_sim_config.py` has known gaps** (stage 3 review round 3, each shown on a copy, none fixed
  because the review cap was reached): a duplicated `<xacro:arg>` (xacro keeps the first, the checker
  the last); an override typed into `rocko_sim.launch.py` (`body_mass_kg:=3.0` in the xacro command, or
  an extra parameter dict on the control node); a struct member in `SimPlantParams` not written as
  `type name = value;`; a second or oddly spaced `<param>` tag; a Python crash mislabelled as drift; and
  nothing checks that each entry of `read_parameters()` in `SimPlant.cpp` points at the member of the
  same name. A wrong value there changes the simulated robot without a red line.
- **The impulse has not been run end to end.** The IMU-fault branch (C-2) now has: check C2 runs a
  short and a long outage through a launch.
- **The stage 4 safeguards are checked in the model, not on the Pi, and each blind spot below is
  specific.**
  - *H4 (tipover cutoff).* It watches the commands `controller_manager` publishes on its introspection
    topic (best-effort QoS, so a dropped message is possible; 801 commands were seen in the latest
    run), not the PWM. The 35 deg limit is a proposal and applies to the **reported** pitch, which
    includes the IMU node's unexplained `pitch_offset = 7.6` deg. A latched cutoff does not deactivate
    anything, so the upstream PID integrators keep winding while it is disarmed; the first command after a
    re-arm could be large (finding H-10), and the sim does not test a re-arm that is accepted, nor the
    10 to 35 deg re-arm band (the host latch test does). Activating a controller re-arms it without the
    band check. A frozen IMU that keeps a safe-looking pitch cannot trip it (H-9), and neither can a stopped
    IMU node, because the blocking `read()` (C-1) then never returns. `AddFeedforwardController.cpp` holds
    the tipover block twice, an inline block that runs and an unused duplicate helper, because the guard
    hook denied the move (next-steps row 1.14); nothing here checks that the duplicate matches. The three
    tipover keys are only code defaults in a file the hook does not watch.
  - *C3 (watchdog).* The fake gamepad publishes `TwistStamped` straight to
    `/diffbot_base_controller/cmd_vel`, so the TCP path (`Joystick.py`'s socket handling and the laptop
    sender's heartbeat) is only covered by the host tests with stand-ins, not by a launch. The timeout (0.2
    s) is a proposal. The reference steps to zero with no ramp (the `RateLimiter` is unused); what that
    does to a robot that is leaning was not measured.
  - *H6 (pitch rate).* The sim's rate has no delay and an assumed sign (`pitch_sign`), so the check shows
    the wiring (the D term reads the exported rate) and not that the benefit exists. The real YAML
    still says `["pitch"]`; the gyro axis and sign (`pitch_rate_axis`, `pitch_rate_sign` in
    `ICM20948.py`) are unverified guesses, and so is the gyro's unit across driver versions (older than
    5.0 returns deg/s, a rate 57 times too large). The sim ties the rate to the pitch in its fault policy;
    the plugin judges the rate on its own and never returns `ERROR` for it, and no sim check exercises that. Upstream
    `pid_controller` sets the rate **reference** once, at activation (finding L-8), which the sim hides
    because its robot is still then (rate 0.0). The sim file's allowed wiring difference was not made
    to fail by the author of this page.
  - *C2 (IMU outage).* See the paragraph under the proves table: it is one component in the sim.
- **The Pi build has not been run.** Nothing here compiles the motor plugin against real WiringPi (the
  compile-only check uses stubs) or checks that the `OFF` path produces a working `librocko_env.so`.
- **Dependency versions are not pinned.** `rosdep` installs the current Jazzy release of
  `pid_controller` and `diff_drive_controller` (4.42.1 in the container). ADR 0001 pins
  `ros2_control` ≥ 4.0.0, not these. Check what a fresh Pi actually gets.
- **A skip build cannot run the real motors.** Launched with the mock off it prints
  `Could not create class of type rocko_env::Motor12VoltQuadEncoder` and the controllers never come
  up. That is the plugin being absent; it fails safe.
- **Launch files can sidestep the gain guard** by loading a different YAML; the
  `ros2-control-reviewer` agent checks for it, nothing automated does.
- **The gain guard reads lines, not YAML.** It cannot see a quoted key (`{"p": 5}`), a `gains: |` block
  scalar, or an edit that moves a line out of its map. Closing that needs a YAML parser (PyYAML),
  which the host may not have; see the hook's docstring.
- **`foxglove_bridge` starts but nothing checks it.**
- **`rocko_gui.launch.py` looks stale.** It spawns `diffbot_base_controller`, which
  `rocko_controllers.yaml` does not define, and is not covered by any tier.
- **No tier opens RViz.** The RViz layouts are installed now (finding **L-4**; after a build,
  `install/rocko_env/share/rocko_env/rviz/` holds `rocko.rviz` and `rocko_view.rviz`), but that RViz
  starts with them needs a display, and `view_robot.launch.py` has not been run since its default
  argument changed.
- **Nothing here covers x86_64.**

## Keeping this page true

Change this page in the **same commit** as anything that changes a tier: a new check, a tier built, a
blind spot closed or opened. If a status here disagrees with what a command prints, the command is
right and this page is a bug.
