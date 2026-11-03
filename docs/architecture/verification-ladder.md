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
| 3 | Simulation | `verify_sim.sh` (container; needs a Tier 1 build first). The plant is `rocko_env/SimPlant`, described in [sim-plant.md](sim-plant.md) | **Partly built. Tier 3 is NOT called built.** Two of the checks that define "built" exist and pass: S1 (the real cascade runs against the simulated robot and pitch responds to the wheels) and S2a (with no motor torque the simulated robot falls). S2b (a copy of the upright scenario with one sign flipped must fail) and the upright, impulse and velocity-step scenarios **do not exist yet**. The plant's parameters are estimates, not measurements, so every result is about the model and says nothing about the robot. [ADR 0003](../decisions/0003-simulation-target.md) is still `Proposed` (it names Gazebo; `SimPlant` was built first) |
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

### The simulation check, `verify_sim.sh` (Tier 3, partly built)

`verify_sim.sh` (repo root, runs in the container, after a Tier 1 build) launches
`rocko_sim.launch.py teleop:=false` three times and reads the simulated pitch with `sim/sim_probe.py`.
Run it from the repo root on the laptop:

```bash
docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_sim.sh
```

Exit 0 prints `PASS (simulation, S1 and S2a)`; 1 is a failure (it says which run and why); 2 means it
could not run (no build, or a `controller_manager` is already running), or `COULD NOT JUDGE (simulation)`: the probe saw no messages or no movement, so read its reason and the kept run directory. The three runs, with the
thresholds it holds (each is explained next to its definition in the script):

| Run | What it does | Passes when |
| --- | --- | --- |
| S1 baseline | Upright start, no wheel command beyond the cascade's own | Pitch stays within 0.01 deg of its start |
| S1 command | The same, plus a 5 rad/s feedforward on both wheels | Pitch moves at least 0.5 deg: it changes **because** the wheels moved |
| S2a | `disable_torque:=true`, start at 2 deg | `\|pitch\|` passes 35 deg within 0.48 s of starting to move |

Each run also checks the wiring: the four cascade controllers and `joint_state_broadcaster` active,
the `SimPlant` component active, all 12 interfaces present, both wheel commands claimed. The robot is
held still until the cascade first commands both wheels (see the *release hold* blind spot below), so
the clock of a run starts there.

### The host unit test of the plant model (not a tier)

`tools/sim_plant_model_test.cpp` tests `SimPlantModel.hpp`, the physics inside `SimPlant`, against the
numbers in [sim-plant.md §7.1](sim-plant.md). It needs only a C++17 compiler, no ROS. From the repo
root:

```bash
g++ -std=c++17 -Wall -Wextra -I rocko_env/rocko_env/hardware/sim/include \
    -I rocko_env/rocko_env/hardware/sensors/icm20948/include \
    tools/sim_plant_model_test.cpp -o /tmp/sim_plant_model_test && /tmp/sim_plant_model_test
```

Success is the line `all SimPlantModel tests passed` and exit status 0 (70 checks; every check prints
`ok` or `FAIL`). It is **not a tier** and **no script runs it**, so nothing makes anyone run it. It
checks that the model computes what its specification says, with the estimated parameters. It cannot
check that the specification matches the robot.

### What a pass proves, and what it cannot show

| Tier | A pass proves | It cannot show |
| --- | --- | --- |
| 0 | The URDF expands and parses in seven expansions (balancing and tank, mock off and on, plus balancing with `use_sim_plant:=true`, without and with `use_mock_hardware:=true`, where the sim must win, plus tank mode asked for `use_sim_plant:=true`, which tank mode ignores by design), and each holds the right hardware: the two real motor plugins (and the IMU, in balancing mode) when the mock is off, exactly one `MockRocko` system when it is on, exactly one `SimPlant` system (`SimRocko`) and nothing else for the sim, and still the two real motor plugins and no `SimPlant` for the tank request; no plugin of any other kind; and the mock exports exactly the same interfaces (joints, sensors and gpio; a duplicate counts as a difference) as the real hardware, and so does the sim: 12 in balancing mode, 6 in tank mode. A pass prints 12 `ok` lines at the left margin (four more, indented, come from the config check below) and `PASS`. `sim/check_sim_config.py` also checks that `rocko_sim_controllers.yaml` equals `rocko_controllers.yaml` in everything except the `gains` maps (values compared with their type, so `100` and `100.0` differ); that in both files each controller's `gains` keys are exactly its `dof_names` and every `gains` key other than the tunable numbers (`p`, `i`, `d`, the clamps, `feedforward_gain`) is equal; that every simulator setting's default is the same in `SimPlantModel.hpp`, `rocko.urdf.xacro` and (for the settings in its `SIM_ARGUMENTS`) `rocko_sim.launch.py`, that each `<param>` in `rocko.ros2_control.xacro` is wired to the xacro arg of the same name, and that an `int` setting is written as an integer; and that the IMU fault limit the sim copies equals `ICM20948.hpp`'s. The guard hook blocks what it should and passes what it should. The git hooks in `.githooks/`, installed by copy in a throwaway repository, refuse and allow what they should | That anything compiles or runs. That the git hooks are installed in your clone |
| host unit test (not a tier) | The counting logic of `ImuFaultTracker`: it trips at exactly the consecutive limit and not before; a good read clears the run; a sensor that alternates 4 bad and 1 good reads, which never reaches the consecutive limit, trips the 10-of-20 window; the window clears after 20 good reads; the total count is never reset; one bad read in ten never trips it; a custom window limit is honoured | That `ICM20948.cpp` calls it correctly, that the limits are right for the robot, or anything about the sensor, timing or physics |
| compile-only (not a tier) | `Motor12VoltQuadEncoder.cpp` parses, every name resolves, every call matches a declared signature, and the ROS 2 headers it uses exist in Jazzy. First run, 2026-10-31, on the unmodified file: pass. Rerun on the edited file (the new `write()`, review round 2): pass. It was not made to fail on the new `write()` | That the real WiringPi signatures match: the stubs are copied by hand, so a stub that differs from the real header would hide a mistake. That it links, runs, behaves, is fast enough, or builds on a Pi |
| 1 | `rocko_interfaces` and `rocko_env` compile and link; declared dependencies install | Anything about **`Motor12VoltQuadEncoder.cpp`** (left out off-Pi: it needs WiringPi; only the compile-only check above type-checks it). Anything at runtime |
| 2 | Every controller **the launch file spawns** (the four cascade controllers in balancing mode; the diff-drive controller and joint-state broadcaster in tank mode) loads against the mock, claims interfaces the hardware layer exports, chains, activates and stays active; `Joystick.py` and `DiffDriveController.py` have not crashed. It refuses to run while another controller manager is up, so it cannot be fooled by a leftover one | **Physics.** The mock echoes each command back as a state and holds pitch at 0.0, so a controller that would drop the robot on the floor passes just the same. It does not start every controller named in the YAML, and it skips the IMU and encoder nodes |
| host unit test of the plant (not a tier) | `SimPlantModel` computes what [sim-plant.md §7.1](sim-plant.md) says: the falling pole is 8.59149 s^-1 (expected 8.5915); the fall from 2 to 35 deg with no torque takes 0.427297 s (expected 0.4273); through the 100 Hz loop with a 20 ms IMU delay it takes 0.45 s; plus the motor, delay-buffer, `write()` copy, IMU-fault and parameter-validation rules, in 70 checks | That the specification matches the robot. Every physical parameter is an estimate, so a pass says the code does what the document says, not that either is true |
| 3 (partly built) | `verify_sim.sh`: the four cascade controllers and `joint_state_broadcaster` load against `SimPlant` unchanged and go active with all 12 interfaces; with no wheel command the simulated pitch does not move (0.0000 deg measured, limit 0.01); with a 5 rad/s feedforward it moves at least 0.5 deg (it moved 0.06 s after release, by 60.23 deg in the latest run; that figure varies from run to run and only the >= 0.5 deg threshold matters, so the pitch responds to the wheels); with the motors producing no torque it passes 35 deg within 0.48 s of starting to fall (0.440, 0.432 and 0.430 s measured; the model alone gives 0.4273 s) | **Everything the model omits** (backlash, PWM aliasing, battery sag, the floor, turning; see [sim-plant.md §6](sim-plant.md)), and **everything that was estimated**: mass, centre-of-mass height, inertia, motor constants, IMU delay, wheel radius. Nothing measured. Whether any controller *holds* the simulated robot up (those scenarios do not exist), whether a sign error would be caught (S2b does not exist), and whether the real robot balances. A pass is a statement about a model |
| 4 | Whether the robot actually balances | — |

**Measured, not assumed:** in the mock the pitch controller's `feedback` is exactly `0.0` (not NaN)
and it computes every cycle — Jazzy's `GenericSystem` zero-fills non-standard state interfaces. To
see it: start `rocko.launch.py use_mock_hardware:=true` in the container, then run
`ros2 topic echo /pitch_pid_controller/controller_state --once` and read `feedback`.

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
| 0 | Break the sim config in copies, one at a time: rename a `gains` key in the sim YAML; delete a `gains` map; add `angle_wraparound: true` to one; write `update_rate: 100.0`; wire `<param name="imu_delay_ms">` to `$(arg imu_delay_max_ms)`; give `belt_ratio` a literal; write `imu_delay_ms` as `20.0` in the xacro or the launch file | `sim/check_sim_config.py` `FAIL`, exit 1, each time with a line naming the cause. Changing only `p`, `i`, `d` and the clamp numbers in the sim YAML still passes (exit 0) |
| 0 | Make the tank description hold a `SimPlant` plugin (a comment containing it, in the tank `<ros2_control>` macro); rename `struct SimPlantParams` so the checker cannot read it | `verify_urdf.sh` `FAIL`, exit 1, with `got sim=1` in the `rocko_tank_mode sim=true` line; and `sim config check could not run (exit 2; this is not drift)` |
| 3 | Give `released_problem` (in `verify_sim.sh`) a hand-made launch log whose release stamp is 9.5 s, 2.5 s or -1.2 s (after) before the last sample in `metrics.json`; and a `metrics.json` with no `last_stamp_s` | 9.5 s: no problem. 2.5 s and -1.2 s: `less than 3 s before the probe's last sample`. No `last_stamp_s`: says so. Run on the function alone, not in a full `verify_sim.sh` run |
| host unit test of the plant | Weaken four of the `invalid_reason()` checks in a copy of `SimPlantModel.hpp` (allow `imu_delay_max_ms` up to 100000; drop the `battery_voltage_V`, `body_depth_m` and mass-matrix checks) | 5 of the #17 checks go red; the real header passes all 70 |

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
- **The controller stop logic has no automated test.** `AddFeedforwardController`'s NaN-to-zero rule,
  3-cycle hold, refusal of non-finite commands and `on_deactivate` stop (finding H-7) are built by
  Tier 1 and loaded by Tier 2, but nothing checks what they do. A gtest was declined in review round 2;
  the plan is that Tier 3 fault scenarios will cover it. Those scenarios do not exist yet (Tier 3 is
  only partly built), so nothing covers it.
- **The IMU and encoder nodes are not exercised.** The mock launch skips `ICM20948.py` and
  `QuadEncoders.py` (they need I²C and GPIO), so the C++ plugins that call them are untested here.
  `ICM20948.cpp` is compiled by Tier 1, but the mock replaces that plugin, so no tier runs its code,
  including the C-2 failure path. Only the counting rules of `ImuFaultTracker` have a test (the
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
- **The impulse and IMU-fault parameters are not exercised end to end.** `impulse_time_s`,
  `impulse_angular_velocity_deg_per_s` and the `imu_fault_*` parameters are covered by the host unit
  test of the model but no launch has run them, and `verify_sim.sh` has no run that uses them. The
  recovery-from-an-impulse scenario and the C-2 downstream check do not exist yet.
- **The sim has one hardware owner; the robot has two.** `SimPlant` is one component that provides the
  wheels and the IMU. When its IMU rule trips and `read()` returns `ERROR`, the whole plant (wheels
  included) stops with it. On the robot the IMU and the motor plugin are separate components, and the
  motor plugin may keep driving its last command after the IMU fails (finding H-7). So this sim cannot
  show the H-7 behaviour, and a "safe" outcome after an injected IMU fault in the sim would not carry
  over.
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
  (controller names, types, `dof_names`, interfaces, `update_rate`) or any `gains` key other than the
  tunable numbers differs from `rocko_controllers.yaml`, or if a controller's `gains` keys are not
  its `dof_names`. It does not look at the gain numbers, which are allowed to diverge, so it cannot say
  whether sim gains are sensible.
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
- **The sim's IMU-fault branch (C-2) and the impulse have not been run end to end.** The code and the
  unit test cover them; no scenario launches them yet (stage 4).
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
