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
> modes. The `pid_controller` version was not read again.

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
| 0 | Static | `verify_urdf.sh` (container); `.claude/hooks/test-guard-protected-files.sh`, `.githooks/test-git-hooks.sh` (laptop) | **Works.** Needs a Tier 1 build first (`verify_urdf.sh`). The git hooks it tests are not installed in this clone until [proposal 0002](../proposals/0002-unattended-run-settings.md) item E |
| 1 | Build | `build_ros.sh` (container) | **Works** off-Pi, *without* the motor plugin; exits 1 when the build fails. **The Pi build (flag `OFF`) has not been run** since the CMake change |
| 2 | Mock | `verify_mock.sh`, `verify_mock.sh tank` (container) | **Works**; shown to fail on deliberate breaks |
| 3 | Simulation | — | **Not built.** [ADR 0003](../decisions/0003-simulation-target.md) is still `Proposed` |
| 4 | Robot | A person, tethered | **Human only.** Not something an agent can run or observe |

Before any tier that uses the container, `check_container_mount.sh` (run on the laptop) says whether
the container is looking at your checkout. It is a precondition, not a tier: it proves one thing and
leaves out others (see the blind spots).

### What a pass proves, and what it cannot show

| Tier | A pass proves | It cannot show |
| --- | --- | --- |
| 0 | The URDF expands and parses in all four modes (balancing and tank, mock off and on), and each holds the right hardware: the two real motor plugins (and the IMU, in balancing mode) when the mock is off, exactly one `MockRocko` system when it is on; no plugin of any other kind; and the mock exports exactly the same interfaces (joints, sensors and gpio; a duplicate counts as a difference) as the real hardware: 12 in balancing mode, 6 in tank mode. The guard hook blocks what it should and passes what it should. The git hooks in `.githooks/`, installed by copy in a throwaway repository, refuse and allow what they should | That anything compiles or runs. That the git hooks are installed in your clone |
| 1 | `rocko_interfaces` and `rocko_env` compile and link; declared dependencies install | Anything about **`Motor12VoltQuadEncoder.cpp`** (left out off-Pi: it needs WiringPi). Anything at runtime |
| 2 | Every controller **the launch file spawns** (the four cascade controllers in balancing mode; the diff-drive controller and joint-state broadcaster in tank mode) loads against the mock, claims interfaces the hardware layer exports, chains, activates and stays active; `Joystick.py` and `DiffDriveController.py` have not crashed. It refuses to run while another controller manager is up, so it cannot be fooled by a leftover one | **Physics.** The mock echoes each command back as a state and holds pitch at 0.0, so a controller that would drop the robot on the floor passes just the same. It does not start every controller named in the YAML, and it skips the IMU and encoder nodes |
| 3 | *(when built)* controller behavior against modeled dynamics | Everything the model omits: backlash, PWM aliasing, battery sag, IMU latency |
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

- **The motor plugin is never compiled off-Pi.** A change to `Motor12VoltQuadEncoder.cpp` gets Tier 0
  and a reviewer's eyes, nothing more, until it is built on a Pi.
- **The IMU and encoder nodes are not exercised.** The mock launch skips `ICM20948.py` and
  `QuadEncoders.py` (they need I²C and GPIO), so the C++ plugins that call them are untested here.
- **Timing is not measured.** Container and mock timings say nothing about the Pi's 10 ms budget.
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
  run outside `auto` mode, where a classifier can approve a call whose rule does not match. The git
  hooks in `.githooks/` are tested on a throwaway repository and are inactive in this one until that
  proposal's item E is done.
- **The Pi build has not been run.** Nothing here compiles the motor plugin against real WiringPi or
  checks that the `OFF` path produces a working `librocko_env.so`.
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
- **The RViz configs are never installed** (finding **L-4**), so three launch files cannot find them.
- **Nothing here covers x86_64.**

## Keeping this page true

Change this page in the **same commit** as anything that changes a tier: a new check, a tier built, a
blind spot closed or opened. If a status here disagrees with what a command prints, the command is
right and this page is a bug.
