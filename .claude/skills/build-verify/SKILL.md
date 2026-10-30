---
name: build-verify
description: Build ROCKO and climb the verification ladder (static, build, mock, simulation, robot). Use whenever asked to build, compile, "check this builds", run colcon, work in the Docker dev container, run the mock, or after implementing any code change. Says what each tier proves, what it cannot, and exactly which commands to run.
---

# Building and verifying ROCKO

This skill is the *procedure* for the [verification ladder](../../../docs/architecture/verification-ladder.md).
That page is where each tier's status lives; do not restate it here. Run every tier that can run,
record each command and its result in the run journal, and report the highest tier you actually
reached.

## Know which target you are on before you type anything

```bash
echo "$ROS_DISTRO"                      # must print: jazzy
uname -m                                # aarch64 on a Pi (and on an Apple-silicon container!), x86_64 elsewhere
ls /usr/lib/libwiringPi.so* 2>/dev/null || echo "no WiringPi -> use -DROCKO_SKIP_WIRINGPI=ON"
```

`uname -m` does **not** tell a Pi from a container on an M-series Mac; both say `aarch64`. WiringPi
being present does.

| Target | Tiers it can run | Notes |
| --- | --- | --- |
| Docker dev container | 0, 1, 2 | Builds without the motor plugin. This is where an agent works |
| Raspberry Pi (Ubuntu 24.04, Jazzy, WiringPi) | 0, 1 (the only place the motor plugin builds), 2, and 4 | A human's machine; `ros2 launch` here drives real motors. Not run by an agent |
| Simulation | 3 | See the ladder for whether it exists; `bootstrap-sim` builds it |

## Before any tier (Docker target) — is the container looking at your files?

```bash
./check_container_mount.sh     # on the laptop, from the repo root of your checkout
```

There is one dev container per laptop and it shows whichever checkout last ran `up -d`. In another
checkout (a second clone, a git worktree) the tiers below still run and still pass, but on the *other*
folder's files: the same mistake as a sensor that cannot fail. Run it at the start of a session and
whenever you change checkout. Exit 0 `ok`: carry on. Exit 1 `FAIL`: it prints the fix and the
rebuild-from-clean command; `ok` does not say the build output is yours, so after a switch clean
the build before Tier 1. Exit 2: could not tell. What it proves, what it cannot, and how it was
tested are in the [guide](../../../docs/guides/verify-in-container.md) and the
[ladder's blind spots](../../../docs/architecture/verification-ladder.md).

## Tier 1 — the build

This repo is **not** a colcon workspace. It lives inside one, at `<ws>/src/ROCKO-env`. Build from the
workspace root, never from `src/`.

```bash
source /opt/ros/jazzy/setup.bash
cd "${ROS2_WS:-$HOME/ros2_ws}"   # Pi: ~/ros2_ws · dev container: /ros2_ws (compose sets ROS2_WS)
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` is required by the workflow: Python nodes, launch files, xacro and the gain
YAMLs are linked rather than copied, so editing them needs no rebuild. C++ and anything in
`rocko_interfaces` still does.

**Off the Pi, WiringPi does not exist**, so the build must opt out of the motor plugin:

```bash
colcon build --symlink-install --cmake-args -DROCKO_SKIP_WIRINGPI=ON
```

Without that flag the build fails with a message saying so, deliberately: a robot must never be
built without its motor driver by accident. With it, `rocko_env` builds but **leaves out
`Motor12VoltQuadEncoder.cpp`**, so a green build says nothing about that file.

Two consequences worth knowing:

- **CMake remembers the flag** in `build/rocko_env/CMakeCache.txt`. `build_ros.sh` therefore always
  passes it explicitly (`ON` or `OFF`), and CMake stops with `ROCKO_SKIP_WIRINGPI=ON, but WiringPi IS
  installed here ... almost certainly a stale CMake cache` if it is `ON` where WiringPi exists. On a
  Pi, after any skip build, `rm -rf build/rocko_env`.
- **A skip build cannot run the real motors.** Launching it *without* `use_mock_hardware:=true`
  prints `Could not create class of type rocko_env::Motor12VoltQuadEncoder` and then `Could not load
  and initialize hardware`, and the spawners time out. That is the plugin being absent, not a bug.

`build_ros.sh` at the repo root does all of the above plus `sudo apt-get update` first, and always
passes `-DROCKO_SKIP_WIRINGPI` (`OFF` unless `ROCKO_SKIP_WIRINGPI` is set; compose sets it to `ON` in
the container). It does not `source install/setup.bash` at the end. Success ends with `Finished <<<
rocko_env`, `Summary: 2 packages finished` and `Done!`, exit 0. **A failed build prints `ERROR:
colcon build failed` and exits 1**; it used to say `Done!` and exit 0, which made Tier 1 unable to
fail by exit status. In the container run it as `cd /ros2_ws && src/ROCKO-env/build_ros.sh` (see the
stale-copy warning below). Two things look alarming and are not:

- `2 packages had stderr output` — CMake warnings (the skip notice for `rocko_env`, and "manually
  specified variable not used" for `rocko_interfaces`) land on stderr.
- `rosdep` installing packages you never asked for — it reads `package.xml`. If a controller package
  is missing at launch, the fix is a `<exec_depend>` in `package.xml`, not a manual `apt install`.

**Read the warnings.** In real-time C++ they are often the finding. The first off-Pi build turned up
a dropped `set_value()` result (finding M-5) this way.

## Tier 0 — static checks

Needs the build first: the xacro files resolve `$(find rocko_env)` through the installed package.

```bash
docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_urdf.sh
```

It expands balancing and tank mode, each with the mock off and on, runs `check_urdf` on every
result, **and asserts what is inside**: `mock=false` holds the real plugins
(`<plugin>rocko_env/Motor12VoltQuadEncoder</plugin>`, and `rocko_env/ICM20948` in balancing mode) and
no `mock_components`; `mock=true` holds exactly one `MockRocko` system and no real motor plugin.
Success is six `ok` lines (one per combination, plus one interface-parity line per model) and `PASS`. The assertions are the point: `check_urdf` alone passes
whichever block was selected, so a swapped `xacro:if`/`xacro:unless` would otherwise pass.

When you change that script, break it on purpose to see it go red for the right reason (swap the
mock block's `xacro:if` for `xacro:unless`, run it, restore the file). It once failed a correct
description because it counted lines containing a plugin *name* and an XML comment mentioned one;
it now counts whole `<plugin>` tags.

On the host (no ROS needed): `.claude/hooks/test-guard-protected-files.sh` — the guard hook's own
tests, which must stay green, and `bash -n` any shell script you touch, `python3 -m py_compile`
any Python. The hook tests assert the exit status as well as the decision, because a hook that
crashes lets the edit through. If you changed `.githooks/`, run `.githooks/test-git-hooks.sh` and
`.githooks/mutation-check.py` too (the hooks are not installed in this clone until a person does it:
[proposal 0002](../../../docs/proposals/0002-unattended-run-settings.md), item E).

## Tier 2 — the mock

```bash
docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_mock.sh        # balancing
docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_mock.sh tank   # tank mode
```

Success prints `PASS (balance, mock hardware): all controllers active, ...` and exits 0. On failure
it prints `FAIL`, what was missing, the controller and interface lists, and the launch errors, and
exits 1; exit 2 means it could not run (no build yet, a bad argument, or a controller manager is
already running).

What a pass means: every controller the launch file spawns (the four cascade controllers in
balancing mode; the diff-drive controller and the joint-state broadcaster in tank mode) loaded,
claimed interfaces the hardware layer exports, chained, activated and *stayed* active for a few
seconds, and `Joystick.py` / `DiffDriveController.py` had not crashed. It does not start every
controller named in the YAML: `joint_state_broadcaster` is declared there but only tank mode spawns
it. In the mock, pitch reads `0.0` (measured: the pitch controller's `feedback` is `0.0`, not NaN,
and it computes every cycle). What a pass does **not** mean: the
mock has no physics. Each command is echoed back as a state and pitch stays 0.0, so a controller that
would drive a real robot into the floor passes just the same. Never describe a Tier 2 pass as
"works" or "balances".

Things that have gone wrong before, so you do not rediscover them:

- `ros2 control ...` **blocks forever** with no controller manager. Wrap it in `timeout`.
- ROS's `setup.bash` reads unset variables, so a script under `set -u` dies at `source`.
- A background job in a non-interactive script **ignores SIGINT**; stop `ros2 launch` with SIGTERM
  (the script uses its own process group and kills the lot).
- `pkill -f 'ros2 control'` run through `bash -c '...'` kills its own shell, because the pattern
  appears in the shell's command line.
- The URDF is passed to `robot_state_publisher` as a *string* parameter. Without that, `launch_ros`
  parses it as YAML and a `: ` in any xacro comment breaks the launch.

**Make the verifier fail once.** When you change `verify_mock.sh`, the mock block or the YAML,
break the thing on purpose — delete a `<state_interface>` from the `MockRocko` block — and confirm
`FAIL` and exit 1, then restore the file. A check you have only ever seen pass is not yet a check.

## Tier 3 and 4

Tier 3: see the ladder for whether it exists, and `bootstrap-sim` to build it. Tier 4 needs a robot and a person. You cannot run
it, so end every behavior change with the procedure for it (tethered, what to launch, what success
looks like, when to abort), or a link to the proposal that holds it.

## Docker dev container

```bash
# macOS or Linux — from the repo root (add -f docker/compose.mac.yml for XQuartz GUI forwarding)
docker compose -f docker/compose.yml up -d
docker compose -f docker/compose.yml exec ros2 bash

# Windows — from inside WSL2, Docker Desktop WSL2 integration on
docker compose -f docker/compose.yml -f docker/compose.windows.yml up -d
docker compose -f docker/compose.yml -f docker/compose.windows.yml exec ros2 bash
```

The Mac override only adds XQuartz display forwarding and bind-mounts `~/.Xauthority`; for builds and
the mock you do not need it.

The repo is bind-mounted at `/ros2_ws/src/ROCKO-env`; `build/`, `install/`, `log/` live in a named
volume so they neither dirty `git status` nor crawl on a Mac. `docker compose down -v` wipes it for
a clean rebuild.

**`/ros2_ws/build_ros.sh` can be a stale copy.** Compose also mounts that one file on its own
(`../build_ros.sh:/ros2_ws/build_ros.sh`). A file mounted by itself is pinned when the container
starts, so if an editor or tool *replaces* the host file while the container runs, the container
keeps an older, even truncated, version. This was observed: after `build_ros.sh` was edited, the
container's copy had a different checksum and stopped mid-statement, so a test of the new behavior
quietly ran the old script. The repo directory mount is live, so **run
`/ros2_ws/src/ROCKO-env/build_ros.sh`** (it works from any directory; it `cd`s to `$ROS2_WS`), or
recreate the container. Not fixed in `docker/compose.yml` because it is the maintainer's file and
the README tells people to run `./build_ros.sh`; the single-file mount looks redundant now that the
whole repo is mounted.

**The container's writable layer is thrown away when compose recreates it** (any change to its
config, or `down` then `up`). Anything installed with `apt` by hand disappears; anything declared in
`package.xml` comes back with `build_ros.sh`. That is a feature: it keeps the declared
dependencies honest.

**Where `build_ros.sh` builds.** The script `cd`s to `$ROS2_WS`, defaulting to `~/ros2_ws`. That
default is right on the Pi but wrong in the container: the workspace volume is mounted at
`/ros2_ws` and the user is `rocko`, so `~` is `/home/rocko`, which has no `ros2_ws`. Before this was
fixed, the `cd` failed, the script (which has no `set -e`) carried on, and `rosdep`/`colcon` ran
in whatever directory you launched it from — correct only when that happened to be `/ros2_ws`.
`docker/compose.yml` now sets `ROS2_WS=/ros2_ws`, and the script exits with an error if the
workspace directory is missing.

Use compose rather than plain `docker run`. The image ships an empty `/ros2_ws`: `src/ROCKO-env`
and `build_ros.sh` both arrive through compose's bind mounts, and `ROS2_WS` and
`ROCKO_SKIP_WIRINGPI` through its `environment:`. A bare `docker run` gets none of the four. The
equivalent of what compose does (**not run**; the compose path is the one that has been):

```bash
# from the repo root
docker run --rm -it -e ROS2_WS=/ros2_ws -e ROCKO_SKIP_WIRINGPI=ON \
  -v "$PWD:/ros2_ws/src/ROCKO-env" \
  -v "$PWD/build_ros.sh:/ros2_ws/build_ros.sh" \
  rocko-env-dev:latest ./build_ros.sh
```

Without compose's named `workspace` volume, `build/`, `install/` and `log/` land in the container's
writable layer and vanish with `--rm`.

To check a running container: `docker compose ... exec ros2 bash -c 'echo $ROS2_WS $ROCKO_SKIP_WIRINGPI'`
should print `/ros2_ws ON`. If it prints anything else, the container was created before those
variables were added and still has the old environment — recreate it with `docker compose ... up -d`.

**What was and wasn't verified:** the evidence for each tier, and what was not run (nothing on a Pi),
is in the [verification ladder](../../../docs/architecture/verification-ladder.md) and the
[run journal](../../../docs/journal/2026-10-27-ai-forward-bootstrap.md). The `docker run` form above
was not run.

## Reporting

State which tiers you ran and the result of each, as a table in the run journal. If a tier did not
run, write "not run" and why. If you touched the motor plugin, write that it is uncompiled here. Do
not let silence imply it passed.
