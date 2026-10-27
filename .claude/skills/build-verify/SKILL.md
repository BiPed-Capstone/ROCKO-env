---
name: build-verify
description: Build ROCKO and verify a change compiles on every target that can build it. Use whenever asked to build, compile, "check this builds", run colcon, work in the Docker dev container, or after implementing any code change. Covers the three targets (Docker container, Raspberry Pi, simulation) and what each one can and cannot build.
---

# Building and verifying ROCKO

## Know which target you are on before you type anything

```bash
# Which distro is sourced?
echo "$ROS_DISTRO"                      # must print: jazzy
# Am I on a Pi?
uname -m                                # aarch64 on the Pi, arm64/x86_64 on a laptop
# Is WiringPi present? (decides whether rocko_env can link)
ls /usr/lib/libwiringPi.so* 2>/dev/null || echo "no WiringPi -> rocko_env will NOT link"
```

| Target | Builds `rocko_interfaces` | Builds `rocko_env` | Runs the robot |
| --- | --- | --- | --- |
| Docker dev container | yes | **no** — WiringPi is Pi-only | no |
| Raspberry Pi (Ubuntu 24.04, Jazzy, WiringPi installed) | yes | yes | yes |
| Simulation (`gz_ros2_control`) | — | — | **does not exist yet** |

This is a *hardware* limit, not an OS limit — Docker cannot route around it. Say so plainly rather
than reporting a red build as a mystery.

## The build

This repo is **not** a colcon workspace. It lives inside one, at `<ws>/src/ROCKO-env`. Build from
the workspace root, never from `src/`.

```bash
source /opt/ros/jazzy/setup.bash
cd "${ROS2_WS:-$HOME/ros2_ws}"   # Pi: ~/ros2_ws · dev container: /ros2_ws (compose sets ROS2_WS)
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

`--symlink-install` is required by the workflow: Python nodes, launch files, and the gain YAMLs are
linked rather than copied, so editing them needs no rebuild. C++ and anything in `rocko_interfaces`
still does.

Off-Pi, build only what can build:

```bash
colcon build --packages-select rocko_interfaces --symlink-install
```

`build_ros.sh` at the repo root runs the same build, plus `sudo apt-get update` first and without
the final `source install/setup.bash`. The container mounts it at `/ros2_ws/build_ros.sh` and makes
it executable but does not run it — run `./build_ros.sh` yourself.

## Docker dev container

```bash
# macOS — from the repo root
docker compose -f docker/compose.yml -f docker/compose.mac.yml up -d
docker compose -f docker/compose.yml -f docker/compose.mac.yml exec ros2 bash

# Windows — from inside WSL2, Docker Desktop WSL2 integration on
docker compose -f docker/compose.yml -f docker/compose.windows.yml up -d
docker compose -f docker/compose.yml -f docker/compose.windows.yml exec ros2 bash
```

The repo is bind-mounted at `/ros2_ws/src/ROCKO-env`; `build/`, `install/`, `log/` live in a named
volume so they neither dirty `git status` nor crawl on a Mac. `docker compose down -v` wipes it for
a clean rebuild.

**Where `build_ros.sh` builds.** The script `cd`s to `$ROS2_WS`, defaulting to `~/ros2_ws`. That
default is right on the Pi but wrong in the container: the workspace volume is mounted at
`/ros2_ws` and the user is `rocko`, so `~` is `/home/rocko`, which has no `ros2_ws`. Before this was
fixed, the `cd` failed, the script (which has no `set -e`) carried on, and `rosdep`/`colcon` ran
in whatever directory you launched it from — correct only when that happened to be `/ros2_ws`.
`docker/compose.yml` now sets `ROS2_WS=/ros2_ws`, and the script exits with an error if the
workspace directory is missing.

Use compose rather than plain `docker run`. The image ships an empty `/ros2_ws`: `src/ROCKO-env`
and `build_ros.sh` both arrive through compose's bind mounts, and `ROS2_WS` through its
`environment:`. A bare `docker run` gets none of the three, and fails differently depending on what
you leave out — no script mounted: `bash: ./build_ros.sh: No such file or directory`; script but no
repo: `rosdep` reports `given path 'src' does not exist`; script and repo but no `ROS2_WS`: the
script's own `ERROR: workspace '/home/rocko/ros2_ws' not found`. The equivalent of what compose does:

```bash
# from the repo root
docker run --rm -it -e ROS2_WS=/ros2_ws \
  -v "$PWD:/ros2_ws/src/ROCKO-env" \
  -v "$PWD/build_ros.sh:/ros2_ws/build_ros.sh" \
  rocko-env-dev:latest ./build_ros.sh
```

Without compose's named `workspace` volume, `build/`, `install/` and `log/` land in the container's
writable layer and vanish with `--rm`.

To check a running container: `docker compose ... exec ros2 bash -c 'echo $ROS2_WS; ls -d $ROS2_WS'`
should print `/ros2_ws` twice. If it prints a blank line and `.`, the container was created before
this change and still has the old environment — recreate it with
`docker compose ... up -d` (compose recreates a container whose config changed) or `down` then `up -d`.

**What was and wasn't verified.** The script's behavior was verified through `docker compose run`
with `rosdep`/`colcon` stubbed to print their working directory. It was *not* verified with a real
build, and not against a container recreated from the new compose file.

## Verifying, beyond "it compiled"

Compiling proves almost nothing about a control change. In order of strength:

1. **Clean build**, no new warnings. Read the warnings — in real-time C++ they are often the finding.
2. **Plugins are discoverable** — anything new must be in `rocko_env_hardware.xml` or
   `rocko_env_controllers.xml`, and installed by `CMakeLists.txt`.
3. **xacro expands** (catches URDF/ros2_control wiring errors without any hardware):
   ```bash
   xacro rocko_env/rocko_env/description/urdf/rocko.urdf.xacro use_mock_hardware:=false > /tmp/rocko.urdf
   check_urdf /tmp/rocko.urdf
   ```
4. **Launch-time inspection** (needs the Pi, or sim once it exists):
   ```bash
   ros2 control list_controllers              # state of each controller
   ros2 control list_hardware_interfaces      # what the hardware layer exposes
   ros2 topic hz /left_feedforward            # is the loop actually running at rate
   ```
   A controller stuck `inactive` or missing is almost always a name or interface mismatch in
   `rocko_controllers.yaml`.
5. **On-robot behavior** — you cannot do this. Write the procedure for the human instead.

## Reporting

State which targets you built and the result of each. If `rocko_env` was not built because you were
off-Pi, say that explicitly — do not let silence imply it passed.
