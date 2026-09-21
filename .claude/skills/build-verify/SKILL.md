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
cd ~/ros2_ws
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

`build_ros.sh` at the repo root wraps the sequence above and is what the container uses.

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

**Unverified suspicion:** `docker/compose.yml` (startup command) and `build_ros.sh` both
`cd ~/ros2_ws`, but the container's workspace volume is mounted at `/ros2_ws` and the user is
`rocko`, so `~` is `/home/rocko`. If `/home/rocko/ros2_ws` doesn't exist, `build_ros.sh` runs
`rosdep`/`colcon` in the wrong directory. Not yet tested in a running container — check with
`docker compose ... exec ros2 bash -c 'ls -d ~/ros2_ws /ros2_ws'` before relying on the script.

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
