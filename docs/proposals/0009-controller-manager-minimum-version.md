# Proposal 0009 — what minimum `controller_manager` version should the build require?

- **Status:** Open
- **Gate:** version pin ([ADR 0001](../decisions/0001-version-lock.md): `ros2_control` ≥ 4.0.0)
- **Raised:** 2026-11-04, built on `77664f4`, run journal:
  [final report](../journal/2026-11-04-final-report-stage-4-6.md). The question comes from
  [next-steps](../next-steps.md), "Decisions waiting on the maintainer".

## The change

Nothing is edited. The question is whether `rocko_env/CMakeLists.txt` (line 26)

```cmake
find_package(controller_manager 4.0.0)
```

and its `FATAL_ERROR` message, and the `≥ 4.0.0` row of ADR 0001, should name a higher version.

## Why

The check is meant to turn "wrong ROS version" into a clear build failure rather than a runtime
surprise (ADR 0001). The code written on this branch uses newer `ros2_control` APIs than 4.0.0 may
have:

- `set_value()` on a command interface **returning `bool`** (`AddFeedforwardController.cpp`, lines
  271 and 443; finding M-5 depends on it);
- `get_optional()` on a state interface (same file, lines 347 and 495);
- the `realtime_tools` header `realtime_tools/realtime_buffer.hpp` (`AddFeedforwardController.hpp`
  line 29; finding L-3 moved to it from a deprecated header).

Whether a Jazzy `controller_manager` older than the one in the dev container lacks any of these is
**unknown**: it is the reviewer's concern and it is plausible, and this page has not checked it
against upstream. If one does lack them, then today a Pi with an older Jazzy passes the CMake check
and then fails inside the compiler with an error about a missing function, which is the surprise the
check exists to prevent.

## Evidence (and what is not known)

| Fact | Where it comes from |
| --- | --- |
| `controller_manager` **4.48.1** in the dev container (`ros:jazzy`, arm64) | observed in the log of check C2 long, 2026-11-02 ([ladder](../architecture/verification-ladder.md), [journal](../journal/2026-10-31-sim-plant-and-critical-fixes.md)) |
| `pid_controller` 4.42.1 (that is `ros2_controllers`, a different repo and version) | the ladder |
| `hardware_interface` version | **unknown**, never recorded |
| `realtime_tools` version | **unknown**, never recorded |
| any version on the Raspberry Pi | **unknown**, never recorded |
| which version first has `set_value()` returning `bool`, `get_optional()` and the `.hpp` header | **unknown**; not looked up |

The container always installs the newest Jazzy packages, so it can never show that an older version
fails. Record the facts, in the container and on the Pi:

```bash
source /opt/ros/jazzy/setup.bash
ros2 pkg xml controller_manager | grep '<version>'
ros2 pkg xml hardware_interface | grep '<version>'
ros2 pkg xml realtime_tools | grep '<version>'
apt-cache policy ros-jazzy-controller-manager ros-jazzy-realtime-tools
```

In the container, from the repo root on the laptop, run the same four lines with
`docker compose -f docker/compose.yml exec -T ros2 bash -c '...'` around them. Paste the output
under *Decision* with the date and the machine.

## Options

| Option | What it is | Cost |
| --- | --- | --- |
| A. Leave `4.0.0` | The status quo | The check does not protect the new code; an old Pi fails with a compiler error |
| B. Raise to the tested version | `4.48.1`, the only version this code has been built against | Honest and exact, but may be stricter than needed; a Pi that has not run `apt upgrade` fails at once, with a clear message |
| C. Raise to the lowest version known to build | After the Pi's versions are recorded: if the code builds on the Pi, use the lower of the Pi's and the container's | Needs one real build on the Pi; the number rests on evidence rather than a guess |
| D. Check the features, not the number | A small CMake test that compiles a call to `set_value()` and `get_optional()` | Names the missing feature, but it is new build logic with one user; against the "simplest thing" rule |

**Recommendation: C, after the versions are recorded; until then A.** Do not pick a number from
upstream release notes recalled from memory. If the Pi cannot build the code, that is itself the
answer (B, with a deliberate Pi upgrade). If accepted, the edit is one line in `CMakeLists.txt`, the
`FATAL_ERROR` text, the ADR row and the line in `CLAUDE.md` §3 that repeats the pin.

## Risk

Raising the minimum makes some existing Pis fail to build until they upgrade; that is the point,
but it blocks a team whose Pi is not reachable. Leaving it hides the problem until compile time.
Nothing here affects what a built robot does.

## Test for a human

1. Run the commands above on the container and on the Pi; record them here.
2. If a higher minimum is accepted: make the edit, then `build_ros.sh` in the container (Tier 1)
   must pass:

   ```bash
   docker compose -f docker/compose.yml exec -T ros2 bash -c 'cd /ros2_ws && src/ROCKO-env/build_ros.sh'
   ```

   and build once on the Pi (`colcon build --symlink-install` from the workspace root, after
   sourcing `/opt/ros/jazzy/setup.bash`). Success: both exit 0. To see the check work, temporarily
   set the number above the installed version in a scratch copy and confirm CMake stops with the
   `FATAL_ERROR` message.

## Decision

(Human fills in: date, who, the recorded versions, A/B/C/D, why.)
