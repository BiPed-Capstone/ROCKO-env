# Rename the IMU plugin from `ICM20948` to `BNO055` (finding L-2, next-steps row 1.6)

- **Date / commit:** 2026-11-26, built on `d0588ae` (this entry cannot name the commit that adds it)
- **Branch:** `ai-forward`
- **Environment:** laptop (macOS) + the Docker dev container (Jazzy); no robot

## Goal and success criteria

The IMU hardware plugin was named for an Adafruit ICM20948, a different IMU part; the part fitted is a
Bosch BNO055 (finding L-2). A student reading the xacro to learn which sensor is on the robot got the
wrong answer. Rename the class, plugin name, folder and files to `BNO055`. Done when:

1. `tools/run_ladder.sh --clean` passes (Tiers 0 to 3, host tests, hook tests, doc link scan).
2. `verify_urdf.sh`'s plugin-name check (`tools/check_plugin_names.py`) passes with `rocko_env/BNO055`,
   and goes red on a deliberate mismatch.
3. `git grep -i icm20948` hits only history (journals, proposals, dated status lines, analyses written
   before the change, ADR 0004), one sentence in CLAUDE.md §10 giving the old name, and the
   supervisor test that checks the removed `icm20948_node` is no longer expected.
4. Names only: no gain, limit, pin, offset or other value in a xacro or YAML changes.

## Plan

One mechanical change, done with `git mv` and a fixed list of substitutions, then a hand pass over
what remained. Tier 0 proves the three names agree (xacro, plugin XML, `PLUGINLIB_EXPORT_CLASS`);
Tier 1 proves the renamed class and moved include folder compile and install. Neither loads the real
plugin: only the Pi does. Risks: a stale `install/` on the Pi still holding the old XML and library
(so the owner builds from clean), and a dated record rewritten by the substitution (checked by hand:
the 2026-11-26 row in the ladder's "seen to fail" table was restored to what the check printed then).

New names: class `rocko_env::BNO055`, plugin `rocko_env/BNO055`, folder
`rocko_env/rocko_env/hardware/sensors/bno055/`, files `BNO055.cpp` and `include/rocko_env/BNO055.hpp`,
header guard `ROCKO_ENV_BNO055`, logger `BNO055` (log lines now start `[BNO055]`). All-caps, as the
old name was, because it is a part number. The helper headers (`Bno055UartProtocol.hpp`,
`ImuFaultTracker.hpp`, `ImuSnapshotCheck.hpp`) keep their names; they already fit. The ros2_control
component is still called `Gyro`, and every interface name is unchanged, so no controller or YAML is
touched.

## What changed

- `rocko_env/rocko_env/hardware/sensors/icm20948/` → `bno055/`; `ICM20948.cpp`/`.hpp` → `BNO055.cpp`/`.hpp`
  (moved with `git mv`, so history follows). Inside: class name, constructor/destructor, guard, logger;
  comments that named the removed Python node by its file now say "the removed Python IMU node".
- `rocko_env_hardware.xml`, `rocko.ros2_control.xacro` (the `<plugin>` line and a commented-out block):
  the plugin name. `git diff` on the xacro shows only those two lines.
- `rocko_env/CMakeLists.txt`: the source, include and install paths.
- `verify_urdf.sh`: the expected `<plugin>rocko_env/BNO055</plugin>` in both counts.
- `tools/check_plugin_names.py` (docstring example), `tools/run_ladder.sh` (include path),
  `sim/check_sim_config.py` (the header it reads; `ICM_HEADER` → `IMU_HEADER`), the four host tests'
  build lines, `SimPlant.cpp` and `SimPlantModel.hpp` (comments), `rocko.launch.py` (a comment).
- Docs: CLAUDE.md §6 (critical-path list) and §10 (the naming note, now saying what it is and what it
  was); the `build-verify` and `control-loop-change` skills; README (glossary, folder tree);
  architecture pages (`README`, `imu-i2c-read`, `sim-plant`, `verification-ladder`); the
  `verify-in-container` guide; next-steps (row 1.6 done in code, the file pointers in the tunables
  list); finding L-2 and `bno055-improvements.md` Naming get a dated status line. Older journals,
  proposals, ADR 0004 and the bodies of the 2026-09-20 analyses keep the old name: they describe the
  code as it was.

## Verification

| Tier | Command | Result |
| --- | --- | --- |
| precondition | `./check_container_mount.sh` | `ok`: the container sees this checkout |
| 0 to 3 + host | `tools/run_ladder.sh --clean` | **RESULT: PASS** on every step: Tier 1 build (0 compiler warnings), Tier 0 `verify_urdf.sh` (`ok    plugin names: rocko_env/BNO055, rocko_env/Motor12VoltQuadEncoder, rocko_env/SimPlant each match the plugin XML and a C++ class exported from a built source`; `ok    IMU fault limit equal in BNO055.hpp and SimPlantModel.hpp`), Tier 2 mock balance and tank, `verify_supervisor.sh`, the compile-only motor-plugin check, Tier 3 `verify_sim.sh` (S1 S2a H4 C3 C2 H6 FF), `verify_tools.sh`, scenarios (U I V K S2b AB), every host test, the hook tests, the sim config drift check, the doc link scan |
| 0, red check | In the real tree, one break at a time, each restored from a copy before the next: (1) the xacro's `<plugin>` put back to `rocko_env/ICM20948` (a rename that missed the URDF); (2) the plugin XML's `type` put back to `rocko_env::ICM20948` (a rename that missed the XML); `verify_urdf.sh` after each | (1) `FAIL`, exit 1: `rocko_env/ICM20948: no <class name="rocko_env/ICM20948"> in rocko_env_hardware.xml`, and the real-hardware count `imu=0`. (2) `FAIL`, exit 1: `rocko_env/BNO055: rocko_env_hardware.xml says type rocko_env::ICM20948, but no PLUGINLIB_EXPORT_CLASS(rocko_env::ICM20948, ...)`. Restored: `git diff` on both files shows only the rename; `verify_urdf.sh` `PASS`, exit 0 |
| names only | `git diff d0588ae -- '*.yaml' '*.xacro'` | only the `<plugin>` line and the commented-out `<sensor name=...>` line in `rocko.ros2_control.xacro`; no YAML |
| leftovers | `git grep -n -i icm20948 -- ':!docs/journal' ':!docs/proposals'` | Outside journals and proposals: the CLAUDE.md §10 sentence; `tools/supervisor_state_machine_test.py:250` (checks the removed `icm20948_node` is not expected); `ICM20948.py` or `Icm20948Data` in sentences dated before 2026-11-26 (`imu-i2c-read`, `sim-plant`, `sim-to-real-gap`, `verification-ladder` rows of 2026-11-03 and 2026-11-26, `measure-inertia`, next-steps rows 1.1, 1.6 and the tunables list); ADR 0004; the 2026-09-20 analyses (`balance-loop-findings`, `bno055-improvements`), which carry a dated status line for the rename. No current code or instruction uses the old name |

`BNO055.cpp` is compiled by Tier 1 (only the motor plugin is left out off the Pi), so the renamed class
builds. Nothing here opens the I2C bus: no tier runs the real plugin.

## Review rounds

| Round | Reviewer | Findings | Fixed / declined (why) |
| --- | --- | --- | --- |
| 1 | `realtime-auditor` (mandatory: §6 files) | clean: only names, guard, logger string and comments change; no body of `on_init`/`on_configure`/`read()`, no constant, no xacro parameter or YAML | — |
| 1 | `ros2-control-reviewer` | rename clean in all five places (XML name and type, export, class, xacro, CMake source/include/install), checks and tests follow it; nothing in `pi/`, `docker/` or other launch files names the old plugin. One finding: the journal's result placeholders were unfilled | fixed (filled after the ladder) |
| 1 | `explainability-reviewer` | 2 low: CLAUDE.md §10 did not say why the class is all-caps beside PascalCase helper headers, nor what the old name was | fixed: §10 now says both |
| 1 | `docs-reviewer` | 1 medium: the `IMU read timing` line comes every 3000 reads (about 30 s), not "every few seconds"; 1 low: the Pi procedure lacked `cd ~/ROCKO-env` and a fresh `pi/rocko_shell.sh` after the build. Asked whether the `ready` line's pitch includes the offset | fixed; confirmed it does (`ImuSnapshotCheck.hpp:84` adds `pitch_offset_deg` before the value is stored) |

The fixes after round 1 touch only CLAUDE.md and this journal, so the ladder was not rerun; the doc link scan was (below).

## Not verified — needs a human

Whether the renamed plugin loads on the Pi: pluginlib finds it by the name in the installed
`rocko_env_hardware.xml` and the class in the installed library, and off the Pi the mock or the
simulator stands in for it. No behavior changed, so no tethered test is needed beyond seeing it start.

On the Pi, in `~/ROCKO-env`, as your user:

1. Stop the boot service if it runs (`sudo systemctl stop rocko.service`; the guide's check then prints
   `running now: inactive`), then pull and build **from clean**, so no old `ICM20948` library, header or
   plugin XML survives in `install/`:
   ```bash
   cd ~/ROCKO-env
   ```
   ```bash
   git pull
   ```
   ```bash
   rm -rf build install log
   ```
   ```bash
   ./build_ros.sh
   ```
   Success: the build ends without errors (the motor plugin is built here, unlike in the container).
2. With the robot on its stand or held, wheels free, launch the supervisor:
   ```bash
   pi/launch.sh robot
   ```
3. Switch to balance mode from the gamepad or the dashboard, as usual. Watch the launch output:
   - Success: a line from `[BNO055]` saying `BNO055 ready: pitch ... deg, roll ... deg, pitch rate ... deg/s`,
     then an `IMU read timing: ...` line about every 30 s (one per 3000 reads at 100 Hz), and the controllers reach `active`.
   - Failure that this change would cause: an error that the hardware class `rocko_env/BNO055` (or
     `rocko_env/ICM20948`) cannot be found or loaded, and the launch stops. The likely cause is a stale
     `install/`: stop, repeat step 1 from clean. If it persists, send the output back; do not edit the plugin XML on the Pi.
4. Hold the robot flat and level (the posture used for the 2026-11-25 offset measurement). The `pitch`
   in the `BNO055 ready` line, and `pitch` for `left_wheel_joint` in `ros2 topic echo /dynamic_joint_states`
   from a `pi/rocko_shell.sh` opened after the build (an older one does not see the new `install/`), should read about 0 deg (both include `pitch_offset_deg`; within a degree; the offset is -2.5 deg since
   2026-11-25). A value near +2.5 or +10 means the offset is not applied: stop and report it.
5. Abort at any point (Ctrl-C) if the wheels move unexpectedly or the robot is not held.

## Reproduce

```bash
git checkout ai-forward
```
```bash
./check_container_mount.sh
```
```bash
tools/run_ladder.sh --clean
```
