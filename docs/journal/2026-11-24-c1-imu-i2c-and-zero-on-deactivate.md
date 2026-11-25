# Read the IMU from C++ over I2C (finding C-1, IMU half) and make a failed zero-on-deactivate visible (row 4.14)

- **Date / commit:** 2026-11-24, built on `c6f73e7`; the commit that adds this file
- **Branch:** `ai-forward`
- **Environment:** laptop (macOS, Apple clang) for host tests; the Docker dev container (Jazzy) for Tiers 0 to 3

## Goal and success criteria

The owner asked to "implement next-steps to get closer to the robot balancing". Items 1 and 2 of the
2026-11-24 handoff list are owner measurements. The first items an agent can write are item 3
(C-1) and, from item 7, row 4.14. Asked at the start, the owner chose: **C-1's IMU half, over I2C
from C++, not the UART rewiring of proposal 0010; the encoders stay for later.**

1. `ICM20948.cpp` no longer makes a ROS service call. It reads the BNO055 over `/dev/i2c-1` in **one**
   12-byte burst per `read()` (gyro and Euler angles together, H-3), bounded by a kernel timeout.
   It allocates nothing in `read()`, logs only throttled, and holds the last good values on a failed
   read (C-2).
2. The sample check is a pure header with a host test, and the test goes red on deliberate breaks.
3. `ICM20948.py` is no longer launched or expected by the supervisor; its offset, axis and sign are
   xacro parameters with the same values (7.6 deg, axis 0, sign -1).
4. Row 4.14: `AddFeedforwardController::on_deactivate` returns `ERROR` if any zero write failed. After a
   disable, the supervisor kills the stack at once if a motion controller is in any state other than
   `active` or `inactive` (not while the stack is starting); one still `active` is asked to deactivate
   again, as before. Both are host-tested, red first.
5. The ladder stays green: Tier 0, Tier 1 (with no new warnings), Tier 2, Tier 3, and the host tests.
6. (Added after review round 1.) The plugin rejects a pitch that jumps further than the gyro allows
   (I2C has no checksum), and logs how long its I2C read takes on the robot.

## Plan

- New `ImuSnapshotCheck.hpp` (judge one decoded burst), reusing `Bno055UartProtocol.hpp`'s register
  decoding (the register layout is the same on both buses). Rewrite `ICM20948.hpp/.cpp` around
  `/dev/i2c-1` with `I2C_RDWR`, `I2C_TIMEOUT` and the Adafruit-style start-up in `on_configure`.
- xacro parameters; remove the node from `rocko.launch.py`, `supervisor/modes.py`, CMake, and delete
  `ICM20948.py` and `tools/imu_node_test.py`.
- New `ZeroCommands.hpp` for the deactivate zero; one line in `state_machine.py`.
- Which tier proves what: Tier 1 compiles the IMU plugin. Nothing here can run it: the mock replaces
  it and the container has no I2C. The host tests prove the decision logic only.
- Risks: register values written from memory; the bus timeout is bus-wide; changing what a robot
  launch runs. The robot procedure below covers them.

## What changed

| File | What and why |
| --- | --- |
| `hardware/sensors/icm20948/ICM20948.cpp`, `include/rocko_env/ICM20948.hpp` | The plugin reads the sensor itself: open the bus, set a 10 ms timeout and no retries, check the chip id, start it the way Adafruit's driver did (CONFIG, reset, normal power, page 0, NDOF), then wait up to 3 s for a believable first sample in `on_configure`. In `read()`: one `I2C_RDWR` burst of 0x14 to 0x1F, then the check, then the fault tracker as before. No node, no client, no service. [How it works](../architecture/imu-i2c-read.md) |
| `include/rocko_env/ImuSnapshotCheck.hpp` (new) | The all-or-nothing check of one sample: all-equal angles (start-up zeros), angle limits, a rate beyond 2000 deg/s, a bad mounting. Offset, axis and sign are applied here. **Behaviour change:** a bad pitch rate is now a failed read that counts in the fault tracker. Until now it was only held and logged. That closes the hazard row 1.2 recorded before H-6 can be wired |
| `description/ros2_control/rocko.ros2_control.xacro` | Gyro block gets `i2c_device`, `i2c_address`, `pitch_offset_deg` (7.6), `pitch_rate_axis` (0), `pitch_rate_sign` (-1): the values `ICM20948.py` had |
| `bringup/launch/rocko.launch.py`, `supervisor/modes.py`, `CMakeLists.txt` | The IMU node is no longer launched, expected or installed |
| `ICM20948.py`, `tools/imu_node_test.py` | Deleted (the node is gone, so is its test). `rocko_interfaces`' `Icm20948Data` msg/srv are now unused and left in place; removing them is a follow-up |
| `controllers/add_feedforward/include/rocko_env/ZeroCommands.hpp` (new), `AddFeedforwardController.cpp` | `on_deactivate` zeroes every interface and returns `ERROR` if any write failed (row 4.14). Before, it counted the failure and returned `SUCCESS` |
| `supervisor/state_machine.py` | After a disable, a motion controller in any state other than `active` or `inactive` (for example `unconfigured` or `finalized`, where a failed `on_deactivate` leaves it) makes the supervisor kill the stack **at once** (review round 1; first written as a 6 s wait). Not while the stack is starting, where `unconfigured` is normal |
| `tools/imu_snapshot_check_test.cpp`, `tools/zero_commands_test.cpp` (new), `tools/supervisor_state_machine_test.py` | Host tests (below) |
| Docs | [imu-i2c-read.md](../architecture/imu-i2c-read.md) (new); the ladder page's host-test tables; next-steps rows 1.1, 1.2, 1.8, 4.14, the handoff items 3, 5 and 7, and the parameter table; findings C-1 and H-3; proposals 0008 (precondition 2) and 0010 (status); README |

## Verification

Final run, `tools/run_ladder.sh` on the laptop after the round 2 fixes (`./check_container_mount.sh`: `ok`):

| Tier | Command | Result |
| --- | --- | --- |
| precondition | `./check_container_mount.sh` | `ok    the container sees this checkout` |
| 0 static | `verify_urdf.sh`; hook tests; sim config drift check; doc link scan | pass |
| 1 build | `build_ros.sh` | pass, **0 compiler warnings** (round 1 had 1, the deprecated base `on_init` call, fixed). `ICM20948.cpp` compiles; the motor plugin is not built here (compile-only check: pass, but it did not change) |
| 2 mock | `verify_mock.sh` balance and tank; `verify_supervisor.sh` | pass. The mock **replaces the IMU plugin**, so this shows only that the launch, the supervisor and the controllers still wire up without the IMU node |
| 3 sim | `verify_sim.sh` (S1 S2a H4 C3 C2 H6 FF), `verify_tools.sh`, `verify_sim.sh scenarios` (U I V K S2b AB) | pass. `SimPlant` replaces the IMU plugin too: this shows nothing about the new read |
| host tests | all 14 (`tools/*_test.cpp`, `tools/*_test.py`, `pi_scripts_test.sh`) | pass, including `imu_snapshot_check_test` (9 functions), `zero_commands_test` (4) and `supervisor_state_machine_test.py` (23 tests). Each new rule went red on a deliberate break first (8 of 8, 2 of 2, 3 of 3; listed in the [ladder](../architecture/verification-ladder.md)) |

Highest tier reached for the IMU read itself: **Tier 1 (it compiles) plus host tests of its decision logic.** No tier ran the I2C code.

## Review rounds

| Round | Reviewer | Findings | Fixed / declined (why) |
| --- | --- | --- | --- |
| 1 | `realtime-auditor` | High: an in-range corrupted I2C sample is accepted (no checksum). Medium: the supervisor rule fired while the stack starts (`unconfigured` is normal there); a failed zero still waited 6 s before the kill. Low: fault tracker not reset on re-configure. Plausible High: the 10 ms timeout may not be a bound (100 Hz tick; Pi 5's driver polls up to 20 ms) | All fixed: pitch-jump check; no rule while starting, immediate kill; tracker reset; timeout left as is but the comment corrected and `read()` now measures and logs the slowest transaction every 30 s |
| 1 | `ros2-control-reviewer` | Low: `pitch_rate_axis`/`sign` parsed as decimals then truncated; stale "paste it in ICM20948.py" in the calibration script | Both fixed (`std::stoi` with a whole-string check) |
| 1 | `explainability-reviewer` | Medium: I2C code calls a `bno055_uart` decoder. Low: `kI2cTimeout10ms = 1` reads wrong; template with one user; stale offset pointer; loose angle limits unexplained | Header STATUS note added (rename declined: it touches the UART test too, for a name); `kI2cTimeoutMs = 10`; comments added |
| 1 | `docs-reviewer` | High: four living pages told the reader to run the deleted node; Medium: stale parameter tables and descriptions | All fixed (`measure-inertia.md`, `sim-plant.md`, `verify-in-container.md`, proposals 0004 and 0008, `sim-to-real-gap.md`). The superseded handoff boxes in next-steps were left as history |
| 1 | Tier 1 | A new deprecation warning: the base `on_init(info)` call | Fixed: parameters read from `info`, as `SimPlant` does. Round 2 ladder: 0 warnings |
| 2 | `realtime-auditor` | Medium: the gyro rate unchecked for corruption, and a bad new rate widened the pitch allowance. Low: the sample ending configure was never compared | Fixed: rate-step limit, allowance from the last rate only; configure needs two agreeing samples |
| 2 | `docs-reviewer` | Medium: journal criterion 4 and row 1.2 contradicted the code; a count copied into row 1.1. Low: wording | Fixed |
| 3 | `realtime-auditor` | **Medium, NOT fixed (round 3 dirty):** after one rejected sample the pitch allowance still uses the stale rate, so a real step of 250 deg/s or more in 10 ms is rejected every cycle until the tracker returns ERROR (a stop, so a fall when balancing). Proposed fix: `allowed_deg = (|last_rate| + 0.5 * kMaxPitchAccelDegS2 * dt) * dt + kMaxPitchJumpSlackDeg`, with the test `assert(pitch_step_plausible(0.0, 0.0, 6.0, 300.0, 0.02))` | **Handed to the owner** (CLAUDE.md §2 step 6). Whether such a step ever happens on the robot is unknown (the fused gyro is filtered). Nothing has been pushed |

## Not verified — needs a human

- **Nothing in the IMU change has run on the robot.** No tier here can run `ICM20948.cpp`. It is
  compiled (Tier 1), but the mock replaces it and the container has no I2C. Not checked: that the Pi's
  I2C driver honours the 10 ms timeout; the register values and the start-up sequence (written from
  memory of Adafruit's driver, not checked against the datasheet); how long the burst takes on the Pi
  and how often it fails; that the sensor ends up in the same orientation convention, so that the
  measured axis and sign still hold.
- **Row 4.14:** what `controller_manager` does when `on_deactivate` returns `ERROR` (expected: the
  controller is left `unconfigured` or `finalized`, and its interfaces are released) is read from
  the code, not run. The failure cannot be provoked on the mock.
- Nothing here says anything about balancing.

### Robot procedure

Owner at the robot; the agent may run the commands over SSH only if the owner allows it in that
session. **Balance mode stays disabled throughout** (the handoff rule). Wheels off the ground.

1. Pull and build on the Pi, as your user (not root): `cd ~/ROCKO-env && git pull && ./build_ros.sh`
   ([Run ROCKO on the Pi](../guides/run-on-the-pi.md)), then `sudo systemctl restart rocko.service` so the
   service picks up the new build. The build must finish with no errors in `ICM20948.cpp`. I2C must be enabled
   (it is: the Python node used the same bus).
2. Switch the supervisor to balance (LT+RT+RB, or the dashboard's Balance button). **Do not enable.**
3. In the launch log (`journalctl -u rocko.service -f`), expect, within about 2 s of the switch:
   `BNO055 on /dev/i2c-1 address 0x28; pitch offset +7.60 deg; pitch rate = -1 * gyro[0]`, then
   `BNO055 ready: pitch ... deg, roll ... deg, pitch rate ... deg/s`. **Abort** (switch back to tank) on
   `No BNO055 at address`, `start-up writes failed` or `no believable sample`. Note the exact line.
4. Check the values: `pi/rocko_run.sh ros2 topic echo /dynamic_joint_states` (or the dashboard's pitch
   plot). Tilt the robot forward by hand: **pitch must go DOWN**, as it did with the Python node, and
   `pitch_rate` must be **negative while you tilt forward**. Hold it still: `pitch_rate` within about
   +/-1 deg/s of zero. If either sign is reversed, stop. The reset may have changed the orientation
   convention; record what you saw.
5. Leave it running, disabled, for 10 minutes. Record the Pi model and the kernel tick
   (`grep CONFIG_HZ= /boot/config-$(uname -r)`). Count `IMU read failed` lines in the log, and copy the
   `IMU read timing: slowest ... us` lines (one every 30 s). **Success:** no `IMU read failed`, or a
   handful that never reach `ERROR`; no `IMU read failed (pitch or pitch-rate jump too large ...)` line while you tilt the robot by hand at normal
   speed; the slowest read well under 10 000 us. A steady stream of `IMU read failed` lines means the bus is unreliable from C++:
   record the reason the line gives (`not decoded` = bus error or timeout), and UART (proposal 0010)
   is next.
6. Shutdown check (the reason for C-1): switch back to tank, then `sudo systemctl restart rocko.service`.
   It must stop without a SIGKILL of `ros2_control_node` caused by the IMU. The encoder service can
   still hang it (row 1.3); say which.
7. Record the commit SHA, battery voltage and the log location ([reproducing results](../guides/reproducing-results.md)).

## Reproduce

From the repo root on the laptop:

```bash
./check_container_mount.sh
tools/run_ladder.sh
```

Host tests alone: `tools/run_ladder.sh --host`, or the `g++` lines at the top of each `tools/*_test.cpp`.

## Why the run stopped

Review round 3 was still dirty (one Medium, in the review table). Under CLAUDE.md §2 step 6 the agent
stopped, handed the finding to the owner, committed the verified work locally, and **did not push**.
