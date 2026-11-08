# Stage 6, bounded: the BNO055 UART framing as a tested header, the rest of C-1 as a proposal

- **Date / commit:** 2026-11-07, built on `4374d5c`. The helper and its test are one commit; this
  entry, the proposal and the doc updates are the commit after it.
- **Branch:** `ai-forward`. Committed locally; this run was told not to push.
- **Environment:** laptop (macOS, Apple clang 21.0.0) and the `rocko-env-dev` container (`ros:jazzy`,
  arm64, GCC 13.3.0). **Nothing ran on a robot or a Raspberry Pi.**

## Goal and success criteria

The person: "land only what the compile-only check / the ladder can cover; anything else becomes a
proposal". Finding [C-1](../claude-analysis/balance-loop-findings.md#c-1--unbounded-blocking-service-calls-inside-the-real-time-loop)
(blocking service calls with no timeout in the 100 Hz loop) needs a BNO055 on the Pi's UART and a
wiring change, so the plugins are **not** touched. Done means:

1. A header-only, allocation-free, I/O-free helper builds BNO055 UART read (and write) requests,
   checks replies with an explicit status, and decodes the 12-byte burst of registers `0x14`–`0x1F`
   into gyro (deg/s) and Euler (deg) as one snapshot, all NaN on any failure; every datasheet fact it
   relies on is marked UNVERIFIED.
2. A host test covers request bytes, a good reply with negative values, every error status,
   truncated and oversized frames, a wrong header, a length mismatch, and that no failure returns a
   finite value; and it goes red on at least three deliberate breaks.
3. `build_ros.sh` still passes with 0 warnings; `verify_urdf.sh`, `check_motor_plugin_compiles.sh`,
   the default `verify_sim.sh`, all `tools/` host tests and the hook tests pass.
4. Proposal 0010 holds the rest of C-1 (UART wiring, the plugin read with a bounded deadline, the
   encoders in C++, H-9), with the bench test and the tethered procedure.

## Plan

Files: the new header next to `ImuFaultTracker.hpp` (same include directory, so a later plugin change
needs no CMake change); the new host test in `tools/`; docs. No plugin, CMake, launch, URDF or YAML
change, so Tier 2 (the mock) has nothing new to see: it loads none of these files. Risk: a helper that
looks authoritative but encodes a wrong datasheet fact. Mitigation: every such fact is labelled
UNVERIFIED in the header and in the proposal, and checking them is step 2 of the proposal's bench test.

### Self-check (`control-loop-change` §1)

1. **Cycle stage:** none today. When wired in (proposal 0010) it runs in the IMU plugin's `read()`, at
   the start of the 10 ms cycle, before every controller's `update()` and the motors' `write()`.
2. **Concept:** the balance loop is only as good as the age and regularity of its pitch sample. A
   service call with no time limit makes that age unbounded; a serial read with a deadline makes it
   bounded (about 2–3 ms), and a missed deadline becomes a counted failure, not a stall.
3. **Measurement:** none. No Pi timing exists; the 1.56 ms wire time in the proposal is arithmetic
   (18 bytes × 10 bits ÷ 115200 baud), not a measurement.
4. **Gated?** No gain, version pin or safety behaviour changes. The proposed deadline is a new value a
   human confirms.

## What changed

| File | What and why |
| --- | --- |
| `rocko_env/rocko_env/hardware/sensors/icm20948/include/rocko_env/Bno055UartProtocol.hpp` (new) | Namespace `rocko_env::bno055_uart`. Named constants for the frame bytes, the `0xEE` status codes (`DeviceStatus`), the six registers and the 16 LSB/(deg/s) and 16 LSB/deg scale factors (all UNVERIFIED against the datasheet). `make_read_request`, `make_write_request`, `reply_frame_size` (how many bytes to wait for, from the first byte), `parse_read_reply`, `parse_write_reply` (explicit `ReplyStatus`: ok, device error, too short, bad header, length mismatch, too long), `int16_from_le` (two's complement without implementation-defined casts), `decode_gyro_euler`, `parse_gyro_euler_burst_reply`. `GyroEulerSnapshot` defaults to all NaN and `valid = false`. No plausibility limits: those stay in the plugin. **Included by nothing in the build** |
| `tools/bno055_uart_protocol_test.cpp` (new) | Host test, 11 test functions, 388 checks, a counting `CHECK` that prints the failing line and continues |
| [proposal 0010](../proposals/0010-c1-imu-uart-and-encoders-in-cpp.md) (new), [proposals index](../proposals/README.md) | The rest of C-1 |
| [verification ladder](../architecture/verification-ladder.md), [README](../../README.md), [verify-in-container](../guides/verify-in-container.md) | The host test listed where host tests are listed, as not a tier, with its breaks |
| [next-steps](../next-steps.md) rows 1.1, 1.3, 1.8, 1.10; [C-1 in the findings](../claude-analysis/balance-loop-findings.md#c-1--unbounded-blocking-service-calls-inside-the-real-time-loop); [final report](2026-11-04-final-report-stage-4-6.md) status box | Facts updated: the helper exists and is unwired; the rows stay open |

## Verification

| Tier | Command | Result |
| --- | --- | --- |
| precondition | `./check_container_mount.sh` | `ok    the container sees this checkout: /Users/ghart/projects/ROCKO-env` |
| host test (not a tier), laptop | `g++ -std=c++17 -Wall -Wextra -I rocko_env/rocko_env/hardware/sensors/icm20948/include tools/bno055_uart_protocol_test.cpp -o /tmp/bno055_uart_protocol_test && /tmp/bno055_uart_protocol_test` (also with `-Wpedantic -Wconversion -Wsign-conversion`) | `all Bno055UartProtocol tests passed (388 checks)`, exit 0, no warnings |
| host test, container (GCC 13.3) | the same, plus `-Wpedantic -Wconversion` | 388 checks pass, exit 0. The first run warned `multi-line comment [-Wcomment]` about the build command in the test's header comment (a `\` ending a `//` line); fixed by putting the command on one line, then no warnings. **Found in passing, not changed:** `tools/imu_fault_tracker_test.cpp` has the same backslash and prints the same warning under GCC (it still passes) |
| host test, deliberate breaks | five scratch copies of the header (the real one untouched), test compiled against each: (1) `kEulerLsbPerDeg` 16 → 100; (2) low and high byte swapped in `int16_from_le`; (3) `reply_frame_size` accepts any first byte (header check removed); (4) no two's complement (`return raw`); (5) an oversized `0xBB` frame accepted | red each time, exit 1: (1) 6 of 388 failed, first `heading_deg == 359.9375`; (2) 14, first `gyro_x_deg_s == 1.5`; (3) 265, first `status == kBadHeader`; (4) 7, first `gyro_y_deg_s == -2.0625`; (5) 2, first `status == kTooLong`. The real header: 388 of 388 |
| 1 build | `docker compose -f docker/compose.yml exec -T ros2 bash -c 'cd /ros2_ws && src/ROCKO-env/build_ros.sh'` | exit 0, `Summary: 2 packages finished`, 0 lines containing `warning`. Nothing includes the new header, so the build compiles nothing new: it shows only that nothing else broke |
| 0 static | `docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_urdf.sh` | `PASS: all seven expansions ...`, `ok    sim config: no drift` |
| compile-only (not a tier) | `docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/check_motor_plugin_compiles.sh` | `PASS (compile-only): syntax and types only. Not linked, not run, not a Pi build.` (the motor plugin is unchanged) |
| 0 static, host | `.claude/hooks/test-guard-protected-files.sh` | 81 cases, exit 0 |
| other host tests | the commands in the ladder | `all ImuFaultTracker tests passed`, `all SimPlantModel tests passed`, `all TipoverLatch tests passed`, `all FeedforwardTimeout tests passed`, `22/22 passed`, `5/5 passed`, `7/7 passed` |
| 2 mock | not run | No file the mock loads changed (no plugin, URDF, YAML or launch edit); the new header is not compiled into any library |
| 3 sim | `docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_sim.sh` | see *Sim run* below |

### Sim run

The default run **failed, exit 1**, on one check: `FAIL (simulation: S1 S2a H4 C3 C2 H6 FF)`. S1, S2a,
C3, C2, H6 and FF were `ok` (S2a: 35 deg 0.430 s after it started to move; C3: timed out 0.210 s after
the last command; FF: both wheels 0.208-0.209 s). H4's cutoff part passed (1599 of 1599 commands past
35 deg were exactly 0.0, both controllers tripped and refused the re-arm), and it failed on the
harness's own timing rule: `the commands were not watched for 1 s after the refused re-arm (refused
1794101590.873900130, last sample 1794101587.9139566)`. The refusal was logged about 19 s after the
trip, 3 s after the 16 s recording window (`TIPOVER_RECORD_S`) had closed.

`verify_sim.sh H4` alone, three more times: PASS, FAIL (the same rule: refused 0.76 s after the last
sample), PASS. So **H4 failed 2 of 4 runs today on the length of its recording window**, not on the
cutoff. Not caused by this change: nothing the sim loads changed (no plugin, YAML, launch or xacro
edit; the build recompiled nothing). Not fixed here (out of this task's scope); recorded in the
[ladder](../architecture/verification-ladder.md)'s H4 blind spot. A likely cause, not shown: the same
container slowness that put the refusal 11.4 s after the first write when the window was set to 16 s.
Run directory of the first failure kept in the container: `/tmp/tmp.bDj5NqSs4v`.

## Review rounds

Not run in this sub-task: the orchestrator that set it did not include the reviewer agents. The change
touches no §6 file, so `realtime-auditor` is not mandatory for it (it is for proposal 0010's plugin
change). `explainability-reviewer` (code) and `docs-reviewer` (docs) are owed.

## Not verified — needs a human

Everything about the hardware: the datasheet facts the header encodes (all UNVERIFIED), the serial
port, termios, the sensor's reply time, the 3 ms deadline, the axes and signs, the encoders in C++, and
anything on the robot. The procedure (bench test first, then wheels-off, then tethered) is in
[proposal 0010](../proposals/0010-c1-imu-uart-and-encoders-in-cpp.md). **Nothing in this change alters
what runs on the robot.**

**Why the run stopped here:** the bounded part of stage 6 that the ladder can check is landed; the
rest needs a Pi, a BNO055 in UART mode and a person (proposal 0010).
