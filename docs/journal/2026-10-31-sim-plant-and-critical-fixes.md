# Fix C-1/C-2/C-3 (+M-5, H-4, H-6) and build Tier 3: a `SimPlant` physics simulation

- **Date / commit:** 2026-10-31, built on `3882790`
- **Branch:** `ai-forward`
- **Environment:** `rocko-env-dev` container (`ros:jazzy`, arm64 on Docker Desktop for macOS), laptop
  for the hook tests. Unattended run (CLAUDE.md §2 "Unattended runs"): the person is away, so this
  entry is also the report.

## Goal and success criteria

The person asked for, in order: (1) fix the critical findings C-1, C-2, C-3, plus M-5, H-4 and H-6,
which the findings' own order of work puts on the same path and the sim needs; (2) build Tier 3, a
`SimPlant` physics simulation (finding M-4), so the existing cascade runs against dynamics and not
the mock; (3) show in that sim that the cascade balances, then verify every fix in the sim where the
sim can see it. The wording below is the person's.

- **S1.** `verify_sim.sh`, run in the container, launches `rocko_sim.launch.py`. The four cascade
  controllers load unchanged against `SimPlant` and go active. Pitch changes when the wheels move
  (the mock holds it at 0.0).
- **S2.** The verifier can fail. (a) With no control effort the simulated robot falls past the
  tipover angle within a stated time. (b) With one pitch-loop sign flipped in a throwaway copy, the
  upright scenario fails. Both are watched going red for the right reason before Tier 3 is called
  built.
- **S3.** Scenarios pass: hold upright 10 s, recover from an impulse, follow a velocity step. Sim
  gains live only in `rocko_sim_controllers.yaml`. Thresholds are committed **before** tuning starts.
  Then every scenario is rerun across a ±30 % sweep of the estimated parameters (mass,
  centre-of-mass height, motor gain, IMU delay). Report which corners fail; do not widen thresholds.
- **S4.** Each fix has the check that suits it. C-2: an injected IMU fault makes the hardware
  interface return ERROR and the controllers never see pitch 0.0. C-3: the watchdog trips in sim when
  commands stop. H-4: the tipover cutoff trips and disarms in sim. H-6: the D term comes from gyro
  rate. Pi-only code (motor, IMU and encoder plugins) is compile-checked only (next-steps row 0.2)
  and marked *done, awaiting robot*, with a robot procedure in this journal. The sim says nothing
  about those files.
- **S5.** Docs and statuses updated per §7. The ladder marks Tier 3 built only after S1 and S2.

### Decisions the person made in advance (not re-asked)

- Plant: `SimPlant` first, not Gazebo; a proposal to amend ADR 0003 is to be written (not yet written; see "Not done") and the work carries on.
- Inertia and wheel radius are unmeasured. Every such number sits in one parameter block labelled
  `ESTIMATE, NOT MEASURED`. No "right" wheel radius is picked and issue #16 is not closed.
  `docs/guides/measure-inertia.md` and a proposal ask a human to measure.
- `bringup/config/rocko_controllers.yaml` gains and `update_rate` are untouched. The sim launch loads
  the `*_sim.yaml`. Hardware launch files keep `use_mock_hardware:=false` as default; any new sim
  switch defaults to off. Hardware thresholds (watchdog timeout, tipover angle) and candidate
  hardware gains go in proposals, never into the real YAML.
- A structural reason the cascade cannot balance (a sign, wiring, a missing state) is a finding:
  journal it and fix the cause, do not tune around it.

## Plan

Stages, each: implement → verify up the ladder → parallel review → fix → document → commit → push.

1. Preflight and baseline Tiers 0–2.
2. Foundations: row 0.2 (compile-only check of the motor plugin), then 0.1 (C-2), 0.3 (M-5); 0.4–0.6
   if cheap.
3. Plant: `docs/architecture/sim-plant.md` first, then the `SimPlant` plugin, fault injection, xacro
   switch, `rocko_sim.launch.py`, `verify_sim.sh`. Proves S1, S2.
4. Safeguards against the sim: H-4, C-3, H-6, and the C-2 downstream check. Also H-2 (the `pwmVal > 2`
   guard in the motor plugin), moved up from stage 6 after review round 1, because whether H-7's
   mitigation reaches the PWM depends on it. **After review round 2 the code for H-2, M-1 and M-3
   was written earlier still, in stage 2** (see [*Deviation from the stage order*](#deviation-from-the-stage-order-the-motor-write-was-pulled-forward)).
5. Balance in the sim: scenarios, thresholds committed first, then tuning, then the robustness sweep,
   the sim-to-real-gap page and the proposals.
6. Only if 1–5 are done and review-clean: C-1 and M-1/M-3, limited to what a compile-only check
   covers; the rest becomes proposals. (M-1 and M-3 were pulled forward out of this stage; C-1 was not.)

## Stage log

Each stage is added below when it completes. Rows 0.x are in [`docs/next-steps.md`](../next-steps.md).

### Stage 2 — foundations (rows 0.2, 0.1, 0.3, 0.4, 0.5, 0.6, and the motor `write()` part of 1.4)

Built on `3882790` plus the stage 2 changes listed here (they are in the commit that adds this file). Status words are the ones in the
queue's [*Reading the table*](../next-steps.md#reading-the-table). This subsection describes the code
**after the fixes from review rounds 1 and 2** (see [Review rounds](#review-rounds)); the earlier
versions of the change are not described separately.

#### Deviation from the stage order: the motor `write()` was pulled forward

The person's stage order (the plan above) puts H-2 in stage 4 and M-1/M-3 in stage 6. Review round 2
(`realtime-auditor`) found that the stop paths written for C-2, M-5 and H-4 do nothing if the motor
plugin skips the write for a small command: the previous duty stays latched (H-2). So the substance of
H-2, M-1 and M-3 was written **in stage 2, before the order called for it**. Status: **pulled
forward; compile-checked only (type and syntax), awaiting robot.** It was not run by any tier, and
the plan's stage 4 and stage 6 entries above keep their original wording with a note added, so the deviation stays visible. What
is **not** pulled forward: C-1 for the encoders, L-1 (`joint_name_` at namespace scope), and any
hardware PWM (M-2).

**What changed**

| File | Change |
| --- | --- |
| `check_motor_plugin_compiles.sh` (new, repo root), `tools/compile_check/stubs/wiringPi.h`, `softPwm.h` (new) | Row 0.2. Runs `g++ -std=c++17 -fsyntax-only -Wall -Wextra` on `Motor12VoltQuadEncoder.cpp` against hand-written stub headers that hold declarations only. The stubs are `#error` unless `-DROCKO_COMPILE_CHECK_ONLY` is passed, and the command has no `-o`, so nothing can be linked or run. Optional first argument: another `.cpp` to check. Exit 0 pass, 1 compile error, 2 could not run. Run it in the container (below) |
| `rocko_interfaces/srv/Icm20948Data.srv` | Row 0.1 (C-2). The response gains `bool success`: true only if the three angles came from one good sensor read. Comments in the file give the units (degrees) and say that the sign of `pitch` is **not established** anywhere in the repo |
| `rocko_env/rocko_env/hardware/sensors/icm20948/ICM20948.py` | Row 0.1 (C-2). Reads `self.icm.euler` once (this also covers H-3 for this file). On an exception, or a `None` value, it answers `success = False` with NaN angles and a throttled warning, instead of the default 0.0. A comment on `pitch_offset` (7.6, degrees, added to the sensor pitch) says its origin and sign convention are undocumented. The value is unchanged. After review round 2, the node also rejects a reply in which all three Euler values are exactly equal (it answers `success = False` with NaN angles and the throttled warning): real data has 1/16 deg resolution and three independent angles, so three identical values are what an all-zero block from a bus fault or a reset sensor would look like, and `(0, 0, 0) + pitch_offset` would pass the range check. Not run by any tier: nothing here executes `ICM20948.py` |
| `rocko_env/rocko_env/hardware/sensors/icm20948/include/rocko_env/ImuFaultTracker.hpp` (new) | Row 0.1. Header-only counter in namespace `rocko_env`, standard library only, no allocation. `record(good_read)` once per cycle. `tripped()` is true when **either** `max_consecutive_failures` reads failed in a row **or** `kMaxWindowFailures = 10` of the last `kWindowReads = 20` reads failed (finding H-8: the window catches a sensor that alternates good and bad reads). Not latched: a good read clears the consecutive run at once, and the windowed count falls only as good reads push old failures out of the 20-read window. Host unit test: `tools/imu_fault_tracker_test.cpp` (below) |
| `rocko_env/rocko_env/hardware/sensors/icm20948/include/rocko_env/ICM20948.hpp` | Row 0.1. `kMaxConsecutiveImuFailures = 5` (5 reads = 50 ms at 100 Hz) and `kMaxWindowImuFailures = 10` (of the last 20 reads, 200 ms): both **proposed** values that a human confirms on the robot. `kMaxAbsPitchRollDeg = 180.0` and `kMaxAbsYawDeg = 360.0`: a reply beyond them is garbled and counts as a failed read. `_yaw`, `_pitch`, `_roll` now start as NaN, so a failure on the first cycle cannot look like 0.0. After review round 2: a `static_assert` that both limits are positive and the windowed limit fits in the 20-read window, and a comment saying the reported pitch already includes the +7.6 deg offset, so a real pitch in (172.4, 180] deg is also rejected (only an upside-down robot) |
| `rocko_env/rocko_env/hardware/sensors/icm20948/ICM20948.cpp` | Row 0.1. `read()` counts a read as good only if the service call succeeded, `success` is true, all three angles are finite, and they are inside the limits above. Otherwise it keeps the last good angles, records the failure, logs a throttled error (`kLogThrottleMs` = 1000 ms) that prints the consecutive, windowed and total counts, and returns `ERROR` while the tracker is tripped. The blocking service call itself (C-1) is unchanged |
| `rocko_env/rocko_env/controllers/add_feedforward/AddFeedforwardController.cpp`, `.../include/rocko_env/AddFeedforwardController.hpp` | Row 0.3 (M-5, M-6) and the mitigation half of H-7. `update_and_write_commands` checks the `bool` from `set_value()`, still tries every interface, counts failures, and returns `ERROR` only after `kMaxConsecutiveWriteFailures = 5` consecutive failed cycles (a **proposed** value; one transient failure no longer permanently stops one wheel). In chained mode it sets the reference back to NaN after reading it each cycle, and it **always writes** a value (before, NaN meant "skip the write"). After review round 2 the rules for that value are: a finite reference gives `reference + feedforward`; a NaN reference repeats the last finite command for at most `kMaxStaleReferenceCycles = 3` cycles in a row (30 ms nominal at 100 Hz) and then gives `0.0`, so one late upstream write no longer makes a 10 ms zero pulse; a reference that is +/-inf, or a sum that overflows, is refused, counted in `rejected_commands_`, logged (throttled) and written as `0.0`. `on_deactivate` now writes `0.0` to every command interface (finding H-7: the motor plugin keeps whatever is in its command slot). The per-interface state (`last_command_`, `stale_reference_cycles_`) is sized in `on_configure`, so the loop does not allocate. The count of 3 is in cycles, so it assumes this controller and `pitch_pid_controller` both run at `update_rate` 100 (a comment says so). `_feedforward` is now `std::atomic<double>` initialised to 0.0 (M-6); a non-finite message on its topic is ignored. Log throttle named `kLogThrottleMs` (1000 ms). Row 0.4 (L-3): the include is now `realtime_tools/realtime_buffer.hpp` |
| `rocko_env/rocko_env/hardware/motors/Motor12VoltQuadEncoder.cpp`, `.../include/rocko_env/Motor12VoltQuadEncoder.hpp` | Rows 1.4 (partly: H-2, M-1, M-3) and the other half of H-7; pulled forward, see *Deviation*. `write()` now calls `softPwmWrite()` on **every** cycle. A non-finite command, or `|cmd|` below `kDeadbandRadPerSec = 0.05` rad/s, writes duty 0 and leaves the direction pin as it was. Otherwise the direction pin is set from the sign, and duty = `clamp(|cmd| / MAX_RAD_PER_SEC * 100 + kStictionDutyPercent, 0, 100)` percent, with `kStictionDutyPercent = 2.0`, truncated to a whole percent by `static_cast<int>` because `softPwmWrite()` takes an `int`. The old `pwmVal > 2` guard (H-2), the unclamped value (M-3) and the per-cycle `RCLCPP_INFO` for the right wheel (M-1) are gone. Both constants are **proposed** and are in the table below. `MAX_RAD_PER_SEC` is unchanged. This file is on the 100 Hz path and is **not** compiled by Tier 1 |
| `tools/imu_fault_tracker_test.cpp` (new) | A host unit test of `ImuFaultTracker`: 7 test functions, plain `assert`, no ROS. Not a ladder tier. See *Verification* |
| `rocko_env/rocko_env/controllers/utils/RateLimiter.py` | Row 0.4 (L-3). The two docstrings that contain `\param` are raw strings (`r"""`) |
| `rocko_env/CMakeLists.txt` | Row 0.5 (L-4). `rocko_env/description/rviz` is added to the description `install(DIRECTORY ...)` block |
| `rocko_env/rocko_env/description/launch/view_robot.launch.py` | Row 0.6 (#11). The default of `description_package` is now `"rocko_env"`. Passing `description_package:=rocko_env` still works |

**Risks**

- `ICM20948.cpp` and `AddFeedforwardController.cpp` are on the 100 Hz path (CLAUDE.md §6), so
  `realtime-auditor` is mandatory for this stage. `Motor12VoltQuadEncoder.cpp` **was edited** after
  review round 2 (`write()`); it is only type-checked by the new script, and it is still not compiled
  by Tier 1, so nothing here has built or run it.
- Behaviour changes on the Pi that no tier can see: `write()` now writes the speed pin every cycle
  (before, never for a small command); a command under 0.05 rad/s now stops the wheel (before, it was
  dropped and the old duty stayed, H-2); a command from 0.05 up to about 1.19 rad/s now gets 2 % duty
  (before, every command under `|cmd| / MAX_RAD_PER_SEC < 0.01`, about 1.19 rad/s, was dropped, by
  arithmetic on the old code; 1.19 uses `MAX_RAD_PER_SEC` as coded, 119.38 rad/s, see the note on
  integer division below); and a pitch loop that issues many small corrections near
  equilibrium now gets a stop for each one under the deadband. Whether that helps or hurts balance
  is unknown: the 0.05 rad/s deadband and the 2 % offset are proposals, and nobody has run them.
- Because `AddFeedforwardController` repeats the last finite command for up to 3 cycles when its
  reference goes NaN, the wheels can keep driving for up to about 30 ms (nominal at 100 Hz, not
  measured) after the upstream controller stops, before the `0.0` is written.
- Both new `ERROR` returns change what `controller_manager` does when they fire. What the robot does
  next is not known and has not been observed, and it depends on the ros2_control version; see H-7
  below.
- The `.srv` gained a field, so build the node and the plugin from the same commit.
- The new code may need a newer Jazzy `ros2_control` 4.x than the `find_package(controller_manager
  4.0.0)` minimum in `CMakeLists.txt` promises (`set_value()` returning `bool`; the `realtime_tools`
  `.hpp` headers). **Plausible, unproven**: raised by the `ros2-control-reviewer`, not tested against
  an older release. Raising the minimum would be a version-pin change, which is a human's decision
  ([ADR 0001](../decisions/0001-version-lock.md)); no proposal has been written yet. Until someone
  decides, record the installed versions with the result (step 1 of the procedures below).

**Row 0.2 — what the compile-only check proves.** It proves the file parses, every name resolves,
every call matches a declared signature, and the ROS 2 headers it uses exist in Jazzy. First run
(2026-10-31) on the unmodified file: **PASS, exit 0**, so the motor plugin does compile against
Jazzy's headers. Rerun on the **edited** `Motor12VoltQuadEncoder.cpp` (the new `write()`) after review
round 2: **PASS, exit 0**. So the new `write()` parses, `std::clamp` and `<algorithm>` resolve, and the
`softPwmWrite` call matches the stub's declaration. It does **not** prove: that the real WiringPi signatures match (the stubs are
hand-copied, so a stub that differs from the real header would hide a mistake); that it links; that
it runs; or that it builds on a Pi. It is not a ladder tier. Tier 1 (`build_ros.sh`) still leaves the
motor plugin out. It was made to fail for the right reason on four broken copies in the container's
`/tmp`, each exit 1 with `FAIL (compile-only)`:

| Break in the copy | First error printed |
| --- | --- |
| a string where an `int` is expected | `invalid conversion from const char* to int` |
| a missing `;` | `expected ';' before 'return'` |
| `#include <softPwm.h>` removed | `softPwmCreate was not declared` |
| `HW_IF_VELOCIT` (a misspelt interface constant) | `not a member of hardware_interface` |

**Row 0.1 (C-2) — what was and was not checked.** Tier 1 compiles `ICM20948.cpp` in the container,
and the build log (`log/latest_build/rocko_env/stderr.log`) has no compiler warnings. Tier 2 passes,
but the mock replaces this plugin, so Tier 2 cannot see the change. The counting logic alone
(`ImuFaultTracker`) has a host unit test that passes and was seen to fail on three deliberate breaks
(see *Verification*). **Not checked:** anything on a Pi, the node against a real sensor, and the
fault path through `read()` itself. **C-1 is not fixed:** `read()` still makes a blocking service
call, so a dead `icm20948` node can still block `read()` forever and no failure is ever counted.
**H-9 is not fixed:** a BNO055 that is frozen, or that has reset and still returns valid-looking
numbers, is not detected, because the check only sees failed or out-of-range reads. One variant is
now caught in the node: a reply with three identical Euler values (for example all zeros) is rejected
(round 2). That is a guess about what a bus fault or reset returns, and it has not been seen on a
sensor; a frozen sensor that returns its last real orientation, or any garbage whose three values
differ, is still not detected. Status: *done, awaiting robot*.

**Row 0.3 (M-5, M-6).** The Tier 1 build no longer prints the `ignoring return value ... set_value`
warning (the build log was read), and after the round 2 changes the build has no compiler warnings.
The failure path cannot be provoked on the mock, so no tier has seen it run. **The new logic of this
controller (the NaN-to-zero rule, the 3-cycle hold, the non-finite refusal, `on_deactivate`) has no
automated test.** A gtest for it was proposed in review round 2 and **declined for now**: the plan is
that the Tier 3 fault scenarios exercise it end to end (not built yet, so nothing exercises it today).
The only automated test near it is the host test of `ImuFaultTracker`, which is a different class.
Status: *done, awaiting robot* (critical-path file). The `std::atomic<double>` `static_assert` that
the type is lock-free compiled in the container; that it also holds on a Pi was not checked.

**Rows 0.4, 0.5, 0.6.** L-3: the Tier 1 build has no warnings from the include, and the Python
`SyntaxWarning` is gone (checked on the host with `python3 -W error -m py_compile`). L-4: after the
build, `ls install/rocko_env/share/rocko_env/rviz/` shows `rocko.rviz` and `rocko_view.rviz`.
**Not checked:** that RViz opens with those layouts, which needs a display. #11: the default
changed in code and Tier 1 builds it; `view_robot.launch.py` was **not run**, with or without a
display.

Static facts that were checked by reading, not by running: `rocko_view.rviz` and `rocko.rviz` are
installed under `share/rocko_env/rviz/`, and `view_robot.launch.py` builds its config path as
`FindPackageShare(description_package)/rviz/rocko_view.rviz`, so with the new default
(`description_package` = `rocko_env`) it points at an installed file.

**Procedure for a person with a display (rows 0.5 and 0.6)**

1. Machine: any machine with a display and the built workspace (the Pi with a monitor, or a laptop
   with ROS 2 Jazzy). In a shell run `source /opt/ros/jazzy/setup.bash` and then
   `source <workspace>/install/setup.bash`.
2. Run: `ros2 launch rocko_env view_robot.launch.py`
3. Success: RViz opens and shows the robot model with the `rocko_view.rviz` layout. No hardware is
   needed.
4. If it fails, retry with `ros2 launch rocko_env view_robot.launch.py description_package:=rocko_env`
   and report which of the two ran and the error text.

**Robot procedure for C-2 (a person, on the Pi; tethered, wheels off the ground first)**

1. Machine: the Raspberry Pi, in a shell where you have run `source /opt/ros/jazzy/setup.bash`. First
   bring Jazzy up to date, because the new code may need a newer `ros2_control` 4.x than CMake's
   minimum (see *Risks*): `sudo apt update && sudo apt upgrade`. Then record the versions
   (`ros2 pkg xml hardware_interface | grep '<version>'` and `ros2 pkg xml realtime_tools | grep
   '<version>'`) and compare them with the same two commands run in the dev container, and write both
   pairs down. Build with the motor plugin (WiringPi installed, `ROCKO_SKIP_WIRINGPI` not set to
   `ON`): `cd ~/ros2_ws/src/ROCKO-env && ./build_ros.sh`, then `source ~/ros2_ws/install/setup.bash`.
   Record the commit SHA. If the build fails on `set_value` or a `realtime_tools` header, that is the
   version question above answered; report it.
2. Put the robot on a stand so both wheels are free. Have a way to cut motor power within reach. Abort
   by cutting motor power at the first wheel motion you did not expect.
3. Launch balancing: `ros2 launch rocko_env rocko.launch.py`. In another terminal (run both `source` lines there too) run `ros2 control list_hardware_components` and `ros2 topic echo /pitch_pid_controller/controller_state`.
   Success: the IMU component is `active` and `feedback` follows the way you tilt the robot by hand.
4. Normal running: confirm the log has no `IMU read failed` line.
5. With the controllers active, briefly disconnect the IMU SDA/SCL wires, or stop the `icm20948` node.
   Expected: `feedback` never shows `0.0` because of the fault; a throttled `IMU read failed` error
   appears, naming the failed-in-a-row and last-20 counts; after five failed reads in a row the
   plugin's `read()` returns ERROR. ERROR is a return code, **not** a lifecycle state: expect the
   component to end up unconfigured or finalized (**unverified**, and it depends on the ros2_control
   version), so run `ros2 control list_hardware_components` again and write down what it says. Timing:
   five cycles is 50 ms only if each failed read returns promptly. A bus that is held (clock
   stretching, SCL stuck low) can block one read for up to the kernel I2C timeout (about 1 s, not
   measured here) before the Python node raises, whereas pulling the wires should fail quickly, so
   time both and write the numbers down. If you stop the node instead, `read()` blocks (C-1) and no
   failure is ever counted, so unplugging the wires is the better test.
6. Record what the wheels and the PWM do **after** the ERROR (see H-7: they may keep turning at the
   last command). From the code, what should happen: the controller repeats its last command for at
   most 3 cycles after its reference stops arriving and then writes `0.0`, and the motor plugin now
   writes 0 % duty for it (H-2 pulled forward). Whether either of those happens on the robot, and what
   `controller_manager` does with the other components after the `ERROR`, is unobserved. Do not
   assume. Also write down what `ros2 control list_controllers` shows.
7. Reconnect the IMU and confirm the component and controllers can be activated again.
8. Record with the result: the commit SHA, battery voltage, the surface (or "on a stand"), and where
   the log went ([Reproducing results](../guides/reproducing-results.md)).

**M-5 on the robot.** There is no practical way to make `set_value()` return `false` on the Pi. Look
for the `set_value() failed on a command interface` line in a normal run; none is expected. If one
appears, it is a finding, not noise. The controller returns `ERROR` only after five failed cycles in a
row, so a single line does not by itself mean it stopped. If five in a row ever happen, write down
what `ros2 control list_controllers` shows afterwards and whether the wheels stop: what
`controller_manager` does after an `update()` returns `ERROR` depends on the ros2_control version,
and nobody has observed it here.

**Row 1.4, partly (H-2, M-1, M-3): what was and was not checked.** Checked: the Tier 1 build has no
compiler warnings (but it leaves this file out); `check_motor_plugin_compiles.sh` passes on the edited
file (syntax and types only, against hand-copied stubs). **Not checked:** that the file builds against
real WiringPi; that it runs; what duty the pin actually gets; that the direction pin does not chatter
near zero; the 0.05 rad/s deadband and the 2 % offset (both proposals); timing of `softPwmWrite()` at
100 Hz. Not done and still open in row 1.4: **L-1** (`joint_name_` at namespace scope) and **C-1 for
the encoders**. An observation, not fixed and not part of this change: `MAX_RAD_PER_SEC = 1150 / 60 *
M_PI * 2` divides two `int` literals, so `1150 / 60` is 19, not 19.17, and the constant is 119.38 rad/s
and not 120.43 rad/s (a 0.9 % difference, by arithmetic; not checked on the robot). It is now finding
[L-7](../claude-analysis/balance-loop-findings.md#l-7--max_rad_per_sec-divides-integers). Status: *pulled forward; compile-checked only (type/syntax), awaiting robot*.

**Robot procedure for the motor `write()` (a person, on the Pi; tethered, wheels off the ground)**

1. Machine, build and versions: step 1 of the C-2 procedure above, with the motor plugin built. Record
   the commit SHA.
2. Put the robot on a stand so both wheels turn freely. Have a way to cut motor power within reach.
   Abort by cutting motor power at the first wheel motion you did not command, or if a wheel does not
   stop when you command zero.
3. Use **tank mode**, so the balance loop is not involved and only the motor plugin is under test:
   `ros2 launch rocko_env rocko_tank_mode.launch.py`. In a second terminal (run both `source` lines
   there too), send velocity commands with the gamepad path in the README. As an alternative, publish to
   `/diffbot_base_controller/cmd_vel` (type `geometry_msgs/msg/TwistStamped`, as `Joystick.py`
   publishes it; the tank config's `cmd_vel_timeout` is 1.0 s, so publish repeatedly with `-r 10`).
   That `ros2 topic pub` route was **not tried**, and whether the controller accepts a message with an
   empty header stamp was not checked. Wheel speed in rad/s is `linear.x / 0.127` there (`wheel_radius` in
   `rocko_tank_mode_controllers.yaml`), which is itself disputed (row 0.7), so read the speed off the
   wheel, not off this formula.
4. **Command zero.** Success: both wheels stop and stay stopped. Failure: either wheel keeps turning.
   This is the H-2 check.
5. **Small commands.** Step `linear.x` through values that give roughly 0.02, 0.05, 0.1, 0.5 and 1.0
   rad/s at the wheel. Expected from the code: below 0.05 rad/s the wheel is stopped; from 0.05 up to
   about 1.19 rad/s the duty is 2 % (whole-percent truncation, about 1.19 rad/s per percent), so the
   wheel may not turn at all, or may stutter. Write down, for each step, whether the wheel turns, in
   which direction, and whether the direction ever flips with no change of sign in the command.
   Whether 2 % moves the motor is exactly what the 2 % offset is a guess about.
6. **Direction near zero.** Command a small positive value, then a small negative value, then zero.
   Success: the wheel reverses with the sign, and at zero it stops without the direction pin
   changing. (The direction pin is deliberately left alone inside the deadband.)
7. **Duty never above 100.** The duty is not visible from ROS. Put an oscilloscope or logic analyser
   on the speed pin (per finding M-2, WiringPi pin 14 for the left wheel and WiringPi pin 13 for the right; these are WiringPi numbers, not BCM GPIO numbers, so look up the physical header pin with `gpio readall` on the Pi) and
   record the duty at each step above. Command a speed above 119.4 rad/s at the wheel only if the
   tank config's velocity limits let it through; if they do not, the clamp is checked by reading the
   code only, and the journal must say so.
8. **Feel.** Write down whether the 0.05 rad/s deadband and the 2 % offset look right (does the wheel
   creep, buzz, or refuse to start?). They are proposals; the wheels decide.
9. Record with the result: the commit SHA, battery voltage, the surface (or "on a stand"), and where
   the log went ([Reproducing results](../guides/reproducing-results.md)). Do not write that balance
   improved: tank mode does not balance.

**Proposed safety and hardware values.** None of these was measured or chosen by a human. Each is an
edit-and-rebuild constant, not a ROS parameter, and a human confirms or changes it on the robot. A
[proposal](../proposals/README.md) for the hardware thresholds is to follow (not written yet).

| Name | Value | File | Meaning |
| --- | --- | --- | --- |
| `kDeadbandRadPerSec` | 0.05 rad/s | `Motor12VoltQuadEncoder.hpp` | A wheel command with a smaller magnitude writes 0 % duty |
| `kStictionDutyPercent` | 2.0 % of full duty | `Motor12VoltQuadEncoder.hpp` | Added to the scaled command so a small command can overcome static friction |
| `kMaxStaleReferenceCycles` | 3 cycles (30 ms nominal at 100 Hz) | `AddFeedforwardController.hpp` | NaN references in a row for which the last finite command is repeated before `0.0` is written |
| `kMaxConsecutiveWriteFailures` | 5 cycles (50 ms nominal) | `AddFeedforwardController.hpp` | Failed `set_value()` cycles in a row before `update` returns `ERROR` |
| `kMaxConsecutiveImuFailures` | 5 reads (50 ms nominal) | `ICM20948.hpp` | Failed IMU reads in a row before `read()` returns `ERROR` |
| `kMaxWindowImuFailures` | 10 of the last 20 reads (200 ms) | `ICM20948.hpp` (default from `ImuFaultTracker.hpp`) | Failed IMU reads in the window before `read()` returns `ERROR` |
| `kMaxAbsPitchRollDeg`, `kMaxAbsYawDeg` | 180 deg, 360 deg | `ICM20948.hpp` | An IMU angle beyond these counts as a failed read (these two were proposed in round 1; the table adds them so every such value is in one place) |

**H-7 (finding, found while writing C-2; read-only analysis, unconfirmed on hardware; raised to
Critical by the `realtime-auditor` in review round 1).** When the IMU plugin returns `ERROR`, or the
cascade drops out, the motor plugin is a separate component and is not deactivated. Its `write()`
keeps writing the stale `_wheel.cmd`, so both motors may keep being driven by the last command while
the gamepad still steers a robot that can no longer sense its tilt. **Status after review round 2: partly mitigated, still
unconfirmed on hardware.** Four things now exist, all in code and none run on a robot: (1)
`AddFeedforwardController` sets its chained reference back to NaN after reading it each cycle, so a
dead upstream controller leaves NaN and not a repeated old value; (2) it then holds the last finite
command for at most 3 cycles and writes `0.0` after that (an explicit stop); (3) `on_deactivate`
writes `0.0` to every command interface; (4) the motor plugin's `write()` now writes the speed pin on
every cycle, so that `0.0` reaches the PWM as 0 % duty and no longer leaves the previous duty latched
(the H-2 guard is gone). Round 2's High finding was an interaction between (2) and the old H-2 guard:
writing `0.0` on the fault paths flipped the motor direction pin while the old duty stayed latched,
which is why (4) was pulled forward. What is **not** known: what `controller_manager` does after a
hardware component's `read()` or a controller's `update()` returns `ERROR` in Jazzy 4.x. What the
author remembers of that behaviour is **from memory, not from a test or from reading the installed
source**, so it is not written down here as fact; the robot procedure records what happens. The stop-or-hold behaviour is now defined in comments but its
limits are proposals, so row 1.7 stays *partly done*. Nobody has run this on a robot. It is in the
[findings](../claude-analysis/balance-loop-findings.md#h-7--the-motors-keep-their-last-command-when-the-cascade-stops)
and queued as [next-steps row 1.7](../next-steps.md), status *partly done*.

**New findings from review round 1** (all in the
[findings](../claude-analysis/balance-loop-findings.md)): M-6, the feedforward value was
uninitialised and shared between threads without synchronisation (fixed); H-8, an intermittent IMU
fault that alternates good and bad reads never tripped the consecutive-only counter (fixed in the
tracker); H-9, a frozen or reset BNO055 that still reads valid-looking numbers is undetectable (not
fixed; needs the read in C++, which is C-1); L-5, a D-term spike after held cycles (not fixed). A note
was added to C-1: when a timeout is added, its timeout branch must remove the pending request, or the
client's pending-request map grows.

### Stage 3 — the plant: `SimPlant`, Tier 3 (partly built)

Built on `1318f95` (the stage 2 commit), committed as the stage 3 commit. The
specification the code follows is [`docs/architecture/sim-plant.md`](../architecture/sim-plant.md).
Status words as in the queue's [*Reading the table*](../next-steps.md#reading-the-table).

**Where each success criterion stands**

| Criterion | State | Evidence |
| --- | --- | --- |
| S1: the four cascade controllers load unchanged against `SimPlant`, go active, and pitch changes when the wheels move | **Met** (in the model) | `verify_sim.sh` baseline and command runs, below |
| S2a: with no torque the simulated robot falls past 35 deg within a stated time | **Met** | `verify_sim.sh` S2a run: 0.440, 0.432, 0.430 s against a limit of 0.48 s |
| S2b: a copy of the upright scenario with one sign flipped must fail | **Not done.** The upright scenario does not exist, so there is nothing to flip | |
| S3: upright, impulse and velocity-step scenarios; thresholds committed before tuning; the +/-30 % sweep | **Not done** | The impulse and IMU-fault *parameters* exist and are unit-tested; no scenario uses them |
| S4: the fixes checked in the sim (C-2 downstream, C-3, H-4, H-6) | **Not done.** C-3, H-4 and H-6 are not written; the C-2 downstream check has no scenario | |
| S5: docs updated; Tier 3 marked built only after S1 **and** S2 | Docs updated. Tier 3 is **not** marked built, because S2b is missing | [ladder](../architecture/verification-ladder.md) |

**What changed**

| File | Change |
| --- | --- |
| `rocko_env/rocko_env/hardware/sim/include/rocko_env/SimPlantModel.hpp` (new, 484 lines on 2026-11-02) | The physics, plain C++17 with no ROS types. One parameter block (`SimPlantParams`) marked `ESTIMATE, NOT MEASURED`; a planar inverted pendulum on wheels integrated with RK4 at `kSubStepS` = 1 ms; a DC-motor model (`motor_output_torque`); the IMU delay as a ring buffer of the lean angle, sized once in the constructor; a copy of the **new** `Motor12VoltQuadEncoder::write()` duty map (`write_path_duty`) that keeps the integer-division `MAX_RAD_PER_SEC` (`kWriteMaxRadPerSec` = 119.381 rad/s) on purpose so the sim sees what the motor sees (finding [L-7](../claude-analysis/balance-loop-findings.md#l-7--max_rad_per_sec-divides-integers)); a switch `model_encoder_node` that copies the encoder node's speed halving; the C-2 rule through `ImuFaultTracker` (`ImuFaultTracker.hpp`) |
| `rocko_env/rocko_env/hardware/sim/SimPlant.cpp`, `.../include/rocko_env/SimPlant.hpp` (new) | The `rocko_env/SimPlant` `SystemInterface` that wraps the model and exports the same 12 interfaces as the real hardware. **Release hold:** `released_` (a member of `SimPlant`, set in `SimPlant::write()`) keeps the plant still until both wheel commands are finite, which happens when the cascade activates, because the hardware component activates seconds before the controllers are spawned. Sim time and every scenario time count from release. `read()` carries the sub-step remainder (`unintegrated_s_`) from one cycle to the next (round 2) |
| `rocko_env/rocko_env_hardware.xml`, `rocko_env/CMakeLists.txt` | Registers the plugin and always builds it (it needs no WiringPi), adds its include folder |
| `rocko_env/rocko_env/description/urdf/rocko.urdf.xacro`, `.../ros2_control/rocko.ros2_control.xacro` | `use_sim_plant` (default `false`) and 29 sim parameters as xacro args. The new `SimRocko` block is the first `xacro:if` in the ros2_control xacro. The real `<ros2_control>` block is **unchanged inside**; only its condition is widened to `unless use_mock_hardware or use_sim_plant`, so with the sim on neither the real plugins nor the mock load |
| `rocko_env/rocko_env/bringup/config/rocko_sim_controllers.yaml` (new) | A copy of the hardware controller wiring and gains. **Sim gains live only here.** `rocko_controllers.yaml` and `rocko.launch.py` are untouched, and nothing sets `use_sim_plant` except `rocko_sim.launch.py` |
| `rocko_env/rocko_env/bringup/launch/rocko_sim.launch.py` (new) | Starts the sim with a banner. `teleop:=true` by default (starts `Joystick.py` and `DiffDriveController.py`); spawns `joint_state_broadcaster` first, then the four cascade controllers; forwards the fault and scenario arguments (`SIM_ARGUMENTS`) to the xacro |
| `sim/sim_probe.py` (new) | Records pitch from `/dynamic_joint_states`, writes `samples.csv` and `metrics.json`, and exits 0 / 1 / 2 by mode (`report`, `exceed`, `change`, `still`) |
| `verify_sim.sh` (new, repo root) | The Tier 3 check: three launches (S1 baseline, S1 command, S2a); thresholds are named at the top of the file with the reason for each |
| `verify_urdf.sh` | A fifth and sixth expansion, `rocko` with `use_sim_plant:=true` (mock off and on): exactly one `SimPlant` (`SimRocko`), no real plugin, no mock, and the same interfaces as the real hardware; a seventh, tank mode asked for `use_sim_plant:=true`, must still hold the two real motors and no `SimPlant`; and `sim/check_sim_config.py`. The success output is now **12 `ok` lines at the left margin plus 4 indented ones from the config check, and `PASS`** (it was 6 lines before stage 3; counted from a run on 2026-11-02) |
| `tools/sim_plant_model_test.cpp` (new) | A host unit test of the model: 70 checks (61 in stage 3, plus 9 for parameter validation in round 2) against the numbers of [sim-plant.md §7.1](../architecture/sim-plant.md). Not a tier |
| `docs/architecture/sim-plant.md` | New in this change (the implementer's specification of the plant, written with the code), with a *predicted vs observed* note in §7.2 added at the end of stage 3 |

**Risks**

- `rocko_sim_controllers.yaml` is in `bringup/config/`, which `CLAUDE.md` §6 lists as critical path,
  so `realtime-auditor` is mandatory for this stage even though `SimPlant` runs only off the robot.
  No hardware gain, limit or `update_rate` was changed; the sim file starts as a copy of them.
- `rocko.ros2_control.xacro` and `rocko.urdf.xacro` are edited, and the real block's condition was
  widened. With `use_sim_plant` left at its default (`false`) the condition is the old one; `verify_urdf.sh`
  checks that the two real expansions still hold the real plugins and no mock. It cannot check the Pi.
- `SimPlant.cpp` is always built, so every Tier 1 build now compiles it and every installed package
  contains the plugin. It cannot be selected by accident: only `use_sim_plant:=true` loads it.
- Copies drift: the duty map and the encoder-node halving are copies of robot code. Nothing checks
  that they still match. (The sim's *configuration* is checked since review round 1: see below.)
- **A pass of any sim check is a pass for a model with invented numbers.** Do not read it as a
  statement about the robot or about any gain.

**The observation worth a human's eye: the hardware gains in the model.** With the unchanged
hardware gains (copied into `rocko_sim_controllers.yaml`), a 2 deg start, teleop on, `pitch_sign =
-1`, `encoder_sign = +1` and a 20 ms IMU delay, the simulated pitch decays to about 1 deg, then
drifts: 4.8 deg at 8 s, 13 deg at 9 s, 31.7 deg at 10 s after release, past 35 deg at 10.05 s, lying
at 60 deg at 10.20 s, with the wheels at -2.0 m/s at 10 s. So in the model the cascade does **not**
hold the robot up with these gains. Three limits on what that means: the plant's numbers are
estimates; the two sign parameters are assumptions that nobody has established on the robot (the
`.srv` and `pitch_offset` comments say the sign of pitch is not established); and this is one run, not
a sweep. It is **not** evidence that the real robot falls, or that these gains are bad. It also
contradicts the failure mode that [sim-plant.md §7.2](../architecture/sim-plant.md#72-model-predictions-with-the-hardware-gains-not-robot-claims)
predicted (fast oscillation made unstable by the delay): the model failed by a slow drift instead.
That note is now in §7.2. **This run was not recorded with an exact command**, so the numbers are the
implementer's report (a slow drift, no growing oscillation observed); a run such as `ros2 launch rocko_env rocko_sim.launch.py initial_pitch_deg:=2.0`
with `sim/sim_probe.py --duration 15 --mode report` should show the same kind of curve (**not rerun
by the author of this entry**). No gain was changed in response. Candidate gains for the sim belong in
`rocko_sim_controllers.yaml` and, if ever promoted, in a proposal.

**Verification (stage 3).** All run on 2026-10-31 in the container, before the stage 3 commit.

| Tier | Command | Result |
| --- | --- | --- |
| pre | `./check_container_mount.sh` | `ok`, exit 0 (as for stage 2) |
| 1 build | `docker compose -f docker/compose.yml exec -T ros2 bash -c 'cd /ros2_ws && src/ROCKO-env/build_ros.sh'` | exit 0, 0 compiler warnings. `SimPlant.cpp` is compiled; `Motor12VoltQuadEncoder.cpp` is still left out |
| 0 static | `docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_urdf.sh` | `PASS`, 7 expansions, 12 `ok` lines at the left margin plus 4 indented from the config check (round 2, 2026-11-02; at the first stage 3 run it was 5 expansions and 8 lines, and after round 1, 6 expansions). `rocko sim=true` holds exactly one `SimPlant` and exports the same 12 interfaces as the real hardware |
| 2 mock | `verify_mock.sh`; `verify_mock.sh tank` (same prefix) | PASS; PASS |
| compile-only (not a tier) | `check_motor_plugin_compiles.sh` | PASS |
| 0 host | `.claude/hooks/test-guard-protected-files.sh` | 81 cases ok |
| host unit test (not a tier) | `g++ -std=c++17 -Wall -Wextra -I rocko_env/rocko_env/hardware/sim/include -I rocko_env/rocko_env/hardware/sensors/icm20948/include tools/sim_plant_model_test.cpp -o /tmp/sim_plant_model_test && /tmp/sim_plant_model_test` | `all SimPlantModel tests passed`, 70 `ok` (61 at the first stage 3 run), 0 `FAIL`, exit 0. Falling-pole rate 8.59149 s^-1 (expected 8.5915); fall from 2 to 35 deg with no torque 0.427297 s (expected 0.4273); through the 100 Hz loop with a 20 ms delay 0.45 s. This command was copied from the test file's header comment and run again while writing this entry |
| 3 sim (partly built) | `docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_sim.sh` | `PASS (simulation, S1 and S2a)`, exit 0. S1 baseline moved 0.0000 deg (limit 0.01). S1 command (+5 rad/s feedforward) moved the pitch 0.06 s after release, to a maximum of 13.8, 60.0 and 60.2 deg in three runs before round 1, 8.30 deg in a later one and 60.23 deg in the round-2 run (it varies from run to run; only the 0.5 threshold matters). S2a: 35 deg passed within 0.48 s, measured 0.440, 0.432, 0.430 s |
| 4 robot | | not run, and not possible for an agent |

**Deliberate breaks (each watched going red for the right reason, then restored)**

| Break | Result |
| --- | --- |
| Flip the sign of the body torque in `SimPlantModel.hpp` | Unit test: 9 checks failed |
| Drop gravity | Unit test: 22 checks failed |
| Leave `roll` out of the sim's `<ros2_control>` block | `verify_urdf.sh` FAIL from the sim-versus-real interface comparison |
| Put the real block's old condition back | `verify_urdf.sh` FAIL: `plugins=4` in the `rocko sim=true` expansion |
| Freeze the pitch `SimPlant` reports | `verify_sim.sh` FAIL: the S1 command run moved 0.0000 deg, and S2a never reached 35 deg |
| Make `SimPlant` ignore `disable_torque` | `verify_sim.sh` FAIL: S2a never reached 35 deg (it peaked at 2.92 deg); unit test: 16 checks failed |

**Not done / not verified (stage 3)**

- **S2b and every scenario** (upright, impulse, velocity step), so Tier 3 is **not** called built, and
  the robustness sweep (S3) has not been run.
- **The impulse and IMU-fault parameters** are unit-tested but were not run end to end through a launch.
- **The wheel radius (issue #16)** is only exercised inside the header class `SimPlantModel`, not through
  the plugin, and the sim picks no value as right.
- **The release hold** has no check of its own, except that `verify_sim.sh` now requires the release to
  be logged at least 3 s before the probe stops (so a robot that was never released cannot pass the
  baseline). It exists so the plant does not fall during the seconds between the hardware activating
  and the controllers starting; the robot has no such hold. How long the loops take to have a
  reference after release (an open-loop gap) was not measured: sim-plant.md §3.6.
- **One hardware owner.** An IMU `ERROR` in the sim stops the whole plant, unlike the robot, where the
  motor plugin is a separate component (H-7). The sim cannot show H-7.
- **The unit test is not called by any script.**
- **Deprecated `on_init(HardwareInfo)` override.** All hardware plugins, `SimPlant` included, use it;
  the build prints no warning, and whether a later release removes it was not checked.
- **A proposal to amend ADR 0003** (Gazebo to `SimPlant` first) is to follow; it is not written yet.
  See [next-steps row 2.1](../next-steps.md).
- **Anything about the robot.** Every number comes from a model with estimated parameters.
- **Review of stage 3**: round 1 findings from `ros2-control-reviewer` and `explainability-reviewer`
  are fixed (next section); `realtime-auditor` and `docs-reviewer` have not reviewed it, and no
  second round has run. See the *Review rounds* table.

**Stage 3, review round 1: what was fixed.** Run 2026-10-31 in the container, before the stage 3 commit.

| Finding | Fix | Evidence it can fail |
| --- | --- | --- |
| The sim controller YAML could drift from the hardware wiring | `sim/check_sim_config.py`, run by `verify_urdf.sh`: both YAMLs with every `gains` map removed must be equal | Renaming one `dof_names` entry in a copy, and in the real file temporarily: `FAIL  wiring differs at /pitch_pid_controller/ros__parameters/dof_names ...`; changing `update_rate` likewise; changing a gain stays green on purpose |
| Simulator defaults written in three places | The same script: every `<xacro:arg>` default equals `SimPlantParams`'s, every `SIM_ARGUMENTS` default equals the xacro's, the `<param>` names equal the struct's members, and the IMU consecutive-failure limit equals `ICM20948.hpp`'s | `imu_delay_ms` 20 to 30 in the xacro: `FAIL  default of 'imu_delay_ms': SimPlantModel.hpp says 20, rocko.urdf.xacro says 30` (and the launch file line); launch default to 25 and header limit 5 to 6 in copies: red |
| `use_sim_plant:=true use_mock_hardware:=true` was not expanded | A sixth expansion in `verify_urdf.sh`, with the same assertions and interface comparison | Removing `and not use_sim_plant` from the mock block: `FAIL  rocko sim=true mock=true: ... (got sim=1 named=1 motors=0 imu=0 mock=1 plugins=2)` |
| S1 baseline could pass vacuously (never released) | `verify_sim.sh` requires SimPlant's "releasing the simulated robot" log line, stamped at least `RELEASED_MIN_S` = 3 s before the probe stopped, in every run | Searching for a line that does not exist: all three runs `FAIL ... SimPlant never logged 'releasing the simulated robot'` |
| A failed run's evidence was deleted | `verify_sim.sh` deletes its run directory only on PASS and prints the path otherwise | The same broken run printed `Run directory KEPT as evidence (inside the dev container): /tmp/tmp.CZtjTOr2nr` (removed afterwards by hand) |
| Partial integer parameters (`20ms`) read as 20 | `SimPlant.cpp` rejects text left over after the number | `rocko_sim.launch.py imu_delay_ms:=20ms`: `SimPlant parameter 'imu_delay_ms' must be a whole number, got '20ms'` |
| Explainability (M1-M7 and lows) | Comments (default parameters are for the unit test, not the plugin; why the release hold; why RK4 at 1 ms; the two spellings of the motor speed, with a `static_assert` that the integer-division constant stays below 120 rad/s; the friction-through-belt factor is N_b squared), `on_init` split into `check_joint_interfaces()` and `read_parameters()`, per-parameter `invalid_reason()` messages, `rocko_sim_controllers.yaml` cleaned of dead lines and given units, `sim-plant.md` rewritten in the present tense | Reviewed by reading only |

One finding was *not* done as asked: the plant's IMU fault limit is not tied to `ICM20948.hpp` with a
`static_assert`. The constant is `private` in a class whose header needs ROS headers, and the model
header is deliberately ROS-free, so `sim/check_sim_config.py` compares the two numbers instead.

Results after the round 1 fixes (container, before the stage 3 commit): `build_ros.sh` exit 0 with nothing but
CMake notices in `log/latest_build/rocko_env/stderr.log` (0 compiler warnings); `verify_urdf.sh` PASS;
`verify_mock.sh` and `verify_mock.sh tank` PASS; `check_motor_plugin_compiles.sh` PASS;
`verify_sim.sh` PASS (S1 baseline 0.0000 deg, S1 command 8.2956 deg, S2a 0.442 s to 35 deg, each run
with the release logged); the host unit test prints `all SimPlantModel tests passed` and compiles with
no `-Wall -Wextra` warning; the hook tests pass (81 cases). Tier reached: 3, partly built (not called
built).

**Reproduce (stage 3).** After the commands in *Reproduce* below (from `check_container_mount.sh`
through the Tier 1 build), run from the repo root:

```bash
docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_urdf.sh       # Tier 0; 12 ok lines at the left margin (+4 indented), PASS
docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_sim.sh        # Tier 3, partly built; PASS, exit 0
g++ -std=c++17 -Wall -Wextra -I rocko_env/rocko_env/hardware/sim/include \
    -I rocko_env/rocko_env/hardware/sensors/icm20948/include \
    tools/sim_plant_model_test.cpp -o /tmp/sim_plant_model_test && /tmp/sim_plant_model_test   # host, not a tier
```

Battery voltage and surface do not apply (nothing ran on a robot). `verify_sim.sh` prints its results
to the terminal; the probe writes `samples.csv` and `metrics.json` to a folder inside the container.

## Verification

| Tier | Command | Result |
| --- | --- | --- |
| pre | `./check_container_mount.sh` | `ok` (container sees `/Users/ghart/projects/ROCKO-env`), exit 0 |
| 0, 1, 2 baseline (stage 2), before any change | `build_ros.sh`; `verify_urdf.sh`; `verify_mock.sh`; `verify_mock.sh tank`; `.claude/hooks/test-guard-protected-files.sh` | `Done!`; PASS; PASS; PASS; 81 cases ok. 2026-10-31, container as in the header |
| 1 build (stage 2), after the changes **and after review round 1's fixes** | `docker compose -f docker/compose.yml exec -T ros2 bash -c 'cd /ros2_ws && src/ROCKO-env/build_ros.sh'` | exit 0 (2026-10-31). No compiler warnings in `log/latest_build/rocko_env/stderr.log`; the `set_value` warning is gone. `Motor12VoltQuadEncoder.cpp` is still left out of this build |
| 0 static (stage 2), after the round 1 fixes | `docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_urdf.sh` | PASS |
| 2 mock (stage 2), after the round 1 fixes | `verify_mock.sh`; `verify_mock.sh tank` (same prefix) | PASS; PASS. The mock replaces the IMU plugin, so this says nothing about C-2. It does load and activate `AddFeedforwardController`, so the new code loads; it was not designed to show the NaN-to-0.0 behaviour and no check here looks for it |
| compile-only (new, not a tier) | `docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/check_motor_plugin_compiles.sh` | PASS, exit 0 (2026-10-31, rerun after the round 1 fixes). Syntax and types only; see the stage log. Seen to fail on four broken copies |
| host unit test (new, **not a tier**) | `g++ -std=c++17 -Wall -Wextra -I rocko_env/rocko_env/hardware/sensors/icm20948/include tools/imu_fault_tracker_test.cpp -o /tmp/imu_fault_tracker_test && /tmp/imu_fault_tracker_test` | prints `all ImuFaultTracker tests passed`, exit 0. It was also rerun on the laptop (Apple clang 21.0.0) while these docs were updated and passed, so it needs only a C++17 compiler. It proves the counting logic of one header (consecutive trip at the limit, a good read clearing the run, alternating failures tripping the window, the window clearing again, the total never reset, isolated failures never tripping, a custom window limit). It says nothing about the plugin, the sensor or the robot. Made to fail on three deliberate breaks, each going red on a named assertion: window trip disabled; `>=` changed to `>`; a good read no longer clearing the run |
| 1 build (stage 2), after **review round 2's fixes** | same `build_ros.sh` command as above | exit 0 (2026-10-31). No compiler warnings. The edited `AddFeedforwardController`, `ICM20948` and the new `static_assert`s compile. `Motor12VoltQuadEncoder.cpp` is still left out |
| 0 static, 2 mock (stage 2), after round 2's fixes | `verify_urdf.sh`; `verify_mock.sh`; `verify_mock.sh tank` | PASS; PASS; PASS (2026-10-31). Same limits as the round 1 rows: the mock replaces the IMU and motor plugins, so none of the round 2 logic is exercised, only that the controller still loads and activates |
| compile-only (not a tier), after round 2's fixes | `check_motor_plugin_compiles.sh` (command above) | PASS, exit 0 (2026-10-31), now on the **edited** `Motor12VoltQuadEncoder.cpp` with the new `write()`. Syntax and types only. I did not break the new `write()` on purpose and watch this fail; the script was shown to fail on four breaks of the earlier file |
| host unit test, after round 2 | same `g++ ... && /tmp/imu_fault_tracker_test` command | passes (2026-10-31). Run **by hand**: no script or tier runs it |
| none | the new `AddFeedforwardController` logic (NaN reset, 3-cycle hold, non-finite refusal, `on_deactivate` stop) and the new `write()` | **no automated test of either exists.** See review round 2 and *Not verified* |
| 0 host | `.claude/hooks/test-guard-protected-files.sh` | not rerun after the baseline: no hook file changed |
| 0 host | `python3 -W error -m py_compile` on `RateLimiter.py` | no `SyntaxWarning` |
| 3 sim, 4 robot | | not run in this stage. Tier 3 is not built; see the [ladder](../architecture/verification-ladder.md) |

## Review rounds

Stage 2, rounds 1 and 2 of at most three. The round 1 reviewers ran on the first version of the stage
2 change, the round 2 auditor on the version after round 1's fixes; the fixes are described in the
stage log above.

| Round | Reviewer | Findings | Fixed / declined (why) |
| --- | --- | --- | --- |
| 1 | `realtime-auditor` (opus) | (a) H-7 raised to Critical: an IMU `ERROR` leaves both motors driven by the last command while the gamepad still steers a blind robot. (b) M-5: a single transient `set_value()` failure returned `ERROR` and could permanently stop one wheel. (c) New, M-6: `_feedforward` was uninitialised and written by one thread while read by another. Also found in round 1 (which reviewer raised each of the next items was not recorded): (d) C-2: a sensor that alternates good and bad reads never reached the consecutive-failure limit (H-8). (e) A garbled sensor value (a huge angle) was accepted as a good read. (f) A wrong comment. (g) Not fixable here: a frozen or reset BNO055 that still reads valid-looking numbers (H-9). (h) Not fixed: D-term spike after held cycles (L-5). (i) Note for C-1: a timeout branch must remove the pending request | (a) **Partly fixed**: the controller resets its reference to NaN after each read and writes `0.0` for a NaN or non-finite reference; whether `0.0` reaches the PWM depends on H-2 in the motor plugin, which was scheduled with H-4 in stage 4 (pulled forward after round 2), so row 1.7 stayed open (*partly done*). (b) Fixed: `ERROR` only after `kMaxConsecutiveWriteFailures = 5` consecutive failed cycles. (c) Fixed: `std::atomic<double>` initialised to 0.0, a non-finite message is ignored. (d) Fixed in `ImuFaultTracker`: also trips on 10 failures among the last 20 reads. (e) Fixed: `|pitch|` or `|roll|` > 180 deg, or `|yaw|` > 360 deg, counts as a failed read. (f) Fixed. (g) **Declined for now**: detecting it needs the read in C++ (C-1) and a plausibility check on the data; recorded as H-9. (h) **Declined for now**: recorded as L-5. (i) Recorded as a note under C-1 |
| 1 | `ros2-control-reviewer` (opus) | (a) The new code may need a newer Jazzy `ros2_control` 4.x than `find_package(controller_manager 4.0.0)` allows (`set_value()` returning `bool`; the `.hpp` `realtime_tools` headers). Plausible, not proven. (b) What `controller_manager` does after an `update()` returns `ERROR` depends on the ros2_control version. (c) The C-2 procedure called `ERROR` a lifecycle state | (a) **Not fixed, by design**: raising the CMake minimum is a version-pin change, which is a human's decision, so no proposal is written in this change; recorded in *Risks*, the robot procedures (record versions, `apt upgrade` first) and *Not verified*. (b) Added robot steps to the C-2 and M-5 procedures. (c) Fixed: the wording now says `ERROR` is a return code and the expected state is unverified |
| 1 | `explainability-reviewer` (sonnet) | Missing units and sign on the `.srv` fields; magic numbers in the throttle calls; `read()` harder to follow than it needed to be; `pitch_offset = 7.6` in `ICM20948.py` unexplained | Fixed: units and the "sign of pitch not established" note on the `.srv`; `kLogThrottleMs` (ms) in both files; `read()` simplified; a comment on `pitch_offset` saying it is in degrees and that its origin is undocumented (the value was not changed) |
| 2 | `realtime-auditor` (opus) | **High:** (a) an interaction with H-2: writing `0.0` on the fault paths flipped the motor direction pin (the old code treated `0.0` as forward) while the old duty stayed latched, because the `pwmVal > 2` guard skipped the speed write. **Medium:** (b) one missed upstream write produced a one-cycle (10 ms) zero pulse; (c) `on_deactivate` wrote no safe state (H-7); (d) the `AddFeedforwardController` logic has no automated test. **Low, four:** (e) comments that did not match the code; (f) a missing `static_assert` on the IMU limits; (g) a range comment that missed the pitch offset; (h) the findings document was out of date. **Plausible, not proven:** (i) in-range garbage from a bus fault, an all-equal Euler triple, would pass the range check | (a) **Fixed, pulled forward** from stage 4: `write()` always writes the speed pin, a non-finite or sub-deadband command writes 0 % with the direction pin untouched, duty clamped to [0, 100] (H-2, M-1, M-3 code; see *Deviation*). (b) **Fixed:** a NaN reference repeats the last finite command for at most `kMaxStaleReferenceCycles = 3` cycles, then writes `0.0`. (c) **Fixed:** `on_deactivate` writes `0.0` to every command interface. (d) **Declined as a gtest this round**: the Tier 3 fault scenarios are planned to exercise it end to end, and Tier 3 is not built yet, so **nothing tests it today**; the tracker test is run by hand. (e), (f), (g) **Fixed** in the code comments and the `static_assert`. (h) **Fixed** in this documentation sync. (i) **Fixed:** `ICM20948.py` rejects an all-equal Euler triple; not seen on a real sensor |
| 3 | `realtime-auditor` (opus) | CLEAN: no Critical/High/Medium. Lows: the motor `write()` direction/duty interaction is closed; a comment named the wrong upstream controller; no test for AddFeedforward's stop logic | Fixed the comments. **Declined, with why:** a gtest for AddFeedforward's NaN/stale logic (it is exercised end to end only once the sim fault scenarios exist; recorded as a blind spot in the ladder); the H-7 watchdog thread idea (row 1.7 stays open) |
| stage 3, 1 | `ros2-control-reviewer`, `explainability-reviewer` | Drift of the sim YAML and of the three copies of the defaults; a vacuous S1 baseline; failed-run evidence deleted; the missing sim+mock expansion; partial integer parse; explainability items M1-M7 | Fixed (see *Stage 3, review round 1* above), except the `static_assert` tie to `ICM20948.hpp` (the constant is private; a script compares the numbers instead) |
| stage 3, 2 | `ros2-control-reviewer` (opus), `realtime-auditor` (opus), `explainability-reviewer` (sonnet), `docs-reviewer` (sonnet) | ros2: the drift checker ignored the whole `gains` map, never tied a `<param>` to its xacro arg, was type-blind, could still pass the S1 baseline without a release, and tank mode ignored `use_sim_plant` silently. Realtime: only Lows (sub-step remainder not carried, `invalid_reason()` gaps, a non-finite IMU sample counted as good, no upper bound on the IMU delay). Explainability: CLEAN, Lows. Docs: stale counts, a stale blind-spot bullet, wrong line citations, a false 'reconciled' sentence | All fixed (checker rewritten and shown red on each mutation; remainder carry; stronger `invalid_reason()` with unit test #17; the `last_stamp_s` release check; a seventh `verify_urdf.sh` expansion). The first attempt at this round hit the account's usage limit and was rerun |
| stage 3, 3 | `ros2-control-reviewer` (opus), `realtime-auditor` (opus), `docs-reviewer` (sonnet) | ros2: **CLEAN**. Six Lows: the checker still misses a duplicated `<xacro:arg>` (xacro keeps the first, the checker the last), overrides typed into `rocko_sim.launch.py`, struct members not written as `type name = value;`, odd or duplicate `<param>` tags, a crash mislabelled as drift, and a mis-pointed row in `read_parameters()`. Realtime: **CLEAN**; a negative period would become owed time; the C-2 fault branch of the sim has never been run end to end. Docs: one Medium (a stale expansion count in the ladder's Tier 0 row) and Lows | Fixed: the stale counts and wording, a negative period now counts as zero. **Declined for this round (cap of three), recorded as known limits in the ladder's blind spots:** the six checker gaps. The C-2 fault scenario is stage 4's first job. `CLAUDE.md` line 177 still says 'all four modes' (a human-reviewed file; flagged for the maintainer) |

**An honest note about the guard hook.** While making the round 1 fixes, the guard hook denied a first
`Write` of `ImuFaultTracker.hpp` because the new file contained fewer safety words (`fault`) than the
version it replaced. The author rewrote the file keeping the word `fault`, and the hook then passed.
The hook only counts words, so its passing proves nothing about whether the safety logic was kept
intact. What does check the logic is the host unit test above (the counting rules) and the
`realtime-auditor`'s round 2 read of the file. The same limit applies to every safety edit after it: the hook passing says nothing about whether the logic is right.

## Not verified — needs a human

Stage 2 only; stage 3's list (and its reproduce commands) is in its own subsection of the stage log. The robot procedures are in the stage log above.

- **Anything on a Pi or a robot.** The motor plugin build with real WiringPi, the IMU node against a
  real BNO055, and every failure path (the IMU fault, a failed `set_value()`) have not run.
- **That the new code builds against the Jazzy on the Pi.** It may need a newer `ros2_control` 4.x
  than CMake's `find_package(controller_manager 4.0.0)` minimum. Plausible, not proven. If it does,
  raising that minimum is a version-pin change (a proposal; none written). Record
  `ros2 pkg xml hardware_interface | grep '<version>'` and the same for `realtime_tools`, in the
  container and on the Pi, and run `sudo apt update && sudo apt upgrade` before building on the Pi.
- **What the robot does after an `ERROR`.** What `controller_manager` does after `read()` or
  `update()` returns `ERROR` depends on the ros2_control version, and nobody has observed it (H-7,
  row 1.7). Whether the wheels stop also depends on the new motor `write()` (H-2 code, pulled forward),
  which has only been type-checked. Anything this entry says about Jazzy 4.x behaviour after an
  `ERROR` is from the author's memory, not from a test.
- **The proposed limits.** All of them are in the *Proposed safety and hardware values* table above:
  the deadband and stiction offset in the motor plugin, the stale-reference hold and write-failure
  limit in the feedforward controller, and the IMU failure and plausibility limits. A human confirms or
  changes them on the robot. The proposal for them is to follow.
- **That the new `write()` does what it says.** No tier built or ran it. See the *Robot procedure for
  the motor `write()`*. Nobody has seen a wheel stop at a zero command, the direction pin stay still
  near zero, or the duty stay at or below 100 %.
- **The `AddFeedforwardController` stop logic.** No automated test exists (round 2, declined as a
  gtest). It is to be exercised by the Tier 3 fault scenarios, which are not built.
- **That rejecting an all-equal Euler triple is safe.** It is a guess about what a fault returns, and
  it would also reject a genuine reading with three identical values. With 1/16 deg resolution that is
  unlikely, but it has not been tried on a sensor.
- **How long a failed IMU read takes.** Five reads in a row is 50 ms only if each fails quickly; a
  held I2C bus may block one read for up to about 1 s (kernel timeout, not measured).
- **The sign and origin of pitch.** Not established in the repo; the `.srv` and `pitch_offset` now say
  so. A plausibility limit of 180 deg on pitch does not depend on the sign.
- **That `std::atomic<double>` is lock-free on the Pi.** It compiled in the arm64 container; the Pi
  was not tried.
- **That RViz opens** with the installed layouts, and that `view_robot.launch.py` runs (rows 0.5, 0.6).
- **The motor plugin's build.** `check_motor_plugin_compiles.sh` checks syntax and types against
  hand-copied stubs only.
- **Not fixed, recorded:** C-1 (blocking service call; encoders as well as the IMU), H-9 (a frozen
  sensor, or a reset one that returns differing values, is not detected), L-1, L-5 (D-term spike after
  held cycles), and the integer division in `MAX_RAD_PER_SEC`
  ([L-7](../claude-analysis/balance-loop-findings.md#l-7--max_rad_per_sec-divides-integers); changing it
  alters duty scaling by about 0.9 %, so a human confirms). Also L-6: a failed encoder read in
  `Motor12VoltQuadEncoder::read` and `QuadEncoder::read` returns OK with the old values (hidden today by
  C-1's missing timeout).
- **Review round 3** is pending; its results are to be added to the table above by the person.

## Reproduce

Stage 2 only; stage 3's commands are at the end of its subsection in the stage log. Commit: the commit that adds this file (built on `3882790`).
No SHA is written here because the file cannot name the commit that contains it; read it from
`git log -- docs/journal/2026-10-31-sim-plant-and-critical-fixes.md`. Environment: the
`rocko-env-dev` container (`ros:jazzy`, arm64 on Docker Desktop for macOS) for everything marked
container; the laptop for the rest. The mock is not a simulation, and nothing was run on a robot.
Run from the repo root, in this order:

```bash
docker compose -f docker/compose.yml up -d
./check_container_mount.sh                                                                    # host; expect ok, exit 0
docker compose -f docker/compose.yml exec -T ros2 bash -c 'cd /ros2_ws && src/ROCKO-env/build_ros.sh'   # Tier 1; exit 0
docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_urdf.sh       # Tier 0; PASS
docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_mock.sh       # Tier 2; PASS
docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_mock.sh tank  # Tier 2; PASS
docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/check_motor_plugin_compiles.sh   # compile-only, not a tier; exit 0
.claude/hooks/test-guard-protected-files.sh                                                   # Tier 0, host; 81 cases ok (baseline only)
g++ -std=c++17 -Wall -Wextra -I rocko_env/rocko_env/hardware/sensors/icm20948/include \
    tools/imu_fault_tracker_test.cpp -o /tmp/imu_fault_tracker_test && /tmp/imu_fault_tracker_test   # host, not a tier
```

Expected: the results in the *Verification* table. Not recorded for these runs: battery voltage and
surface (no robot was involved) and a log location (output was read from the terminal; the build log
is `log/latest_build/rocko_env/stderr.log` in the container workspace).

## Where the run stands

Stages 1 to 3 are done, reviewed in three rounds each, committed and pushed. Stage 3 is **not**
'Tier 3 built': S1 and S2a hold, S2b and the three scenarios do not exist. The first attempt at stage
3's second review round was cut off by the account's usage limit and was rerun; nothing in the code was
affected. See the stage 4 and later entries (if any) below for what came next.
