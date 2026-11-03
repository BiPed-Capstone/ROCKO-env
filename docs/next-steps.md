# Next steps

**This is the work queue.** When you have no specific goal, take the first row below that you can do
and check. It owns three things no other page does: the **order**, **who** can do each item, and the
**status**. It deliberately does not repeat what a flaw *is* or how to fix it; each row links to the
page that does, so that fact has one home.

Checked against the code on 2026-10-28 (commit `059e602`). Two GitHub issues that were already fixed
(#15, #17) have been closed; others are only partly met, or fixed on this branch only. The
[last section](#github-issues-against-the-code) says which. Rows 0.1 to 0.6, row 1.4 and the new rows 1.7 to
1.11 were updated on 2026-10-31 for the work in the
[stage 2 journal entry](journal/2026-10-31-sim-plant-and-critical-fixes.md), built on `3882790` plus
the stage 2 commit, and again after its review rounds 1 and 2.
Rows 2.1 and 2.3 and the #12 line were updated on 2026-10-31 for the
[stage 3 entry](journal/2026-10-31-sim-plant-and-critical-fixes.md#stage-3--the-plant-simplant-tier-3-partly-built)
(`SimPlant`, built on `1318f95`, committed as the stage 3 commit).

## How to use it

1. Pick the first row you can do (see *Who* and *Best check today*). Read its source in full before
   you start.
2. Follow the loop in [`CLAUDE.md`](../CLAUDE.md). Say in the journal which tier can check the
   change; anything only a robot can confirm goes in the hand-off procedure.
3. In the **same commit**, update the row's status here and mark the item fixed in its source (the
   finding, the issue note, the backlog row).
4. Found a new flaw? Add it to the [findings](claude-analysis/balance-loop-findings.md) and add a row
   here.
5. Gated change (gains, version pins, weakening a safety behavior, the guardrails)? Write a
   [proposal](proposals/README.md) and put its number in the row.
6. **Nobody watching** (a headless run, started with `claude -p`, or the person said they are away)?
   Take only rows in [Wave 0](#wave-0--do-now-nothing-blocks-these) that are *open* or *partly done*,
   whose *Who* starts with `agent`, and whose *Best check today* names a tier (not `—` or `none`): nothing could check
   the others, and "done" without a check is what `CLAUDE.md` §4 forbids. As of 2026-10-31 rows 0.1
   to 0.6 are done (0.1, 0.3, 0.5 and 0.6 awaiting a robot or a person), so what is left is 0.9. Work them in order. Wave 1 and later only when the person names the row. For an
   `agent + robot` row, write the code, run the checks, write the robot procedure into the journal
   entry, and set the status to *done, awaiting robot*. If what is left belongs to another row (0.1's
   end state needs 1.1), say so in the row; the status is still *done, awaiting robot*. Then go on to
   the next row.

## Reading the table

- **Who.** *agent*: an AI can write it and check it as far as the
  [verification ladder](architecture/verification-ladder.md) allows. *agent + robot*: an AI writes
  it, but only a person on the Pi can confirm it. *human*: needs a measurement or a decision.
- **Status.** A row's status starts with one of these, so a loop can tell where it stands: *open*
  (not started), *partly done* (some landed; the row says what is left), *done, awaiting robot* (the
  code landed and was checked as far as the ladder allows; a person has to run the procedure in the
  journal entry), *blocked on human* (agent work that waits on a decision or a measurement; the row says which),
  *done*. Change the word in the same commit that changes the row's state.
- **Best check today** is the highest tier that can say anything about the change, per the ladder.
  Row 0.2 added a compile-only check of the motor plugin (syntax and types, against stub headers); it
  is not a tier, and the plugin is still not built off a Pi. Tier 3 is partly built (row 2.3): a change can be run against a model with
  estimated parameters, which is not a check against the robot's dynamics, and only the robot (Tier 4) shows whether
  anything balances.
- Files on the 100 Hz path (`CLAUDE.md` §6) make `realtime-auditor` mandatory.

## Wave 0 — do now, nothing blocks these

Ordered by what can be done and checked now, **not** by risk, so it runs ahead of the audit's steps
2 to 5, which need a Pi or row 0.2 (now done). The audit's own order is not changed: C-2 (its step 1) is still
first here, and Wave 1 keeps its steps 2 to 5 in sequence.

| # | Item | Who | Best check today | Status and what to know |
| --- | --- | --- | --- | --- |
| 0.1 | **C-2**: a failed IMU read must not report pitch 0.0 (audit step 1) | agent + robot | T1 (service, node and plugin compile); a host unit test of the counting logic (not a tier) | done, awaiting robot. The `Icm20948Data.srv` response has a `success` field; the node answers `success = false` with NaN angles when a read fails (and reads the Euler angles once, which partly covers H-3); the plugin holds the last good angles, counts failures in `ImuFaultTracker`, and returns `ERROR` when 5 reads in a row failed **or** 10 of the last 20 failed ([H-8](claude-analysis/balance-loop-findings.md): an alternating fault); a garbled angle (`|pitch|` or `|roll|` > 180 deg, `|yaw|` > 360 deg) counts as a failed read, and the node rejects a reply whose three Euler values are all equal (a guess at what a bus fault or a reset returns; not seen on a sensor). The limits are **proposed**; a human confirms them (table under *Decisions waiting on the maintainer*). Checked: the Tier 1 build compiles it with no compiler warnings; `tools/imu_fault_tracker_test.cpp` passes and was seen to fail on three deliberate breaks. The mock replaces this plugin, so Tier 2 passes but cannot see it. Not checked: anything on a Pi, or the fault path through `read()`. C-1 is **not** fixed, so a dead `icm20948` node still blocks `read()` forever. [H-9](claude-analysis/balance-loop-findings.md) (a frozen sensor, or a reset one that returns differing values, is undetected, row 1.8) is open. What happens to the wheels after the `ERROR` is open: [H-7](claude-analysis/balance-loop-findings.md) (row 1.7). Procedure: the [stage 2 journal](journal/2026-10-31-sim-plant-and-critical-fixes.md). End state: [C-2](claude-analysis/balance-loop-findings.md), reached with 1.1 |
| 0.2 | **Compile-only check for the motor plugin** ([`bootstrap-sim`](../.claude/skills/bootstrap-sim/SKILL.md) step 3) | agent | T1 (new check) | done. `check_motor_plugin_compiles.sh` type-checks `Motor12VoltQuadEncoder.cpp` with `g++ -std=c++17 -fsyntax-only` against stub `wiringPi.h` and `softPwm.h`. It passed on the unmodified file (2026-10-31), failed, for the right reason, on four broken copies, and passed again on the edited file with the new `write()` (row 1.4, after review round 2). It proves syntax and types only: the stubs are hand-copied, so the real WiringPi signatures are unchecked, and nothing is linked or run. **Unlocks 1.3 and 1.4** (only to that extent). Six findings cite this file (C-1, H-1, H-2, M-1, M-2, L-1) and a seventh, M-3, is about its `pwmVal`. Run it as the [ladder](architecture/verification-ladder.md) says |
| 0.3 | **M-5** (and **M-6**): unchecked `set_value()` result; uninitialised, unsynchronised feedforward value | agent | T1 compile, T2 loads. The failure path cannot be provoked on the mock | done, awaiting robot. [M-5](claude-analysis/balance-loop-findings.md), [M-6](claude-analysis/balance-loop-findings.md). The controller checks every `set_value()` result, still tries each interface, and returns `ERROR` with a throttled log only after 5 consecutive failed cycles (**proposed** limit; one transient failure does not stop a wheel). `_feedforward` is now `std::atomic<double>` initialised to 0.0 (M-6), and non-finite messages are ignored. The same file also carries the controller half of H-7 (row 1.7): it always writes; a NaN reference repeats the last finite command for at most 3 cycles (**proposed**) and then writes `0.0`; a non-finite command is refused and written as `0.0`; `on_deactivate` writes `0.0`. Checked: the Tier 1 build has no compiler warnings; Tier 2 passes. The failure path and the new stop logic were not exercised by any tier, and **no automated test of the stop logic exists** (a gtest was declined in review round 2; the Tier 3 fault scenarios are meant to cover it, and they do not exist yet: Tier 3 is only partly built). What `controller_manager` does after an `update()` `ERROR` depends on the ros2_control version and is unobserved. Critical path |
| 0.4 | **L-3**: deprecated include; invalid escapes in `RateLimiter.py` | agent | T1 (warnings gone) | done. [L-3](claude-analysis/balance-loop-findings.md). The include is `realtime_buffer.hpp` and the two docstrings in `RateLimiter.py` are raw. Checked: the Tier 1 build has no warnings from either; `python3 -W error -m py_compile` on the host raises no `SyntaxWarning`. The include is in the controller header, so the auditor is mandatory |
| 0.5 | **L-4**: RViz layouts are never installed | agent | T1 (`install/.../share/rocko_env/rviz/` exists). That RViz opens with them needs a display, so a person | done, awaiting robot (a person with a display runs a procedure; no robot is needed). [L-4](claude-analysis/balance-loop-findings.md). The description `install(DIRECTORY ...)` block now lists `rocko_env/description/rviz`. Checked: after the build, `install/rocko_env/share/rocko_env/rviz/` holds `rocko.rviz` and `rocko_view.rviz`. Also checked, by reading: `view_robot.launch.py` looks for `rviz/rocko_view.rviz` under the `rocko_env` share folder, which is where the install block puts it. Not checked: that RViz opens with them (needs a display). Procedure: "Procedure for a person with a display" in the [stage 2 journal](journal/2026-10-31-sim-plant-and-critical-fixes.md#stage-2--foundations-rows-02-01-03-04-05-06-and-the-motor-write-part-of-14) |
| 0.6 | [#11](https://github.com/BiPed-Capstone/ROCKO-env/issues/11): `view_robot.launch.py` default argument | agent | T1, plus a manual look that the default now resolves to an installed file (needs 0.5) | done, awaiting robot (a person with a display runs a procedure; no robot is needed). The default of `description_package` is now `"rocko_env"`, and `description_package:=rocko_env` still works. Checked: Tier 1 builds it; and, by reading, the default resolves to an installed path (the share folder of `rocko_env`, with the layouts from row 0.5). Not checked: the launch file was not run, with or without a display. Procedure: "Procedure for a person with a display" in the [stage 2 journal](journal/2026-10-31-sim-plant-and-critical-fixes.md#stage-2--foundations-rows-02-01-03-04-05-06-and-the-motor-write-part-of-14) |
| 0.7 | [#16](https://github.com/BiPed-Capstone/ROCKO-env/issues/16): the wheel radius disagrees | human measures, agent finishes | T2 | blocked on human. There are **four** values, not three: `0.06` m (the Python nodes), `0.127` m (the tank config) and `0.015` m (`wheel_radius` in `rocko_description.urdf.xacro`, which a Tier 3 model would use). An agent cannot know which is right. Measure once (the same measurement serves 2.2), then keep a single source |
| 0.8 | [#14](https://github.com/BiPed-Capstone/ROCKO-env/issues/14): unpinned Python dependencies | agent + robot | — | blocked on human: no requirements file exists. The versions must come from a working Pi, otherwise the pins are invented |
| 0.9 | [#13](https://github.com/BiPed-Capstone/ROCKO-env/issues/13): no tests in the packages | agent | T0, T2 | partly done on `ai-forward` by `verify_urdf.sh` and `verify_mock.sh`, which are scripts, not `colcon test`. Still missing: unit tests for `RateLimiter.py`, and running the checks under `colcon test` or CI |

## Wave 1 — the control-loop fixes, in the audit's order

The audit's steps 2 to 5, from its [order of work](claude-analysis/balance-loop-findings.md). All of
it is on the critical path. The audit asks that steps 1 to 5 be shown to reduce the spread of loop
period and feedback age *before* any tuning time is spent.

| # | Item | Who | Best check today | Status and what to know |
| --- | --- | --- | --- | --- |
| 1.1 | **C-1 (IMU)**, with the **UART migration** and **H-3**; the IMUPLUS mode and quaternion read in [`bno055-improvements.md`](claude-analysis/bno055-improvements.md) §§ 3–4 fold in | agent + robot | T1 compile | open. Needs a wiring change and a Pi. When a timeout is added to the service call, its timeout branch must remove the pending request, or the client's pending list grows ([C-1](claude-analysis/balance-loop-findings.md) note). The sensor-health check for H-9 (row 1.8) can only be written here, once the read is in C++. H-3 is partly done: `ICM20948.py` reads the Euler angles once (row 0.1); the single burst read in C++ is not. That doc also covers the hardcoded pitch offset (`7.6` in `ICM20948.py`) |
| 1.2 | **H-6**: gyro rate for the D term | agent + robot | T1; T2 only once the mock block also exports the new interface | open. The axis and sign must be established on the robot: read the finding first. Changing a gain is gated |
| 1.3 | **C-1 (encoders)** | agent + robot | the compile-only check from **0.2** (syntax and types of the motor plugin only) | open |
| 1.4 | **H-2, M-1, M-3**, and **L-1** (same file) | agent + robot | the compile-only check from **0.2** (syntax and types only) | partly done. Code landed for [H-2](claude-analysis/balance-loop-findings.md), [M-1](claude-analysis/balance-loop-findings.md) and [M-3](claude-analysis/balance-loop-findings.md) in `Motor12VoltQuadEncoder.cpp`: `write()` writes the speed pin every cycle, a non-finite or sub-0.05 rad/s command writes 0 % with the direction pin untouched, duty is `clamp(\|cmd\| / MAX_RAD_PER_SEC * 100 + 2.0, 0, 100)` percent, and the per-cycle log is gone. **Pulled forward** from the person's stage order (H-2 was stage 4, M-1/M-3 stage 6) after review round 2, because the stop paths of C-2, M-5 and H-4 do nothing if the motor write is skipped; status of that part: compile-checked only (type and syntax), awaiting robot. The deadband and the 2 % offset are **proposed**. Not done: **L-1** and **C-1 (encoders)** (row 1.3). Checked: the compile-only check passes on the edited file. Not checked: a build against real WiringPi, or anything running. Robot procedure: the [stage 2 journal](journal/2026-10-31-sim-plant-and-critical-fixes.md) |
| 1.5 | **M-2**: software PWM period | human | — | open. Moving to hardware PWM means moving a wire. Obviated by a motor-driver change; see below |
| 1.6 | **L-2**: rename `ICM20948` to the BNO055 | agent | T0, T1. A plugin-name mismatch between the plugin XML and the xacro would show only on a Pi | open. Touches the plugin XML, the URDF and the launch files, and `verify_urdf.sh` expects the current names. Do it while 1.1 has the plugin open |
| 1.7 | **H-7**: the motors keep their last command when the cascade stops (new, found 2026-10-31; raised to Critical by the `realtime-auditor`) | agent + robot | T1 compile, T2 loads; the NaN path is unchecked. T3 once built. Unconfirmed on hardware | partly done. [H-7](claude-analysis/balance-loop-findings.md). Done in code (partly mitigated; unconfirmed on hardware): `AddFeedforwardController` sets its chained reference back to NaN after reading it each cycle, repeats the last finite command for at most 3 cycles of NaN (**proposed**) and then writes `0.0`, and writes `0.0` in `on_deactivate`; and the motor plugin's `write()` now writes the speed pin every cycle (row 1.4), so that `0.0` reaches the PWM as 0 % duty instead of leaving the previous duty latched. Left: all of it is unrun on a robot and the motor part is only type-checked; what `controller_manager` does after an `ERROR` in Jazzy 4.x is not known (the author's recollection is from memory, not a test); and the H-4 tipover cutoff (2.6) is not written. Critical path, so the auditor is mandatory. Until the robot procedure has been run, do not assume row 0.1's `ERROR` return makes the robot stop |
| 1.8 | **H-9**: a frozen or reset BNO055 that still reads valid-looking numbers is undetectable (new, found 2026-10-31) | agent + robot | none yet | open. [H-9](claude-analysis/balance-loop-findings.md). Needs the read in C++, so it is done with row 1.1 (C-1). A brownout could reset the sensor to configuration mode, which would read as a constant `pitch_offset` lean. One variant is now partly covered: `ICM20948.py` rejects a reply whose three Euler values are all equal, which would catch an all-zero block; a frozen sensor that returns its last real orientation is not caught. Critical path |
| 1.9 | **L-5**: D-term spike after cycles in which the IMU angle was held (new, found 2026-10-31) | agent + robot | none yet | open. [L-5](claude-analysis/balance-loop-findings.md). Removed by row 1.2 (H-6, the D term from the gyro rate). Unmeasured |
| 1.10 | **L-6**: a failed encoder read is reported as a good one (`Motor12VoltQuadEncoder::read`, `QuadEncoder::read`; new, found 2026-10-31) | agent + robot | T1 compile-only (the compile-only check from **0.2** for the motor plugin; the Tier 1 build for `QuadEncoder`) | open. [L-6](claude-analysis/balance-loop-findings.md). Both `read()`s return OK and keep the old position and velocity when the encoder call fails, with no fault flag and an unthrottled error log. Hidden today because the blocking call (C-1) has no timeout; it becomes a C-2-class bug the moment C-1 adds one. Do it in the C-1 encoder change (row 1.3), with the same pattern as `ICM20948::read` (tracker, `ERROR`, throttled log). Critical path |
| 1.11 | **L-7**: `MAX_RAD_PER_SEC` divides integers (new, found 2026-10-31) | agent + robot | T1 compile-only (the compile-only check from **0.2**) | open. [L-7](claude-analysis/balance-loop-findings.md). `1150 / 60 * ...` evaluates to 119.38 rad/s, not 120.43 rad/s. Fixing it changes duty scaling by about 0.9 %, so a human confirms before it is changed (a proposal first). Critical path |

## Wave 2 — simulation, then the safeguards that should be built against it

| # | Item | Who | Best check today | Status and what to know |
| --- | --- | --- | --- | --- |
| 2.1 | **Choose the Tier 3 plant**: Gazebo ([ADR 0003](decisions/0003-simulation-target.md)) or `SimPlant` ([M-4](claude-analysis/balance-loop-findings.md)) | human decides | — | partly done. Choice made: `SimPlant` first (the person's instruction for the 2026-10-31 run), and it is built (row 2.3). Gazebo is not ruled out for later. **Outstanding:** the proposal to amend ADR 0003, which still says Gazebo and stays `Proposed`; it is still to be written |
| 2.2 | **Measure the robot** (`bootstrap-sim` step 4) | human | — | open (human work). Every `SimPlant` parameter is an estimate until this is done (see row 2.3). The agent writes `docs/guides/measure-inertia.md` first. Share the wheel measurement with 0.7 |
| 2.3 | **Build the plant and the scenarios** (`bootstrap-sim` steps 6–7) and **the sim-to-real gap page** (step 8) | agent | T3 (partly built) | partly done. **Step 6 is done in the model:** the `SimPlant` plugin, `rocko_sim.launch.py` and `rocko_sim_controllers.yaml` exist, and `verify_sim.sh` checks S1 (the real cascade loads and goes active against it; pitch responds to the wheels) and S2a (with no torque the simulated robot passes 35 deg within 0.48 s). **Step 7 is not done:** there is no upright, impulse or velocity-step scenario and no S2b (a one-sign-flipped copy of the upright scenario must fail), so Tier 3 is **not** called built. Every physical parameter is an estimate, because row 2.2 (measuring the robot) needs a human and has not been done: results are about the model, not the robot. Step 8 (the gap page) is not written, though [sim-plant.md §6](architecture/sim-plant.md) lists what the model leaves out. One observation: with the unchanged hardware gains the model fell by a slow drift after about 10 s, no growing oscillation observed (the implementer's report of one run whose exact command was not recorded; model result only, no gain changed). [M-4](claude-analysis/balance-loop-findings.md) is partly done (the mock, the off-Pi build and now the physics exist as a model), and [#12](https://github.com/BiPed-Capstone/ROCKO-env/issues/12) is the same item. Details and the not-verified list: the [stage 3 journal](journal/2026-10-31-sim-plant-and-critical-fixes.md#stage-3--the-plant-simplant-tier-3-partly-built) |
| 2.4 | **Let the loop run the sim** (`bootstrap-sim` step 9) | agent writes a proposal, human approves | — | open. Needs 2.3 first; the agent then writes a proposal, because it edits `.claude/settings.json`, a guardrail (`CLAUDE.md` §3) |
| 2.5 | **C-3**: teleop watchdog | agent writes, human sets the threshold | none yet (Tier 3 is only partly built and has no scenario for this). Tier 2 may show the velocity reference dropping after the timeout, since the mock runs `Joystick.py` (untried) | open. The right failsafe is counter-intuitive: read [C-3](claude-analysis/balance-loop-findings.md) before writing any code |
| 2.6 | **H-4**: tipover cutoff | agent writes, human sets the threshold | none yet (Tier 3 is only partly built and has no scenario for this) | open. *Adding* a safeguard is allowed; its threshold goes in a proposal |

## Wave 3 — needs the robot

| # | Item | Who | Status and what to know |
| --- | --- | --- | --- |
| 3.1 | **H-5**: real-time scheduling and CPU isolation | human | open. Verify with a histogram of loop periods, not an average |
| 3.2 | **H-1**: close the wheel-velocity loop | agent + robot | open. Obviated by a motor-driver change; see below |
| 3.3 | **Retune the gains, last** ([`tune-gains`](../.claude/skills/tune-gains/SKILL.md)) | human | open. Gains are gated |

**A hardware decision is pending** (findings, "Related hardware work"): brushless motors with ODrive
controllers. It would obviate H-1 and M-2 and partly C-1. The audit says **not to wait for it**: the
other fixes are independent, and several are needed to judge it.

## Decisions waiting on the maintainer

Not agent work. Each is recorded where it was raised.

- The `settings.json` allow-list (it can run code around the hook) and whether `CLAUDE.md`, the
  agents and the skills should be guarded: [ADR 0005](decisions/0005-ai-forward-development.md).
- What to allow, deny and enforce for a run nobody is watching: the permission mode, the loop's
  commands, a narrower `gh`, a deny list, git hooks and a rule on `main`:
  [proposal 0002](proposals/0002-unattended-run-settings.md): A, B, C and D are applied (2026-10-30);
  E and F are open. Guarding `CLAUDE.md`, the agents and the skills is still undecided and matters
  more now: under `acceptEdits` edits to them are expected not to prompt (not tried).
- A PyYAML-based gain check, because the hook reads lines, not YAML: the hook's docstring.
- The single-file `build_ros.sh` mount in `docker/compose.yml`, which goes stale: the ladder's blind
  spots and the [2026-10-27 journal](journal/2026-10-27-ai-forward-bootstrap.md).
- Whether the dev container should be one per checkout (a compose project name per checkout and no
  fixed `container_name`, so a worktree gets its own container and build volume), or whether
  `check_container_mount.sh` is enough: the ladder's
  [blind spots](architecture/verification-ladder.md#blind-spots-stated-plainly). (The script has been
  on the allow-list since 2026-10-30: [proposal 0001](proposals/0001-allow-container-mount-check.md).)
- Amending ADR 0003 to match `SimPlant` first (2.1): decided by the person for the 2026-10-31 run; the proposal is to follow.
- The proposed safety and hardware values below. None was measured or chosen by a human; each is an
  edit-and-rebuild constant, and a human confirms or changes it on the robot. The proposal for these
  hardware thresholds is to follow (not written yet). The journal holds the same table with the
  meaning of each: [stage 2 journal](journal/2026-10-31-sim-plant-and-critical-fixes.md).

  | Name | Value | File |
  | --- | --- | --- |
  | `kDeadbandRadPerSec` | 0.05 rad/s | `Motor12VoltQuadEncoder.hpp` |
  | `kStictionDutyPercent` | 2.0 % of full duty | `Motor12VoltQuadEncoder.hpp` |
  | `kMaxStaleReferenceCycles` | 3 cycles (30 ms nominal at 100 Hz) | `AddFeedforwardController.hpp` |
  | `kMaxConsecutiveWriteFailures` | 5 cycles | `AddFeedforwardController.hpp` |
  | `kMaxConsecutiveImuFailures` | 5 reads (50 ms nominal) | `ICM20948.hpp` |
  | `kMaxWindowImuFailures` | 10 of the last 20 reads | `ICM20948.hpp` (default from `ImuFaultTracker.hpp`) |
  | `kMaxAbsPitchRollDeg`, `kMaxAbsYawDeg` | 180 deg, 360 deg | `ICM20948.hpp` |

- Whether to raise the `find_package(controller_manager 4.0.0)` minimum in `CMakeLists.txt`. The new
  code may need a newer Jazzy `ros2_control` 4.x (plausible, unproven; the reviewer's concern). That
  is a version-pin change ([ADR 0001](decisions/0001-version-lock.md)), so it needs a proposal; none
  is written yet. Record `ros2 pkg xml hardware_interface | grep '<version>'` and the same for
  `realtime_tools`, in the container and on the Pi.
- Whether `main` should adopt any of the `ai-forward` workflow: ADR 0005.
- Which wheel radius is right (0.7).

## Documentation gaps

An agent can draft these; a person checks the facts. They are listed in
[`architecture/README.md`](architecture/README.md) and [`guides/README.md`](guides/README.md).

## GitHub issues against the code

Issues #10 to #17 as of 2026-10-28 (none labelled), compared with the tree. #15 and #17 were closed
that day, at the maintainer's instruction, with a comment naming the evidence on `main`. Closing or
commenting on an issue is the maintainer's call: an agent lists what it thinks can be closed in its
report, and closes one only when told to.

| Issue | State in the code |
| --- | --- |
| [#10](https://github.com/BiPed-Capstone/ROCKO-env/issues/10) `use_mock_hardware` does nothing | **Done on `ai-forward`**: it selects `mock_components/GenericSystem`. Left open on purpose: `main` does not have the fix. Close it when the branch merges |
| [#11](https://github.com/BiPed-Capstone/ROCKO-env/issues/11) `view_robot.launch.py` default | **Done on `ai-forward`** (0.6): the default is now `"rocko_env"`; Tier 1 only, not run with a display. Left open on purpose: `main` does not have the fix. Close it when the branch merges |
| [#12](https://github.com/BiPed-Capstone/ROCKO-env/issues/12) no simulation | **Partly**: the mock landed, as the issue suggested first, and a `SimPlant` physics model now exists with a check (`verify_sim.sh`: S1 and S2a). Its parameters are estimates and it has no scenarios yet (2.3), so Tier 3 is not called built |
| [#13](https://github.com/BiPed-Capstone/ROCKO-env/issues/13) no tests | **Partly** (0.9) |
| [#14](https://github.com/BiPed-Capstone/ROCKO-env/issues/14) unpinned Python dependencies | **Open** (0.8) |
| [#15](https://github.com/BiPed-Capstone/ROCKO-env/issues/15) `setup.py` is dead code | **Closed 2026-10-28.** Already fixed on `main` (the file was removed in `a7d302a`) |
| [#16](https://github.com/BiPed-Capstone/ROCKO-env/issues/16) wheel radius disagrees | **Open** (0.7), and wider than the issue says |
| [#17](https://github.com/BiPed-Capstone/ROCKO-env/issues/17) template metadata in `package.xml` | **Closed 2026-10-28.** Already fixed on `main`: both packages have real descriptions and maintainers |

## Keeping this page true

Update a row in the same commit that changes its status. If a row here disagrees with the code, the
code is right and this page is a bug. Re-check the issues table whenever an issue is opened or closed.
