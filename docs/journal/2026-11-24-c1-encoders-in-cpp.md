# C-1, encoder half: count the wheel encoders inside the C++ motor plugin

- **Date / commit:** 2026-11-24, built on `81f0e03` (this entry is added by the commit that makes the change)
- **Branch:** `ai-forward`
- **Environment:** laptop (macOS) + Docker dev container (Jazzy); the robot's Pi 5 (Ubuntu 24.04, kernel
  6.8.0-1068-raspi, WiringPi 3.20) read over SSH with the owner's permission

## Goal and success criteria

The owner asked to "continue to the next step". Items 1 and 2 of the
[next-steps handoff](../next-steps.md) (measure the wheel, a motor-driven speed check) need the owner at the robot;
item 3, finding C-1, is half done (the IMU, earlier on 2026-11-24). This run does the other half, row 1.3: the motor
plugin's `read()` no longer calls a Python service and waits with no time limit
([C-1](../claude-analysis/balance-loop-findings.md#c-1--unbounded-blocking-service-calls-inside-the-real-time-loop)).
It counts encoder edges itself.

Done means:

1. `Motor12VoltQuadEncoder` has no ROS service client and no `wait_for_service`/`spin_until_future_complete`
   anywhere; `read()` only loads an atomic count and does arithmetic.
2. The position and velocity it reports are **the same signal** `QuadEncoders.py` produced: metres of travel and
   m/s, forward positive, the same x4 decoding table, the same low-pass filter (0.05 per cycle) over the real
   cycle length. So the gains, the sim's encoder replica (`model_encoder_node`) and every doc about that signal
   stay true. A host test checks the decoding and the filter against the Python node's behaviour, and goes red on
   deliberate breaks.
3. The encoder pins, wheel radius and counts per wheel turn are hardware parameters in both real xacros; a missing
   or malformed one refuses to start. `verify_urdf.sh` checks the pins in place of the old `encoder_service`.
4. `QuadEncoders.py` is no longer started by either launch file, nor expected by the supervisor.
5. The compile-only check passes on the new plugin (Tier 1 does not build it); the ladder (Tiers 0 to 3) stays as
   green as it was.
6. On the robot (owner present): both launch modes start, the encoders read forward-positive on each wheel with
   the old scale, and the shutdown no longer needs SIGKILL **if** the encoder waits were the cause.

## Plan

- New header `hardware/motors/include/rocko_env/WheelEncoder.hpp`: the quadrature table (the same as the Python
  `Encoder` library, a port of Paul Stoffregen's Arduino library) and the velocity filter, standard library only,
  so `tools/wheel_encoder_test.cpp` can test them on a laptop.
- `Motor12VoltQuadEncoder.{hpp,cpp}`: in `on_configure`, read the start state and register a WiringPi interrupt
  (`wiringPiISR2`, both edges) on each encoder pin. The callback runs on WiringPi's own thread (one per pin), reads
  both pins, steps the table under a mutex that only those two threads take, and adds to a
  `std::atomic<std::int64_t>`. `read()` loads the count (lock-free) and runs the filter with `period`. Stop the
  interrupts in `on_cleanup`, `on_shutdown` and the destructor (`wiringPiISRStop` cancels and joins the thread, so
  no callback can run on a destroyed plugin).
- xacros (`rocko`, `rocko_tank_mode`): `encoder_pin_a`, `encoder_pin_b` (WiringPi numbers: the BCM 26/20 and 16/19
  the launch files gave the Python node, right channels swapped as the node swapped them), `wheel_radius_m`,
  `encoder_counts_per_wheel_rev`.
- Launch files, `supervisor/modes.py`, CMake install, the compile-check stub, `verify_urdf.sh`, docs.
- **Tiers:** host test for the decoding and filter; compile-only check for the plugin; Tier 0 `verify_urdf.sh`;
  Tiers 1 to 3 with `tools/run_ladder.sh`; Tier 4 is the owner's.
- **Risks:** a missed edge at speed (the Python library had the same exposure, on a slower path); two threads
  touching the decoder (hence the mutex); an interrupt thread outliving the plugin (hence stop and join); a wrong
  pin number (the robot check reads each wheel's sign and scale).

## What changed

- `hardware/motors/include/rocko_env/WheelEncoder.hpp` (new): the quadrature table and the speed filter, standard
  library only. `tools/wheel_encoder_test.cpp` (new) tests them.
- `Motor12VoltQuadEncoder.{cpp,hpp}`: no ROS client, node, `wait_for_service` or `spin_until_future_complete`. Encoder
  parameters parsed strictly; WiringPi interrupts on both edges of both pins from `on_configure`, stopped in
  `on_cleanup`, `on_shutdown` and the destructor; `read()` loads the atomic count and times the step with
  `steady_clock`. **Safeguard added:** `on_shutdown` zeroes and stops the software PWM (ros2_control can shut a
  component down from active without `on_deactivate`).
- Both real xacros: `encoder_pin_a`/`encoder_pin_b`, `wheel_radius_m`, `encoder_counts_per_wheel_rev` replace
  `encoder_service`. `verify_urdf.sh` checks them (and that `encoder_service` is gone).
- `rocko.launch.py`, `rocko_tank_mode.launch.py`: the encoder node is gone. `supervisor/modes.py`: no longer
  expected. `CMakeLists.txt`: its install line is gone. `QuadEncoders.py` and `tools/quad_encoders_test.py` deleted.
- `tools/compile_check/stubs/wiringPi.h`: the interrupt API; `check_motor_plugin_compiles.sh`: no longer needs the
  service header. `tools/run_ladder.sh`: the motors include folder for host tests.
- Docs: [encoders-in-cpp.md](../architecture/encoders-in-cpp.md) (new), next-steps rows 1.3, 1.10, 4.10 and the
  handoff, the ladder's host-test tables, C-1's status, proposal 0010, README, sim-plant, sim-to-real-gap,
  measure-inertia section 8, bench-test-tank-mode, verify-in-container.

## Verification

| Tier | Command | Result |
| --- | --- | --- |
| host | `tools/wheel_encoder_test.cpp` | `all WheelEncoder tests passed`; **red on 5 of 5** deliberate breaks (sign flipped, ±2 dropped, fixed 0.01 s, reference moved on a zero period, NaN period used). The first version caught a wrong comment of mine (which way counts up) |
| 0 | `verify_urdf.sh` | pass; **red** with the right wheel's A and B swapped in the xacro, pass when restored |
| 0/1 | `check_motor_plugin_compiles.sh` | pass, 0 warnings (syntax and types only, against stubs); red on a broken copy |
| all | `tools/run_ladder.sh` (on the first version) | **all PASS**: Tier 1 build (0 compiler warnings), Tier 0, Tier 2 mock balance and tank, supervisor, Tier 3 `verify_sim.sh` both sets and `verify_tools.sh`, every host test, hook tests, doc links |
| all but 3 | `tools/run_ladder.sh --no-sim` (after round 1 fixes) | all PASS, 0 warnings. Tier 3 not rerun: the fixes touch only the motor plugin (not built off the Pi) and docs |
| 1 | the motor plugin on the Pi | see "On the robot" below; **Tier 1 in the container does not build it** |

The mock replaces the motor plugin, and the sim replaces it with `SimPlant`, so no tier below the robot runs the new
code. The sim's encoder model was not changed: the signal it copies is unchanged except that the step is now timed in
`read()` rather than taken from `period` (the Python node also timed it itself).

## Review rounds

| Round | Reviewer | Findings | Fixed / declined (why) |
| --- | --- | --- | --- |
| 1 | realtime-auditor | High: a dead encoder reads as standing still (row 1.10, known); Low: `period` is not when the count was taken; Medium: WiringPi's priority-55 threads (0.5 ms poll) above the SCHED_OTHER loop; Medium: ISR setup failures might `exit()` | 2 fixed (`steady_clock` in `read()`). 1 declined here: needs a measured threshold (row 1.10). 3 to measure on the robot (procedure step 6). 4 checked in WiringPi 3.20's source on the Pi: the line request runs synchronously and returns -1, `exit()` only when WiringPi is not initialised, so configure does fail |
| 1 | ros2-control-reviewer | Medium: not built against real WiringPi; Low: the `exit()` question; Low: does `wiringPiISRStop` join; Low (pre-existing): `on_shutdown` leaves PWM running | Pi build is robot step 1; `exit()` and join checked in the source (join: `pthread_cancel` then `pthread_join`, cited in the comment); `on_shutdown` safeguard added |
| 1 | explainability-reviewer | 10: undocumented switch keys, two near-identical parse lambdas and one message for six faults, finding IDs as explanations, a circular comment, the NaN guard's reason, two homes for alpha, too many references to the deleted node, WiringPi numbering | all fixed except the pre-existing integer division in `MAX_RAD_PER_SEC` (finding U-3, kept on purpose; the sim copies it) and the member naming (matches the file) |
| 1 | docs-reviewer | stale pass/fail check in bench-test-tank-mode, README library and folder rows, gap-page and sim-plant present tense, handoff line, "robot result is in the journal" before there was one, an unverified priority claim, numbers restated | all fixed |
| 2 | realtime-auditor | clean on the round-1 fixes; 3 Low: `softPwmCreate`'s result ignored (M-5 pattern), the destructor does not stop the motor, no `on_error` stop | all three added as safeguards |
| 2 | explainability-reviewer | half-documented switch keys, a finding ID still opening `read()`, references to the deleted node, a dated "safeguard" prefix | fixed; the suggestion to move parameter parsing to its own function declined for now (the reviewer agreed) |
| 3 | realtime-auditor | clean: the three safeguards are correct (`on_error` returning SUCCESS sends the component to unconfigured, so it can be configured again; edges are not counted until then) | nothing to fix |
| robot fix | realtime-auditor | clean: the speed pin held LOW from configure, PWM only from activate; no pin left driving, nothing in the loop changed | nothing to fix |

**Not verified by the ladder:** anything on the robot. The robot steps below were run afterwards; see "On the robot".

## Robot procedure

The owner is present for every step; the agent runs the commands over `ssh rocko` with the owner's permission. Nothing
here enables balance mode.

1. **Build on the Pi**, as the `rocko` user (not root): `cd ~/ROCKO-env && git pull && ./build_ros.sh`. Success: both
   packages build, the motor plugin included, no warnings in `Motor12VoltQuadEncoder.cpp`.
2. **Restart the service** (it boots into tank, disabled): `sudo systemctl restart rocko.service`. Success: the
   journal (`journalctl -u rocko.service`) shows both motor components configured and active, and no
   `Could not start the interrupt` line; `ros2 node list` (from `pi/rocko_shell.sh`) has no `/node` (the old encoder
   node).
3. **Sign and scale, wheels off the ground** (owner turns each wheel by hand; agent records about 30 s, started only
   after the owner says "ready"): `ros2 topic echo /joint_states`. Success: turning a wheel **forward** raises its
   `position`; one full turn moves it about 0.36 m (the 2026-11-23 reading of the Python node); the other wheel does not
   move.
4. **Motor-driven, still on the stand, tank mode, enabled with LB+RB**: a gentle stick forward for a few seconds. Success:
   both `velocity` values positive and close to each other, no `ERROR` from the motor components.
5. **Shutdown:** switch to balance and back (LT+RT+RB, disabled), then `sudo systemctl restart rocko.service`. Record
   whether `ros2_control_node` still needs SIGKILL (`journalctl` shows `SIGKILL` / `Killing`).
6. **What the interrupt threads cost** (realtime audit, round 1): `ps -eLo tid,cls,rtprio,pcpu,comm | grep -E 'ros2_control|FF|RR'`
   with the wheels still and while one is spun by hand: how many WiringPi threads, at which priority, how much CPU.
   WiringPi's threads wake every 0.5 ms even with no edge (a 0.5 ms `ppoll` timeout), at real-time priority 55,
   above the control loop's ordinary priority (H-5). If they cost the 100 Hz loop, that is loop-timing item 4's
   business (pin them to another core).

Abort on: a wheel turning with no command, a wheel running away, either motor component in `ERROR`, any buzz or heat.

## On the robot, 2026-11-24 (owner present, agent over `ssh rocko` with the owner's permission)

Commit `f689421` plus the configure fix below (built on the Pi from the working copy, then committed). Pi 5, Ubuntu
24.04, kernel 6.8.0-1068-raspi, WiringPi 3.20. Robot on the stand, wheels off the ground, battery voltage not
recorded. Recordings were made on the Pi in `/tmp` (`enc_turn.csv`, `enc_drive.csv`) and were not kept.

| Step | Result |
| --- | --- |
| 1 build | `./build_ros.sh`: both packages, motor plugin compiled, no warnings |
| 2 start (first try) | **FAILED, and the fix:** `RightDriveMotor` refused to activate: `Could not start software PWM on speed pin 13`, then `ros2_control_node` exited (`Failed to set the initial state of the component`). Cause, read in WiringPi's source: `on_configure` called `pinMode(speedPin, SOFT_PWM_OUTPUT)`, which itself starts software PWM (`wiringPi.c:2268`), so `on_activate`'s `softPwmCreate()` always returned -1, "already running" (`softPwm.c:134`). The old code ignored that return value, so the bug was invisible; the safeguard added in review round 2 exposed it. Fix: `on_configure` sets the speed pin as a plain output held LOW, and PWM starts only in `on_activate`. Nothing drove the wheels during the failed start |
| 2 start (after the fix) | both motor components configured and active, tank `phase=ready`, disabled; no `/node` in `ros2 node list`; `/joint_states` 0.0, 0.0 |
| 3 hand turns (30 s) | each wheel turned forward one full turn by the owner: left **+0.384 m**, right **+0.388 m**, the other wheel unchanged at 0.000 both times. Signs right, wheels independent. The code's constants predict 0.377 m per turn; the Python node read 0.360 m per turn on 2026-11-23 over five turns. One hand turn is only good to a few per cent, so this **suggests** the Python node lost counts; a five-turn repeat would settle it (handoff item 1) |
| 4 motor-driven (30 s) | enabled with LB+RB, gentle steady stick forward: both speeds positive, steady about **0.85 m/s left, 1.07 m/s right**, no error from either motor component; both stopped when released. The 25 % difference fits the measured break-away difference (left 28 %, right 21 % duty) in open-loop tank mode; whether any of it is lost counts is not known |
| 6 thread cost | the four WiringPi interrupt threads run at `SCHED_RR` 55 and use about **1.5 % of a core each** with the wheels still (the 0.5 ms poll), about 6 % in all; `ros2_control_node` about 10 % of one core. There is a `SCHED_FIFO` 50 thread in `ros2_control_node`, very likely the control loop, so the interrupt threads (55) can preempt it: a loop-timing item 4 question. `ps`'s CPU figure is a lifetime average, so the spinning-wheel number was not measured |
| 5 shutdown | tank: `systemctl restart` took **0.96 s**, no SIGKILL. Tank to balance to tank with LT+RT+RB (disabled): the balance stack stopped in about **0.7 s, no SIGKILL**; the IMU started as before (pitch +8.29 deg). The SIGKILL seen earlier on 2026-11-24 is gone in both modes, consistent with the encoder service wait having been its cause. Remaining shutdown noise: two `pal_statistics` "context cannot be slept with" errors, harmless |

**Stopped because:** the goal is met on the stand (code, ladder, three review rounds, robot steps 1 to 6). Not done:
anything with the wheels on the floor, a five-turn scale check, the loop-timing measurement.
