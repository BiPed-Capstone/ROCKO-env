# Project history: how ROCKO got here, in one page

The journal has one entry per change, more than forty by late November 2026, and most are long. This page is the
short version: what was tried, in what order, what was learned, and where the direction changed. Each step links
the entries that hold the evidence. Read this first, then open an entry only when you need its detail. Words you do not know are in the
[glossary](../glossary.md).

## Before `ai-forward` (to October 2026)

The capstone team wrote the first robot code on `main`. Their core decisions are in
[ADRs 0001 to 0004](../decisions/): the ROS 2 Jazzy version lock, the cascaded balance controller, Gazebo as the
planned simulator, and device I/O in C++ hardware plugins. The last two are still `Proposed`; the fourth was an
intention: the robot's IMU and encoders
were still read by Python nodes. The robot had not balanced.

## October 27 to 30: setting up a loop an AI can run

The `ai-forward` branch opened with the rule that AI writes the code and checks its own work
([ADR 0005](../decisions/0005-ai-forward-development.md)). The first runs built the means to check anything without
the robot:
- a mock hardware mode;
- a laptop build in a Docker container;
- the first verify scripts;
- guardrails on what the agent may change.

[Bootstrap](2026-10-27-ai-forward-bootstrap.md), [the mount check](2026-10-29-container-mount-check.md),
[unattended runs](2026-10-29-unattended-run-readiness.md).

**Learned:** a check is only worth having once you have watched it fail. The mount check exists because the
container could quietly test another folder's files and pass.

## October 31 to November 8: a simulated robot, and the critical findings

An audit, [the balance-loop findings](../claude-analysis/balance-loop-findings.md), listed what was wrong, by
severity. The agent then built `SimPlant`, a physics model of the robot, instead of Gazebo: it is one readable
header, it needs no change to the container image, and it runs the real controllers
([proposal 0005](../proposals/0005-amend-adr-0003-simulpant-first.md)). Against it the agent added:
- the tipover cutoff and the teleop watchdog;
- an IMU fault rule;
- balance scenarios, and sim-only gains tuned until those scenarios passed;
- a sweep that reran the scenarios with each physical parameter changed by up to ±30 %.

[SimPlant and the critical fixes](2026-10-31-sim-plant-and-critical-fixes.md),
[stage 5](2026-11-03-stage5-balance-in-sim.md), [the final report](2026-11-04-final-report-stage-4-6.md).

**Learned:** timing checks that use a fixed time window fail at random, and about a third of one run's cost went on
rerunning them ([agent-loop efficiency](../claude-analysis/agent-loop-efficiency.md)). Checks now record when each
event actually happened, and `tools/run_ladder.sh` runs the whole ladder in one call.

**Learned:** the simulator only knows what is written into it. Every physical number is an estimate
([proposal 0007](../proposals/0007-measure-robot-parameters.md) asks for real ones).

## November 16 to 22: driving the real robot, three ways in a week

Laptop driving came first: a keyboard, a web dashboard and recording
([journal](2026-11-16-teleop-dashboard-logging.md)). The first run on the robot since `ai-forward` came on
November 19: tank mode drove, but small commands only made the motors hum
([first robot run](2026-11-19-first-robot-run-tank-mode.md); the dead band,
[proposal 0013](../proposals/0013-tank-mode-motor-breakaway.md)). Then an Xbox controller over Bluetooth from a Mac
([journal](2026-11-20-xbox-controller-split-arcade.md)). Two days later that was replaced by an 8BitDo gamepad
plugged straight into the Pi, and the laptop driving code was deleted
([journal](2026-11-22-usb-controller-tank-mode.md)). Tank mode's slow reversal was measured and fixed
([journal](2026-11-22-tank-reversal-lag.md)).

**Learned:** the simplest path won. A gamepad on the Pi removed the network, the laptop program and a whole class of
lost-link problems.

## November 23 to 24: one supervisor, and sensors read in C++

- A supervisor node now owns which mode runs and whether the robot is enabled, switched from the gamepad or the
  dashboard ([journal](2026-11-23-robot-supervisor.md)). Its boot service replaced the tank-mode and dashboard boot
  services of November 22 ([the robot supervisor](../architecture/robot-supervisor.md)).
- Finding C-1, the most serious, was fixed in code: the IMU and the encoders had been read by Python nodes through ROS
  service calls that could block the control loop with no time limit. A first step on November 23 gave each wheel its
  own encoder service ([journal](2026-11-23-encoder-service-u2.md)); then both sensors moved inside the C++ hardware
  plugins, read once per control cycle ([IMU](2026-11-24-c1-imu-i2c-and-zero-on-deactivate.md),
  [encoders](2026-11-24-c1-encoders-in-cpp.md)). An earlier plan to move the IMU to a serial (UART) link
  ([journal](2026-11-07-c1-bno055-uart-framing.md)) was set aside for I²C and stays as a fallback.
- On the robot, the owner measured:
  - the gyro axis and sign ([gyro check](2026-11-23-gyro-check.md));
  - the encoder sign ([stand sign check](2026-11-24-stand-sign-check.md));
  - the control loop's timing on the Pi ([loop timing](2026-11-24-loop-timing-on-the-pi.md)).
- The simulator was rerun with the measured signs ([journal](2026-11-24-sim-measured-signs-retune.md)).

**Learned:** signs are the first thing to measure. A wrong sign anywhere in a cascade looks like bad gains.

## November 25: the first balance test on the floor

Balance mode was enabled on a flat floor for the first time. The same day the IMU was found to read 10.1 degrees with
the robot standing level; the offset was corrected so level should read 0, which has not yet run on the robot. Whichever way the robot leaned, the wheels
spun the same way, until the tipover cutoff tripped. The likely cause, reasoned from the code and not recorded: the
velocity loop's integral term keeps adding up the speed error and pushes the target lean far past anything
balanceable (integrator wind-up). A model reproduces the symptom once the wheels have started turning. The proposed
fix, waiting on the owner, clamps the target lean to ±5 degrees
([proposal 0017](../proposals/0017-clamp-the-target-lean.md); journals:
[the floor test](2026-11-25-level-pitch-offset-and-lean-clamp.md),
[the test reproduced in the model](2026-11-25-offline-sim-work.md)).

**Learned:** a test with no recording leaves only reasoning. The next balance test should be recorded
(`record:=true`).

## November 26: cleanup while the robot is away

- Dead code and stale docs were removed ([cleanup](2026-11-26-cleanup.md)).
- The IMU plugin was renamed after the part actually fitted ([rename](2026-11-26-rename-imu-plugin-bno055.md)).
- The repo was made easier to start in: a getting-started guide, a glossary, and a proposals index sorted by what
  waits on the owner ([tidy](2026-11-26-repo-tidy.md)).

## Where it stands

The current state and the next steps are in the handoff box at the top of [next-steps.md](../next-steps.md). This
page records how the project got there.
