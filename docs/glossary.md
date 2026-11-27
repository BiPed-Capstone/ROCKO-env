# Glossary

The words a newcomer meets in the first week, in alphabetical order. Each one is explained in a sentence or two,
with a link to the page that explains it properly. Words from ROS and control theory come first, then words that
belong to this project.

## Robotics and ROS words

- **Cascade (cascaded control).** One feedback loop whose output is the *target* of a second, inner loop. ROCKO's
  outer loop turns a speed error into a target lean angle; the inner loop turns a lean error into wheel speed.
  [The control cascade](architecture/control-cascade.md).
- **Chainable controller.** A ros2_control controller whose output is read directly by the next controller, inside
  the same 100 Hz cycle, instead of over a topic. The cascade is built from them.
- **colcon, workspace.** `colcon` is the build tool; the workspace is the folder it builds in, which ends up holding
  `build/`, `install/` and `log/`. On the Pi the workspace is the checkout itself, `~/ROCKO-env`.
- **Container (dev container), Docker.** Docker runs a *container*: a self-contained Linux system on your laptop. The
  dev container in `docker/` has Ubuntu 24.04 and ROS 2 Jazzy, which a laptop does not, and sees your checkout's files.
- **Controller.** Not the gamepad: in ROS, a controller is code that reads sensor values and decides what to command, run by the controller_manager every
  cycle. ROCKO's are the two PID controllers, the two feedforward controllers, and in tank mode `diff_drive_controller`.
- **controller_manager.** The ROS process that loads the hardware plugins and the controllers and runs
  `read()`, `update()`, `write()` on a fixed clock: 100 times a second here (`update_rate` in the controller YAML).
- **Diagnostics (`/diagnostics`).** The standard ROS topic on which a part reports whether it is healthy: a level
  (OK, WARN, ERROR), a message and a few values. The robot extras report there.
- **Encoder (quadrature).** A sensor on each motor that produces pulses as the wheel turns; counting them gives the
  wheel's angle and speed. Read in the motor plugin: [encoders in C++](architecture/encoders-in-cpp.md).
- **Feedforward.** A term added to a controller's output from what is *expected* (here, the commanded wheel speed),
  instead of from a measured error. It lets the feedback loops correct only what the prediction missed.
- **Framebuffer.** A Linux device file, `/dev/fbN`, whose bytes are a screen's pixels: write a picture's bytes to
  it and the screen shows it. The status screen's driver makes the panel one.
- **Hardware interface (hardware plugin).** C++ code that owns a real device and exposes its values to
  ros2_control. `Motor12VoltQuadEncoder` (motor and encoder) and `BNO055` (the IMU) are the only code that touches
  the Pi's pins.
- **IMU.** Inertial measurement unit: the sensor that measures the robot's lean (pitch) and how fast it is changing
  (pitch rate). ROCKO's is a Bosch BNO055, read over I²C (a two-wire bus for chips on the same board): [the IMU read](architecture/imu-i2c-read.md).
- **INA226.** A small chip that measures a voltage (and, through a shunt resistor, a current) and reports it over
  I²C. The battery monitor reads the battery voltage with one.
- **Launch file.** A Python file that starts a set of ROS programs with their settings, run with `ros2 launch`.
  ROCKO's are in `rocko_env/rocko_env/bringup/launch/`.
- **Node.** One running ROS program that does one job. Nodes talk over topics.
- **PID.** Proportional, integral, derivative: a feedback controller that acts on the error now (P), the error
  added up over time (I) and how fast the error is changing (D). Its numbers are the *gains*.
- **Pitch.** How far the robot leans forward or back, in degrees, from the IMU. `pitch_offset_deg` in the
  ros2_control xacro shifts the reading so that level reads about 0. Which way is positive matters to every loop;
  it was measured on the robot: [stand sign check](journal/2026-11-24-stand-sign-check.md) and
  [the gyro check](journal/2026-11-23-gyro-check.md).
- **pluginlib.** The ROS library that finds a C++ plugin by its name when a launch starts. If the name is spelled
  differently in any of the places that declare it, ROS cannot find the plugin (`tools/check_plugin_names.py` checks).
- **PWM, duty.** Pulse-width modulation sets a motor's power by switching it fully on and off many times a second;
  the *duty* is the percentage of time it is on. The motor plugin turns a wheel-speed command into a duty.
- **ROS 2.** The framework the robot's software is written in: many small programs (nodes) passing messages. ROCKO
  uses the Jazzy release.
- **ros2_control.** The ROS framework that separates "how do I talk to this device" (hardware interfaces) from
  "what should it do now" (controllers). [Official docs](https://control.ros.org/jazzy/index.html).
- **Sourcing.** `source .../setup.bash` tells a terminal where the ROS packages are. Needed in every new terminal; on
  the Pi, `pi/rocko_shell.sh` does it.
- **Topic.** A named channel that nodes publish messages on and subscribe to, such as `/joy` (the gamepad's
  sticks and buttons).
- **URDF, xacro.** URDF is the XML file that describes the robot's parts and joints, and here also which hardware
  plugin drives each joint. Xacro is XML with macros that expands into URDF; you edit the `.xacro`.

## Words from this project

- **Balance mode, tank mode.** The two ways the robot runs. Balance mode is the cascade (the real goal; it has not
  balanced yet). Tank mode drives the wheels directly, with no balancing, for when the robot should just move.
- **Dead band.** Two related things: (1) wheel-speed commands so small that the motor plugin sends 0 duty instead;
  (2) the low duties at which a motor hums but does not turn, because friction wins. [Proposal 0013](proposals/0013-tank-mode-motor-breakaway.md).
- **Enabled, disabled.** Whether the motion controllers are active. The robot boots disabled; LB + RB on the
  gamepad, or the dashboard's Enable button, toggles it. [The robot supervisor](architecture/robot-supervisor.md).
- **Extras (robot extras).** The optional status screen, buzzer and battery monitor. Each runs as its own program,
  reports its health on `/diagnostics`, and none can stop the robot: [the robot extras](architecture/robot-extras.md).
- **Finding (C-1, H-3, M-5, L-2 ...).** A numbered problem in
  [the balance-loop findings](claude-analysis/balance-loop-findings.md). The letter is the severity: critical, high,
  medium, low. Cite the ID rather than describing the problem again.
- **Journal.** One page per AI-written change in [`journal/`](journal/README.md): what was asked, what changed, and
  which checks actually ran.
- **Mock (mock hardware).** `use_mock_hardware:=true`: ros2_control's stand-in that echoes every command back as a
  measurement. It checks that controllers load and connect. It has no physics: pitch stays 0.0, so a balance
  controller always looks fine on it.
- **Proposal.** A change the AI may not make by itself (gains, version pins, safety behavior, anything on the robot),
  written up for a human to decide: [`proposals/`](proposals/README.md).
- **SimPlant (the simulator).** A physics model of ROCKO, an inverted pendulum on two motor-driven wheels, with
  estimated numbers. It stands in for the hardware plugins in the dev container: [SimPlant](architecture/sim-plant.md).
  It is not the mock.
- **Supervisor.** `Supervisor.py`, started at boot. It owns which mode runs and whether the robot is enabled, and
  starts and stops each mode's programs. [The robot supervisor](architecture/robot-supervisor.md).
- **Teleop.** Driving by hand. `BalanceTeleop.py` and `TankTeleop.py` turn the gamepad's sticks into drive
  commands.
- **Tier (0 to 4).** A rung of [the verification ladder](architecture/verification-ladder.md): 0 static checks,
  1 build, 2 mock, 3 simulation, 4 the robot (a human only).
- **Tipover cutoff.** A safety rule in the feedforward controller: past a set lean it commands zero wheel speed, and
  it stays at zero until a person deliberately re-arms it with the robot near upright. It never re-arms by itself,
  not even when the robot is picked up (finding H-4).
- **Watchdog.** A safety rule that zeroes the speed the robot is asked to drive at when no fresh command has arrived
  for a set time, so a lost gamepad or a crashed node cannot leave the robot driving away (finding C-3). In balance mode
  the balance loop keeps running: the robot stops driving, it does not stop balancing.
