# ROCKO-env

This is the ROS 2 code for **ROCKO**, a two-wheeled self-balancing robot. Everything here runs on a Raspberry Pi mounted on the robot.

This README is the main documentation: what the robot is, how the pieces connect, and how to build and run
it. More material lives in **[`docs/`](docs/)**. This includes subsystem explanations, procedures, architecture
decision records, and code audits. See [Documentation](#documentation) at the bottom for the map.

---

## Getting Started

The project is built on **ROS 2**, a framework for robot software. The two ideas you need on day one:

- A **node** is just a program that does one job (read the IMU, talk to the gamepad, run a PID loop).
- Nodes talk to each other over **topics**, which are named message channels. One node publishes to `left_feedforward`, another subscribes to it. Nobody calls anybody directly.

On top of that we use **ros2_control**, which is a standard way to separate *"how do I physically talk to this motor"* from *"what should the motor do right now."* That split explains the whole folder layout:

- A **hardware interface** is a C++ plugin that owns a real device. `Motor12VoltQuadEncoder` writes PWM to a motor pin; `ICM20948` reads the IMU. These are the only files that touch GPIO.
- A **controller** reads sensor values and decides what to command. Controllers can be *chained*, meaning one controller's output becomes the next one's input. That chaining is exactly how the balancing works.
- The **controller_manager** is the process that loads both of the above and runs them on a fixed clock (100 Hz for us).

If none of that is familiar yet, the [official ros2_control docs](https://control.ros.org/jazzy/index.html) are the best background reading, and the templates in `examples/example_hardware/` show the minimum skeleton for a new hardware plugin.

### How the pieces actually connect

Here's the real signal path when you drive the robot, from gamepad to spinning wheel:

```
 your laptop                    the Raspberry Pi
┌──────────────────┐   TCP    ┌────────────────────────────────────────────────┐
│ computer_        │  :1234   │ Joystick.py                                    │
│ joystick.py      │─────────>│   publishes → diffbot_base_controller/cmd_vel  │
│ (pygame gamepad) │          └───────────────────────┬────────────────────────┘
└──────────────────┘                                  │
                                                      v
                              ┌────────────────────────────────────────────────┐
                              │ DiffDriveController.py                         │
                              │   → velocity_pid_controller/reference          │
                              │   → left_feedforward / right_feedforward       │
                              └───────────────────────┬────────────────────────┘
                                                      v
                              ┌────────────────────────────────────────────────┐
                              │ velocity_pid_controller                        │
                              │   "am I at the speed I want?" → commands pitch │
                              └───────────────────────┬────────────────────────┘
                                                      v
                              ┌────────────────────────────────────────────────┐
                              │ pitch_pid_controller                           │
                              │   "am I leaning right?" → commands wheel speed │
                              └───────────────────────┬────────────────────────┘
                                                      v
                              ┌────────────────────────────────────────────────┐
                              │ left/right_add_feedforward_controller          │
                              │   adds the feedforward term, then drives the   │
                              │   Motor12VoltQuadEncoder hardware plugin       │
                              └────────────────────────────────────────────────┘
```

To control the robot to go *faster*, the robot first has to *lean* further forward. So the velocity loop doesn't command wheel speed directly. It commands a target lean angle, and the pitch loop underneath it works out the wheel speed needed to hold that lean. The IMU supplies the actual pitch, and the wheel encoders supply the actual velocity.

There are two ways to run the robot:

- **Balancing mode**: the cascade above. This is the real robot. Uses `rocko.urdf.xacro` + `rocko_controllers.yaml`.
- **Tank mode**: balancing disabled, plain differential drive, for when you just want it to move without fighting the control loop. Uses `rocko_tank_mode.urdf.xacro` + `rocko_tank_mode_controllers.yaml`.

---

## What you need installed

### Which ROS 2 version you need

You need **ROS 2 Jazzy Jalisco** on **Ubuntu 24.04**. This is not a preference. `rocko_env/CMakeLists.txt` explicitly checks for `controller_manager >= 4.0.0` and stops the build with a hard error if it doesn't find it:

```
FATAL_ERROR: ros2_control version 4.0.0 or higher is required.
```

Humble ships ros2_control 2.x and Iron ships 3.x, so **both will fail this check**. Our `pid_controller` settings also use a newer config format that older distros don't understand. If you see that error, you have the wrong distro.

| What | Version |
| --- | --- |
| ROS 2 | Jazzy Jalisco |
| Ubuntu | 24.04 (Noble) |
| Python | 3.12 (comes with Ubuntu 24.04) |
| ros2_control | 4.0.0 or newer |
| CMake | 3.16+ |
| C++ | C++17 |

### ROS packages

These are all listed in the `package.xml` files, and `rosdep` (step 2 of the build below) installs them for you, so you shouldn't need to install any by hand. Listed here so you know what's in play:

**Used to build:** `hardware_interface`, `controller_interface`, `pluginlib`, `rclcpp`, `rclcpp_lifecycle`, `rclpy`, `generate_parameter_library`, `parameter_traits`, `backward_ros`, `rosidl_default_generators`

**Used at runtime:** `controller_manager`, `diff_drive_controller`, `forward_command_controller`, `joint_state_broadcaster`, `pid_controller`, `joint_state_publisher_gui`, `robot_state_publisher`, `ros2controlcli`, `ros2launch`, `rviz2`, `xacro`, `std_msgs`, `control_msgs`, `geometry_msgs`, `sensor_msgs`, `foxglove_bridge` (used by `rocko.launch.py`)

### WiringPi (Raspberry Pi only)

The motor code uses [WiringPi](https://github.com/WiringPi/WiringPi) for GPIO and software PWM. There's a copy of `wiringPi.h` checked into `rocko_env/hardware/WiringPi/` for reference, but that header alone isn't enough. The actual compiled library has to be installed on the Pi, or CMake stops with "WiringPi not found".

This is also why a laptop build has to leave the motor plugin out (`-DROCKO_SKIP_WIRINGPI=ON`). More on that below.

### Python libraries

Heads up: **none of these are pinned**. There's no `requirements.txt` yet, so `pip` grabs the latest of each ([#14](https://github.com/BiPed-Capstone/ROCKO-env/issues/14)).

| Library | What uses it | Needs real Pi hardware? |
| --- | --- | --- |
| `numpy` | encoders, IMU, calibration | no |
| `flask` | the telemetry web server | no |
| `pygame` | the laptop gamepad script | no |
| `tqdm`, `ahrs` | IMU calibration scripts | no |
| `Adafruit-Blinka` (imported as `board`) | IMU | **yes**, needs real I²C |
| `adafruit-circuitpython-icm20x` | IMU | **yes** |
| `adafruit-circuitpython-bno055` | old IMU calibration script | **yes** |
| `Encoder` | wheel encoders | **yes**, needs real GPIO |

---

## Where things live

```
ROCKO-env
├── CLAUDE.md                       How AI agents work in this repo (read this branch's rules first)
├── build_ros.sh                    Installs dependencies and builds, in the container or on the Pi
├── check_container_mount.sh        Checks the dev container is mounting THIS checkout, so the verify scripts test your files (laptop)
├── verify_urdf.sh                  Checks the robot description expands and holds the right hardware, mock on and off (container)
├── verify_mock.sh                  Checks the controllers load and chain on mock hardware (container)
├── .githooks/                      Git hooks: no commit on main, no push but a fast-forward of ai-forward to origin. Inactive until copied into .git/hooks/ (proposal 0002, item E)
├── docker/                         Dev container for laptops (compose files + Dockerfile)
├── docs/                           Architecture, guides, decisions, run journal, proposals, audits
├── examples/example_hardware/      Skeleton templates. Copy when adding new hardware
├── joystick/computer_joystick.py   Runs on YOUR laptop, not the robot. Sends gamepad input over TCP
├── rocko_env/                      The main package
│   ├── CMakeLists.txt, package.xml     Build config and dependency list
│   ├── rocko_env_hardware.xml          Registers the hardware plugins with ROS
│   ├── rocko_env_controllers.xml       Registers the controller plugin with ROS
│   └── rocko_env/
│       ├── bringup/
│       │   ├── config/                 PID gains and controller settings. Tune here
│       │   └── launch/                 The three "start everything" files
│       ├── controllers/
│       │   ├── add_feedforward/        C++ controller that adds a feedforward term
│       │   ├── diffdrive/              Turns steering commands into per-wheel targets
│       │   ├── joystick/               TCP server that receives laptop gamepad input
│       │   └── utils/                  RateLimiter.py smooths out sudden changes
│       ├── description/
│       │   ├── urdf/                   The robot's physical shape and joints (xacro files)
│       │   ├── ros2_control/           Which hardware plugin drives which joint, and GPIO pins; also the mock
│       │   ├── rviz/                   Saved RViz visualization layouts
│       │   └── launch/                 view_robot.launch.py shows the model, no hardware
│       ├── hardware/
│       │   ├── WiringPi/               Reference copy of wiringPi.h
│       │   ├── motors/                 Motor driver, the only file writing motor PWM
│       │   └── sensors/
│       │       ├── icm20948/           IMU driver + calibration scripts
│       │       └── quadEncoder/        Wheel encoder readers
│       └── webserver/                  Small Flask app for live plots in a browser
└── rocko_interfaces/                   Custom message and service definitions
```

A note on the `.xacro` files: xacro is "XML with macros." It's a templating layer that expands into a URDF, which is the file format ROS uses to describe a robot's links and joints. You edit the xacro; the launch file expands it for you at startup.

---

## Building it

First, some vocabulary that trips people up:

- A **colcon workspace** is a folder with a `src/` subfolder holding one or more packages. `colcon` is the build tool. **This repo is not a workspace by itself.** It goes *inside* one.
- **Sourcing** a setup file (`source .../setup.bash`) is what tells your terminal where the ROS packages are. You have to do it in every new terminal window.

```bash
# 1. Make a workspace and clone this repo into its src/ folder
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/BiPed-Capstone/ROCKO-env ROCKO-env

# 2. Install dependencies
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
sudo apt update
rosdep install --from-paths src --ignore-src -r -y

# 3. On the Pi only: install WiringPi first, or step 4 stops with "WiringPi not found"
#    https://github.com/WiringPi/WiringPi
#    (Not on a Pi? Skip this and add --cmake-args -DROCKO_SKIP_WIRINGPI=ON to step 4.
#     See "Working on a laptop" below for what that build can and cannot do.)

# 4. Build (run this from ~/ros2_ws, not from inside src/)
colcon build --symlink-install

# 5. Tell this terminal about what you just built
source ~/ros2_ws/install/setup.bash
```

**Why `--symlink-install`?** It links your Python files, launch files, and YAML configs into the build output instead of copying them. Practical upshot: you can edit a Python node or tweak PID gains and just re-run, with no rebuild. Changing C++ or anything in `rocko_interfaces` still needs a rebuild.

Rebuilding one package instead of everything:

```bash
colcon build --packages-select rocko_env --symlink-install
```

---

## Running it

```bash
# Balancing mode (the real thing)
ros2 launch rocko_env rocko.launch.py

# Tank mode (no balancing, just drives)
ros2 launch rocko_env rocko_tank_mode.launch.py

# Just look at the robot model in RViz, no hardware needed
ros2 launch rocko_env view_robot.launch.py description_package:=rocko_env
```

That `description_package:=rocko_env` bit is annoying but currently required, because the default value is wrong ([#11](https://github.com/BiPed-Capstone/ROCKO-env/issues/11)).

Useful launch arguments: `gui:=true` opens RViz alongside in tank mode (`rocko.launch.py` declares it but does not use it), without its saved layout for now: the layouts are never installed, finding L-4 in [the audit](docs/claude-analysis/balance-loop-findings.md)). `use_mock_hardware:=true` swaps the real motor and IMU plugins for ROS's `mock_components/GenericSystem` and skips the nodes that read real sensors. It defaults to `false`, so a plain launch on the robot gets real hardware. **The mock is not a simulation**: it has no physics, each command is just echoed back as a state, and pitch stays at 0.0. It checks that the controllers *wire up*, and tells you nothing about whether the robot balances (see [the verification ladder](docs/architecture/verification-ladder.md)). With the mock on, ROS never loads the motor driver, so nothing launched by these files can move the wheels; with it off (the default), it does.

You can also run individual nodes, which is handy when debugging one piece:

```bash
ros2 run rocko_env DiffDriveController.py
ros2 run rocko_env QuadEncoders.py
ros2 run rocko_env ICM20948.py
ros2 run rocko_env Joystick.py
```

And the gamepad script, which runs **on your laptop** while the robot is running:

```bash
python3 joystick/computer_joystick.py
```

It opens a TCP connection to `Joystick.py` on the Pi at port 1234, so both machines need to be on the same network and you'll need the Pi's IP address.

### Handy debugging commands

```bash
ros2 topic list                          # what topics exist right now
ros2 topic echo /left_feedforward        # watch values on a topic live
ros2 control list_controllers            # which controllers loaded, and their state
ros2 control list_hardware_interfaces    # what the hardware layer is exposing
```

If a controller shows as `inactive` or missing, that's usually a bad name or interface in `rocko_controllers.yaml`.

### Working on a laptop instead of the robot

macOS and Windows have no usable ROS 2 Jazzy install, so `docker/` at the repo root has a dev container: a single `ros:jazzy`-based image, used the same way on both OSes. The OS-specific bits (GUI display forwarding) live in small compose overrides, not in the image itself.

```bash
# From the ROCKO-env repo root

# macOS
docker compose -f docker/compose.yml -f docker/compose.mac.yml up -d
docker compose -f docker/compose.yml -f docker/compose.mac.yml exec ros2 bash

# Windows (run from inside a WSL2 distro, with Docker Desktop's WSL2 integration on)
docker compose -f docker/compose.yml -f docker/compose.windows.yml up -d
docker compose -f docker/compose.yml -f docker/compose.windows.yml exec ros2 bash
```

See the comments in `docker/compose.mac.yml` / `docker/compose.windows.yml` for the one-time XQuartz/WSLg setup each OS needs.

Inside the container, it's the [Building it](#building-it) workflow above with one difference (the flag on the `colcon build` line, because there is no WiringPi off the Pi) — this repo is bind-mounted at `/ros2_ws/src/ROCKO-env` (no separate Docker copy of the packages to keep in sync), and `build/`/`install`/`log` live in a named Docker volume, not on your host:

```bash
cd /ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DROCKO_SKIP_WIRINGPI=ON
source install/setup.bash
```

`./build_ros.sh` (repo root) runs these same steps, plus `sudo apt-get update` first, and exits 1 if the build fails. Inside the container run it as `cd /ros2_ws && src/ROCKO-env/build_ros.sh`: the `/ros2_ws/build_ros.sh` copy is a separate single-file mount that can go stale if the file is replaced while the container runs. In the container, compose sets `ROS2_WS=/ros2_ws` so the script builds in the right place, and `ROCKO_SKIP_WIRINGPI=ON` so it leaves out the motor plugin; on the Pi it defaults to `~/ros2_ws` and requires WiringPi.

There is one container per laptop, and it sees one folder: the *checkout* (a folder holding a copy of this repo) that last ran `docker compose ... up -d`. If you have more than one checkout (a second clone, or a git worktree: an extra working folder for the same repository, made with `git worktree add`), or you moved or re-cloned the repo, run `./check_container_mount.sh` **on the laptop, from that checkout's root**, before building. Otherwise the commands above can run, and pass, against the other checkout's files. [Verify a change without the robot](docs/guides/verify-in-container.md) has the details and what to do when it fails.

**What the container can and cannot do.** The container runs on your laptop's CPU, not the Pi's, and the motor plugin needs WiringPi, which only exists on real Pi GPIO hardware — a hardware limit, not an OS one, so Docker can't route around it. So the container build:

- **builds** `rocko_interfaces` and `rocko_env`, **without** `Motor12VoltQuadEncoder.cpp`. A green container build says nothing about that file; only a build on the Pi compiles it.
- **can run the mock**: `verify_mock.sh` launches the controllers against `mock_components/GenericSystem` and checks they load, chain and stay active. Run it with `docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_mock.sh` (add `tank` for tank mode). The step-by-step version, with the expected output and what to do when it fails, is [Verify a change without the robot](docs/guides/verify-in-container.md).
- **cannot simulate the robot.** There is no physics; see [the verification ladder](docs/architecture/verification-ladder.md) and [ADR 0003](docs/decisions/0003-simulation-target.md). It also cannot drive motors or read the real IMU.

Without `-DROCKO_SKIP_WIRINGPI=ON` the build stops with "WiringPi not found", on purpose: a robot should never be built without its motor driver by accident. If a build directory was ever used for a skip build and you later build on a Pi, run `rm -rf build/rocko_env` first: CMake remembers the flag between builds (it stops with "WiringPi IS installed here" when that has happened). In a skip build, launching **without** `use_mock_hardware:=true` fails with `Could not create class of type rocko_env::Motor12VoltQuadEncoder`, because that plugin is not in it.

---

## Documentation

[`docs/`](docs/) holds everything that doesn't belong on the front page.

| Where | What's in it |
| --- | --- |
| [`docs/architecture/`](docs/architecture/) | How a subsystem works and **why** — start with [the control cascade](docs/architecture/control-cascade.md) |
| [`docs/guides/`](docs/guides/) | Procedures — including [how to record a result so it reproduces](docs/guides/reproducing-results.md) |
| [`docs/decisions/`](docs/decisions/) | Architecture Decision Records: a choice, its alternatives, its cost |
| [`docs/journal/`](docs/journal/README.md) | One entry per AI-written change: what was asked, what changed, which checks actually ran |
| [`docs/proposals/`](docs/proposals/README.md) | Changes waiting on a human decision (gains, version pins, safety behavior) |
| [`docs/claude-analysis/`](docs/claude-analysis/) | Code audits with findings and severities |

**If you are new**, read this README, then
[`docs/architecture/control-cascade.md`](docs/architecture/control-cascade.md) for why "go faster"
means "lean further", then
[`docs/claude-analysis/balance-loop-findings.md`](docs/claude-analysis/balance-loop-findings.md) for
an honest account of what is wrong with the balance loop today and in what order to fix it.

**If you are about to change something**, the relevant ADRs are
[0001 — the version lock](docs/decisions/0001-version-lock.md) (Jazzy and ros2_control 4.x are
pinned, and the build enforces it) and
[0002 — the cascade](docs/decisions/0002-cascaded-balance-control.md).

### Working with AI agents in this repo

This is the **`ai-forward` branch**. The code here is meant to be written by AI under human
direction; `main` uses a human-gated workflow instead. Why they differ:
[ADR 0005](docs/decisions/0005-ai-forward-development.md).

[`CLAUDE.md`](CLAUDE.md) is the working agreement every Claude Code session loads automatically. The
short version: an agent takes a goal, plans, implements, **checks its own work** against the
[verification ladder](docs/architecture/verification-ladder.md), has reviewer agents inspect it, and
iterates — without asking permission along the way. It still keeps the code simple, explains every
change in [`docs/journal/`](docs/journal/README.md), and stops for a human on the things it cannot
verify: PID gains, version pins, and weakening a safety behavior (it writes a
[proposal](docs/proposals/README.md) instead). It cannot claim that balancing works — it has no
robot. Repeatable workflows live in [`.claude/skills/`](.claude/skills/) and review agents in
[`.claude/agents/`](.claude/agents/).
