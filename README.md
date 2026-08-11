# ROCKO-env

This is the ROS 2 code for **ROCKO**, a two-wheeled self-balancing robot. Everything here runs on a Raspberry Pi mounted on the robot.

---

## Getting Started

The project is built on **ROS 2**, a framework for robot software. The two ideas you need on day one:

- A **node** is just a program that does one job (read the IMU, talk to the gamepad, run a PID loop).
- Nodes talk to each other over **topics**, which are named message channels. One node publishes to `left_feedforward`, another subscribes to it. Nobody calls anybody directly.

On top of that we use **ros2_control**, which is a standard way to separate *"how do I physically talk to this motor"* from *"what should the motor do right now."* That split explains the whole folder layout:

- A **hardware interface** is a C++ plugin that owns a real device. `Motor12VoltQuadEncoder` writes PWM to a motor pin; `ICM20948` reads the IMU. These are the only files that touch GPIO.
- A **controller** reads sensor values and decides what to command. Controllers can be *chained*, meaning one controller's output becomes the next one's input — which is exactly how the balancing works.
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

To control the robot to go *faster*, the robot first has to *lean* further forward. So the velocity loop doesn't command wheel speed directly — it commands a target lean angle, and the pitch loop underneath it works out the wheel speed needed to hold that lean. The IMU supplies the actual pitch, and the wheel encoders supply the actual velocity.

There are two ways to run the robot:

- **Balancing mode** — the cascade above. This is the real robot. Uses `rocko.urdf.xacro` + `rocko_controllers.yaml`.
- **Tank mode** — balancing disabled, plain differential drive, for when you just want it to move without fighting the control loop. Uses `rocko_tank_mode.urdf.xacro` + `rocko_tank_mode_controllers.yaml`.

---

## What you need installed

### ROS 2 version — this matters, please read

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

These are all listed in the `package.xml` files, and `rosdep` (step 2 of the build below) installs them for you — you shouldn't need to install any by hand. Listed here so you know what's in play:

**Used to build:** `hardware_interface`, `controller_interface`, `pluginlib`, `rclcpp`, `rclcpp_lifecycle`, `rclpy`, `generate_parameter_library`, `parameter_traits`, `backward_ros`, `rosidl_default_generators`

**Used at runtime:** `controller_manager`, `forward_command_controller`, `joint_state_broadcaster`, `joint_state_publisher_gui`, `robot_state_publisher`, `ros2controlcli`, `ros2launch`, `rviz2`, `xacro`, `std_msgs`, `control_msgs`, `geometry_msgs`, `sensor_msgs`

⚠️ **One exception:** `rocko.launch.py` starts `foxglove_bridge`, but it isn't listed in `package.xml`, so `rosdep` won't catch it. Install it yourself:

```bash
sudo apt install ros-jazzy-foxglove-bridge
```

### WiringPi (Raspberry Pi only)

The motor code uses [WiringPi](https://github.com/WiringPi/WiringPi) for GPIO and software PWM. There's a copy of `wiringPi.h` checked into `rocko_env/hardware/WiringPi/` for reference, but that header alone isn't enough — the actual compiled library has to be installed on the Pi or the build won't link.

This is also the reason you can't build the full project on a normal laptop. More on that below.

### Python libraries

Heads up: **none of these are pinned** — there's no `requirements.txt` yet, so `pip` grabs the latest of each ([#14](https://github.com/BiPed-Capstone/ROCKO-env/issues/14)).

| Library | What uses it | Needs real Pi hardware? |
| --- | --- | --- |
| `numpy` | encoders, IMU, calibration | no |
| `flask` | the telemetry web server | no |
| `pygame` | the laptop gamepad script | no |
| `tqdm`, `ahrs` | IMU calibration scripts | no |
| `Adafruit-Blinka` (imported as `board`) | IMU | **yes** — needs real I²C |
| `adafruit-circuitpython-icm20x` | IMU | **yes** |
| `adafruit-circuitpython-bno055` | old IMU calibration script | **yes** |
| `Encoder` | wheel encoders | **yes** — needs real GPIO |

---

## Where things live

```
ROCKO-env
├── examples/example_hardware/      Skeleton templates — copy these when adding new hardware
├── joystick/computer_joystick.py   Runs on YOUR laptop, not the robot. Sends gamepad input over TCP
├── rocko_env/                      The main package
│   ├── CMakeLists.txt, package.xml     Build config and dependency list
│   ├── rocko_env_hardware.xml          Registers the hardware plugins with ROS
│   ├── rocko_env_controllers.xml       Registers the controller plugin with ROS
│   └── rocko_env/
│       ├── bringup/
│       │   ├── config/                 PID gains and controller settings — tune here
│       │   └── launch/                 The three "start everything" files
│       ├── controllers/
│       │   ├── add_feedforward/        C++ controller that adds a feedforward term
│       │   ├── diffdrive/              Turns steering commands into per-wheel targets
│       │   ├── joystick/               TCP server that receives laptop gamepad input
│       │   └── utils/                  RateLimiter.py — smooths out sudden changes
│       ├── description/
│       │   ├── urdf/                   The robot's physical shape and joints (xacro files)
│       │   ├── ros2_control/           Which hardware plugin drives which joint, and GPIO pins
│       │   ├── rviz/                   Saved RViz visualization layouts
│       │   └── launch/                 view_robot.launch.py — look at the model, no hardware
│       ├── hardware/
│       │   ├── WiringPi/               Reference copy of wiringPi.h
│       │   ├── motors/                 Motor driver — the only file writing motor PWM
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

- A **colcon workspace** is a folder with a `src/` subfolder holding one or more packages. `colcon` is the build tool. **This repo is not a workspace by itself** — it goes *inside* one.
- **Sourcing** a setup file (`source .../setup.bash`) is what tells your terminal where the ROS packages are. You have to do it in every new terminal window.

```bash
# 1. Make a workspace and clone this repo into its src/ folder
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/BiPed-Capstone/ROCKO-env ROCKO-env

# 2. Install dependencies
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
sudo apt install ros-jazzy-foxglove-bridge        # rosdep misses this one

# 3. On the Pi only: install WiringPi first, or step 4 fails to link
#    https://github.com/WiringPi/WiringPi

# 4. Build (run this from ~/ros2_ws, not from inside src/)
colcon build --symlink-install

# 5. Tell this terminal about what you just built
source ~/ros2_ws/install/setup.bash
```

**Why `--symlink-install`?** It links your Python files, launch files, and YAML configs into the build output instead of copying them. Practical upshot: you can edit a Python node or tweak PID gains and just re-run — no rebuild. Changing C++ or anything in `rocko_interfaces` still needs a rebuild.

Rebuilding one package instead of everything:

```bash
colcon build --packages-select rocko_env --symlink-install
```

---

## Running it

```bash
# Balancing mode — the real thing
ros2 launch rocko_env rocko.launch.py

# Tank mode — no balancing, just drives
ros2 launch rocko_env rocko_tank_mode.launch.py

# Just look at the robot model in RViz, no hardware needed
ros2 launch rocko_env view_robot.launch.py description_package:=rocko_env
```

That `description_package:=rocko_env` bit is annoying but currently required — the default value is wrong ([#11](https://github.com/BiPed-Capstone/ROCKO-env/issues/11)).

Useful launch arguments: `gui:=true` opens RViz alongside. There's also a `use_mock_hardware` argument, but be warned — **it currently does nothing at all**, so it won't protect you from driving real motors ([#10](https://github.com/BiPed-Capstone/ROCKO-env/issues/10)).

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

macOS and Windows have no usable ROS 2 Jazzy install, so you'd use a Linux machine or a `ros:jazzy` Docker container with the workspace mounted in. Docker behaves the same on Mac and Windows — on Windows, keep the clone inside the WSL2 filesystem or builds get very slow.

But even in a container, the C++ hardware won't compile, because WiringPi doesn't exist off the Pi. Making off-robot development work needs either a stub WiringPi or CMake guards around the Pi-only source files, plus finishing the `use_mock_hardware` path ([#10](https://github.com/BiPed-Capstone/ROCKO-env/issues/10)). Until then, laptop work is limited to editing code and reasoning about it.

