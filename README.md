# ROCKO-env

ROS 2 packaging for a two-wheeled self-balancing biped robot codenamed **ROCKO**.

The repo holds two ament packages:

| Package | Build type | Contents |
| --- | --- | --- |
| `rocko_env` | `ament_cmake` | `ros2_control` hardware plugins, a chainable controller, Python nodes, URDF/xacro description, launch files and controller configs |
| `rocko_interfaces` | `ament_cmake` | Custom `.msg` / `.srv` definitions |

The robot runs on a Raspberry Pi. Motors and GPIO are driven through WiringPi, and the IMU/encoders are read over I²C and GPIO, so **the full stack only builds and runs on a Pi** (see [Known gaps](#known-gaps)).

---

## Requirements

### ROS 2 / ros2_control

| Requirement | Version | Where it comes from |
| --- | --- | --- |
| ROS 2 distro | **Jazzy Jalisco** | Matches the ros2_control series below |
| `ros2_control` / `controller_manager` | **>= 4.0.0** (hard check) | `rocko_env/CMakeLists.txt` — build fails with `FATAL_ERROR` otherwise |
| `ros2_controllers` | Jazzy series (ships alongside ros2_control 4.x) | `diff_drive_controller`, `pid_controller`, `joint_state_broadcaster` |
| Ubuntu | 24.04 Noble | Jazzy's Tier 1 platform |
| Python | 3.12 | Whatever Noble/Jazzy provides |

> **Humble and Iron will not work.** They ship ros2_control 2.x and 3.x respectively and trip the `>= 4.0.0` check. The `pid_controller` config in `rocko_controllers.yaml` also uses the Jazzy-era `reference_and_state_interfaces` API.

### Toolchain

- CMake >= 3.16 (`rocko_env`), >= 3.8 (`rocko_interfaces`)
- A C++17 compiler (`target_compile_features(... cxx_std_17)`)
- `colcon`, `rosdep`, `xacro`

### ROS package dependencies

Declared in each `package.xml`; install them with `rosdep` rather than by hand.

**Build / link:** `hardware_interface`, `controller_interface`, `pluginlib`, `rclcpp`, `rclcpp_lifecycle`, `rclpy`, `generate_parameter_library`, `parameter_traits`, `backward_ros`, `rosidl_default_generators`

**Runtime:** `controller_manager`, `forward_command_controller`, `joint_state_broadcaster`, `joint_state_publisher_gui`, `robot_state_publisher`, `ros2controlcli`, `ros2launch`, `rviz2`, `xacro`, `std_msgs`, `control_msgs`, `geometry_msgs`, `sensor_msgs`

**Not declared in `package.xml` but required at runtime:** `foxglove_bridge` — `rocko.launch.py` shells out to it via `ExecuteProcess`. Install with `apt install ros-jazzy-foxglove-bridge`.

### System libraries (Raspberry Pi only)

| Library | Notes |
| --- | --- |
| [WiringPi](https://github.com/WiringPi/WiringPi) | GPIO + `softPwm`. Located by `find_library(WIRINGPI_LIBRARIES NAMES wiringPi)`. A vendored copy of `wiringPi.h` lives in `rocko_env/hardware/WiringPi/` for reference; the actual `.so` must be installed on the Pi. |

---

## File structure

```
ROCKO-env
├── examples/example_hardware/      Reference templates for new ros2_control hardware (C++ and Python)
├── joystick/computer_joystick.py   Runs on a laptop; streams gamepad input over TCP :1234
├── rocko_env/                      Main package
│   ├── CMakeLists.txt, package.xml
│   ├── rocko_env_hardware.xml      pluginlib exports: Motor12VoltQuadEncoder, ICM20948, QuadEncoder
│   ├── rocko_env_controllers.xml   pluginlib export:  AddFeedforwardController
│   └── rocko_env/
│       ├── bringup/{config,launch}     Controller YAML + the three top-level launch files
│       ├── controllers/
│       │   ├── add_feedforward/        C++ chainable controller (+ generate_parameter_library yaml)
│       │   ├── diffdrive/              DiffDriveController.py
│       │   ├── joystick/               Joystick.py — TCP server for the laptop gamepad
│       │   └── utils/                  RateLimiter.py
│       ├── description/
│       │   ├── urdf/, ros2_control/    xacro robot description + hardware interface definitions
│       │   ├── rviz/                   Saved RViz configs
│       │   └── launch/view_robot.launch.py
│       ├── hardware/
│       │   ├── WiringPi/               Vendored wiringPi.h (reference only)
│       │   ├── motors/                 Motor12VoltQuadEncoder — C++ actuator plugin
│       │   └── sensors/
│       │       ├── icm20948/           IMU: C++ sensor plugin, Python node, calibration scripts
│       │       └── quadEncoder/        Encoders: C++ sensor plugin + Python nodes
│       └── webserver/                  Flask app for live telemetry plots
└── rocko_interfaces/                   Icm20948Data.msg, Icm20948Data.srv, QuadEncoderData.srv
```

There are two robot configurations:

- **Balancing** (`rocko.urdf.xacro` + `rocko_controllers.yaml`) — cascaded PID: velocity → pitch → per-wheel feedforward.
- **Tank mode** (`rocko_tank_mode.urdf.xacro` + `rocko_tank_mode_controllers.yaml`) — plain `diff_drive_controller`, no balancing.

---

## Building locally

This repo is *not* a colcon workspace on its own — clone it into the `src/` of one.

```bash
# 1. Create a workspace and clone
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone <this-repo> ROCKO-env

# 2. Pull dependencies
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
sudo apt install ros-jazzy-foxglove-bridge   # not covered by rosdep, see above

# 3. On a Pi only: install WiringPi so the C++ hardware can link
#    https://github.com/WiringPi/WiringPi

# 4. Build
colcon build --symlink-install

# 5. Source the overlay (every new shell)
source ~/ros2_ws/install/setup.bash
```

`--symlink-install` is worth it here: Python nodes, launch files, and YAML/xacro configs are symlinked rather than copied, so edits take effect without rebuilding. Changes to C++ or to `rocko_interfaces` still need a rebuild.

To rebuild just one package: `colcon build --packages-select rocko_env --symlink-install`.

### Running

```bash
# Balancing mode (the main one)
ros2 launch rocko_env rocko.launch.py

# Tank mode
ros2 launch rocko_env rocko_tank_mode.launch.py

# Description only — RViz + joint_state_publisher_gui, no hardware
ros2 launch rocko_env view_robot.launch.py description_package:=rocko_env
```

The `description_package:=rocko_env` override is currently required — see [Known gaps](#known-gaps).

Common launch args: `gui:=true` to bring up RViz, `use_mock_hardware:=true|false` (see the caveat below).

Individual Python nodes are installed to `lib/rocko_env` and can be run directly:

```bash
ros2 run rocko_env DiffDriveController.py
ros2 run rocko_env QuadEncoders.py
ros2 run rocko_env ICM20948.py
ros2 run rocko_env Joystick.py
```

The gamepad script runs on your laptop, not the Pi:

```bash
python3 joystick/computer_joystick.py    # connects to the Pi's Joystick.py on TCP :1234
```

### Developing off-robot

macOS and Windows have no usable ROS 2 Jazzy install — use a Linux machine, or a `ros:jazzy` Docker container with the workspace bind-mounted. The container route works the same on both host OSes; keep the clone inside the WSL2 filesystem on Windows for reasonable build speed.

Note that the C++ hardware still won't compile without WiringPi, so an off-robot build currently needs either a WiringPi stub or CMake guards around the Pi-specific sources.

---

## Known gaps

These are real limitations in the current tree, not TODOs someone forgot to delete:

- **`use_mock_hardware` does nothing.** The launch files declare it (default `true`) and pass it into xacro, but `rocko.ros2_control.xacro` and `rocko_tank_mode.ros2_control.xacro` accept the parameter and never branch on it. Both always instantiate the real Pi hardware plugins. Making it swap in `mock_components/GenericSystem` is the prerequisite for any off-robot testing.
- **No simulation.** There is no Gazebo / `gz_ros2_control` integration and no inertial data tuned for it. Balance-controller gains cannot currently be tuned anywhere but on the physical robot.
- **`view_robot.launch.py` has a broken default.** `description_package` defaults to `"description"`, and line 82 feeds that to `FindPackageShare` to locate `rviz/rocko_view.rviz` — but no package by that name exists, so the launch fails unless you pass `description_package:=rocko_env`. (Line 72 hardcodes `rocko_env` correctly for the xacro, so only the RViz config path is affected.)
- **No tests.** `package.xml` declares `ament_cmake_pytest`, `launch_testing_ros` and `liburdfdom-tools` as test deps, but no test files exist.
- **Python deps are unpinned.** No `requirements.txt`.
- **`rocko_env/setup.py` is vestigial.** The package builds as `ament_cmake` and installs Python via `ament_python_install_package`, so `setup.py` is never invoked — and it references a `resource/rocko_env` marker file that isn't in the repo.
- **`package.xml` metadata is inherited from the ros2_control demo template** — maintainers and description still point at upstream authors.
