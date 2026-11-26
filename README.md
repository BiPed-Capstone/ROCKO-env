# ROCKO-env

This is the ROS 2 code for **ROCKO**, a two-wheeled self-balancing robot built as an educational capstone platform.
Everything here runs on a Raspberry Pi mounted on the robot. All of it except the motor plugin can be built and checked
on a laptop (the motor plugin needs the Pi's GPIO library, so off the Pi it is only type-checked).

**Where it stands** (details in the handoff box of [next-steps.md](docs/next-steps.md)):

- **Tank mode drives** on the robot from a USB gamepad. It is plain differential drive, with no balancing.
- **Balance mode has never balanced.** It can be enabled on the robot, and the first floor test (2026-11-25) did not
  balance: [journal](docs/journal/2026-11-25-level-pitch-offset-and-lean-clamp.md).
- **Off the robot**, the code builds and is checked on wiring, on a mock and against a simulated robot. What those
  checks can and cannot show is in [the verification ladder](docs/architecture/verification-ladder.md).

---

## Start here

| You want to | Go to |
| --- | --- |
| Get set up on a laptop and run every check, with no robot | [Getting started: your first hour](docs/guides/getting-started.md) |
| Understand how it balances | [How the pieces connect](#how-the-pieces-connect), then [the control cascade](docs/architecture/control-cascade.md) |
| Look up a word | [The glossary](docs/glossary.md) |
| Run or drive the real robot | [Run ROCKO on the Pi](docs/guides/run-on-the-pi.md), [Drive with a USB controller](docs/guides/drive-with-usb-controller.md) |
| Find something to work on | [next-steps.md](docs/next-steps.md), the work queue |
| Decide what the AI left for a human | [Proposals waiting on you](docs/proposals/README.md#waiting-on-you) |
| Work on it with an AI agent | [Working with AI agents](#working-with-ai-agents-in-this-repo) below, and [CLAUDE.md](CLAUDE.md) |

---

## How the pieces connect

The project is built on **ROS 2**, a framework for robot software. The two ideas you need on day one:

- A **node** is a program that does one job (read the gamepad, run a PID loop).
- Nodes talk to each other over **topics**, which are named message channels. One node publishes to `/joy`, others
  subscribe to it. Nobody calls anybody directly.

On top of that we use **ros2_control**, a standard way to separate *"how do I physically talk to this motor"* from
*"what should the motor do right now."* That split explains the folder layout:

- A **hardware interface** is a C++ plugin that owns a real device. `Motor12VoltQuadEncoder` sets a motor's power
  (PWM) and counts its wheel encoder; `BNO055` reads the IMU, the sensor that measures lean (a Bosch BNO055). These are the only files that touch the
  Pi's pins.
- A **controller** reads sensor values and decides what to command. Controllers can be *chained*: one controller's
  output becomes the next one's input. That chaining is how the balancing works.
- The **controller_manager** loads both and runs them on a fixed clock (100 Hz).

The [official ros2_control docs](https://control.ros.org/jazzy/index.html) are the best background reading.

Here is the signal path in balance mode, from gamepad to spinning wheel. The gamepad is plugged into the Pi's USB;
nothing runs on your laptop:

```
 the Raspberry Pi
┌────────────────────────────────────────────────┐
│ 8BitDo pad ──USB──> joy's game_controller_node │
│                      publishes /joy            │
└───────────┬────────────────────────┬───────────┘
            v                        v
┌───────────────────────┐  ┌───────────────────────────────────────┐
│ Supervisor.py         │  │ BalanceTeleop.py                      │
│  LB+RB: enable        │  │  drives only while /rocko/state says  │
│  LT+RT+RB: mode       │─>│  enabled; publishes                   │
│  publishes            │  │  diffbot_base_controller/cmd_vel      │
│  /rocko/state         │  └───────────────────┬───────────────────┘
└───────────────────────┘                      │
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

To go *faster*, the robot first has to *lean* further forward. So the velocity loop does not command wheel speed
directly: it commands a target lean angle, and the pitch loop underneath works out the wheel speed needed to hold that
lean. The IMU supplies the actual pitch, and the wheel encoders the actual speed.

**Two modes, one supervisor.** `Supervisor.py` starts at boot, owns which mode runs and whether the robot is
**enabled**, and starts and stops each mode's programs. The Pi boots into **tank mode, disabled**. Press **LB + RB** on the gamepad to
enable or disable, and **LT + RT + RB** to switch mode (only while disabled); the web dashboard has the same buttons.
[The robot supervisor](docs/architecture/robot-supervisor.md).

- **Balance mode**: the cascade above. `rocko.launch.py`, `rocko.urdf.xacro`, `rocko_controllers.yaml`.
- **Tank mode**: plain differential drive, no balancing: `TankTeleop.py` feeds `diff_drive_controller` directly.
  `rocko_tank_mode.launch.py`, `rocko_tank_mode.urdf.xacro`, `rocko_tank_mode_controllers.yaml`.

---

## What you need

**ROS 2 Jazzy on Ubuntu 24.04**, on the Pi. This is not a preference: `rocko_env/CMakeLists.txt` stops the build if
`controller_manager` is older than 4.0.0, and Humble and Iron ship older ones.
[ADR 0001](docs/decisions/0001-version-lock.md) has the full version lock (ros2_control ≥ 4.0.0, C++17, CMake ≥ 3.16)
and why.

- **ROS packages**: listed in the two `package.xml` files; `./build_ros.sh` installs them with `rosdep`.
- **WiringPi** (Pi only): the motor plugin uses it for GPIO and PWM, and the build stops with "WiringPi not found"
  without it. A laptop build leaves the motor plugin out instead (`-DROCKO_SKIP_WIRINGPI=ON`).
- **Python packages**: none for the robot's own code. One optional bench tool, `tools/imu_tilt_log.py`, needs
  `Adafruit-Blinka`, `gpiod` and `adafruit-circuitpython-bno055` (not pinned:
  [#14](https://github.com/BiPed-Capstone/ROCKO-env/issues/14)).
- **A laptop** (macOS, or Windows with WSL2) needs only Docker Desktop: the dev container in `docker/` has the rest.

---

## Where things live

```
ROCKO-env
├── CLAUDE.md                 The working agreement every AI agent loads (and the rules for people too)
├── build_ros.sh              Installs dependencies and builds (container or Pi)
├── check_container_mount.sh  Laptop: is the dev container showing THIS checkout?
├── verify_*.sh               The verification ladder's checks, run in the container (tools/run_ladder.sh runs them all)
├── check_motor_plugin_compiles.sh  Type-checks the motor plugin off the Pi (container)
├── pi/                       Runs on the Pi: launch.sh, rocko_shell.sh, rocko_run.sh (ros2 as root), the boot service
├── docker/                   The laptop dev container
├── docs/                     Everything that is not on this page (map below)
├── examples/example_hardware/  Skeleton to copy when adding a hardware plugin
├── sim/                      The simulator's checks, scenarios and parameter sweeps (container)
├── tools/                    run_ladder.sh, host tests (C++ and Python, no ROS needed), Pi helpers, link and cost tools
├── rocko_interfaces/         Custom messages and services (the supervisor's RobotState, SetEnabled, SetMode)
└── rocko_env/                The main package
    ├── CMakeLists.txt, package.xml, rocko_env_hardware.xml, rocko_env_controllers.xml
    └── rocko_env/
        ├── bringup/config/   Controller settings and PID gains (a human changes the hardware gains: CLAUDE.md §3)
        ├── bringup/launch/   rocko_robot.launch.py (what boots), rocko.launch.py, rocko_tank_mode.launch.py, rocko_sim.launch.py
        ├── controllers/      add_feedforward (C++), diffdrive, tank_teleop, balance_teleop
        ├── supervisor/       Supervisor.py, with its rules in state_machine.py and the mode table in modes.py
        ├── teleop/gamepad.py The controller layout, stick shaping and button combos, shared by everything that reads /joy
        ├── description/      The robot's URDF/xacro, which plugin drives which joint, RViz layouts
        ├── hardware/         motors/ (PWM and encoders), sensors/bno055/ (the IMU), sim/ (SimPlant, the simulated robot)
        ├── recording/        RecordRun.py: record:=true writes a rosbag and a run.yaml
        └── webserver/        Dashboard.py: live values and buttons in a browser, port 8080, no login
```

---

## Build and run

**On a laptop:** follow [Getting started](docs/guides/getting-started.md). In short, `docker compose -f
docker/compose.yml up -d`, then `tools/run_ladder.sh` builds everything in the container and runs every check.

**On the Pi:** build as yourself, run as root through the scripts in `pi/` (the motor plugin needs root).
[Run ROCKO on the Pi](docs/guides/run-on-the-pi.md) explains why and covers the boot service.

```bash
cd ~/ROCKO-env
./build_ros.sh                                   # build (as yourself, not root)
pi/launch.sh robot                               # the whole robot, as at boot: supervisor, tank mode disabled, dashboard
pi/launch.sh robot tank_max_speed:=0.3 balance_max_speed:=0.02   # the same, with speed caps for a first test
pi/launch.sh tank                                # one mode by hand, no supervisor (bench tests)
```

If the boot service (`rocko.service`) is installed, the robot is already running after power-on, and `pi/launch.sh`
refuses to start a second launch until `sudo systemctl stop rocko.service`.

**Driving:** plug the gamepad into the Pi's USB, press **LB + RB** to enable; left stick is speed, right stick
is turn. [Drive ROCKO with a USB controller](docs/guides/drive-with-usb-controller.md). Add `record:=true` to keep a
recording ([Record a run](docs/guides/record-a-run.md)); open `http://<robot-ip>:8080` for
[the dashboard](docs/guides/use-the-dashboard.md).

`use_mock_hardware:=true` on any launch swaps the motor and IMU plugins for a mock that echoes commands back. **The mock
is not a simulation**: it has no physics and pitch stays 0.0, so it shows that controllers wire up and nothing about
balancing. With it, ROS never loads the motor driver, so nothing launched can move the wheels. It defaults to `false`,
so a plain launch on the robot gets real hardware.

---

## Documentation

[`docs/`](docs/README.md) holds everything that doesn't belong on the front page.

| Where | What's in it |
| --- | --- |
| [`docs/guides/`](docs/guides/README.md) | Procedures: do this, get that, on which machine |
| [`docs/architecture/`](docs/architecture/README.md) | How a subsystem works and **why**; start with [the control cascade](docs/architecture/control-cascade.md) |
| [`docs/decisions/`](docs/decisions/) | Architecture Decision Records: a choice, its alternatives, its cost |
| [`docs/glossary.md`](docs/glossary.md) | The words of the first week |
| [`docs/next-steps.md`](docs/next-steps.md) | The work queue, and the current handoff |
| [`docs/proposals/`](docs/proposals/README.md) | Changes waiting on a human decision (gains, version pins, safety behavior) |
| [`docs/journal/`](docs/journal/README.md) | One entry per AI-written change: what was asked, what changed, which checks actually ran |
| [`docs/claude-analysis/`](docs/claude-analysis/) | Code audits with findings and severities, and the measured cost of the agent loop |

### Working with AI agents in this repo

This is the **`ai-forward` branch**. The code here is written by AI under human direction; `main` uses a human-gated
workflow instead. Why they differ: [ADR 0005](docs/decisions/0005-ai-forward-development.md).

[`CLAUDE.md`](CLAUDE.md) is the working agreement every Claude Code session loads automatically. An agent takes a goal,
plans, implements, **checks its own work** against the [verification ladder](docs/architecture/verification-ladder.md),
has reviewer agents inspect it, and iterates without asking permission along the way. It explains every change in
[`docs/journal/`](docs/journal/README.md), and stops for a human on what it cannot verify: PID gains, version pins and
weakening a safety behavior (it writes a [proposal](docs/proposals/README.md) instead). It cannot claim that
balancing works: it has no robot. Repeatable workflows live in [`.claude/skills/`](.claude/skills/) and review agents
in [`.claude/agents/`](.claude/agents/).
