# Run ROCKO on the Pi: the helper scripts

**Who this is for:** anyone who types a `ros2` command on the robot.

**Which machine:** the Pi. (The keyboard and gamepad scripts run on your laptop and need none of this.)

**What has been checked:** the scripts are tested against a fake ROS, a fake `sudo` and a fake `ros2`
(`tools/pi_scripts_test.sh`; what it proves, how many checks it has and what it cannot prove are in
the [verification ladder](../architecture/verification-ladder.md)). **They have not been run on the Pi.**
Whether the real `sudo` accepts them and whether WiringPi then works is checked by step 1 of
[the tank-mode bench test](bench-test-tank-mode.md).

---

## Why every `ros2` command on the Pi runs as root

The motor plugin drives the GPIO pins through WiringPi, and WiringPi only works as root. So the
launch has to run as root. And a `ros2` command run by an ordinary user usually cannot see a launch
run by root: ROS 2's programs talk through shared memory that belongs to one user. The symptom is a
terminal that sees nothing:

```text
waiting for service /controller_manager/list_controllers to become available...
```

So on the Pi **every terminal that runs `ros2` uses root and the same `ROS_DOMAIN_ID`** (17). The
scripts below do both, and set up the rest of the terminal (ROS, the Python virtual environment, the
workspace's build) the way your hand-written helper did.

## The workspace is the checkout

On the Pi the colcon workspace (the folder holding `build/`, `install/` and `log/`) is the repo
checkout itself, `~/ROCKO-env`. There is no `~/ros2_ws` and no `src/` folder. (The Docker dev container on
a laptop is different: there the checkout sits inside a workspace, `/ros2_ws/src/ROCKO-env`.)

## The scripts (in `pi/`, run from the repo root)

| Script | What it does | Use it for |
| --- | --- | --- |
| `pi/launch.sh tank` or `pi/launch.sh balance` | Launches `rocko_tank_mode.launch.py` or `rocko.launch.py` as root. Anything after the mode goes to `ros2 launch`: `pi/launch.sh tank gui:=true` | Starting the robot |
| `pi/rocko_shell.sh` | Opens a root shell that already has the environment, in the workspace. `exit` leaves it. The prompt ends in `#` | One per extra terminal: `ros2 control list_controllers`, `ros2 topic echo ...` |
| `pi/rocko_run.sh <command>` | Runs one command as root in the environment | A single command without opening a shell: `pi/rocko_run.sh ros2 topic list` |
| `pi/rocko_env.sh` | **Sourced**, not run: `source pi/rocko_env.sh`. Sets up the terminal as your user (no root) | Building: it needs no root |

Typical session, two terminals (SSH twice). Each command starts with `sudo`'s password prompt the
first time:

```bash
cd ~/ROCKO-env
pi/launch.sh tank
```

```bash
cd ~/ROCKO-env
pi/rocko_shell.sh
```

then, inside that second shell:

```bash
ros2 control list_controllers
```

Success: the second shell prints `rocko_env: user=root domain=17 workspace=/home/rocko/ROCKO-env`,
and `list_controllers` lists `joint_state_broadcaster` and `diffbot_base_controller` as `active`
(tank mode) rather than waiting.

Hardware is real unless you add `use_mock_hardware:=true`. The scripts use root even then, so that all
terminals agree.

### Settings you can override

Set these in the terminal before running a script. The defaults are the robot's.

| Variable | Default | Meaning |
| --- | --- | --- |
| `ROCKO_WS` | the checkout folder | the workspace: where `install/` is and where the shell starts |
| `ROCKO_VENV` | `~/.venv` (the **calling** user's) | the Python virtual environment |
| `ROCKO_DOMAIN_ID` | `17` | `ROS_DOMAIN_ID`. All terminals on the robot must agree |
| `ROCKO_ROS_SETUP` | `/opt/ros/jazzy/setup.bash` | ROS's setup file |

### Two things the scripts do for you

- **No `.pyc` files from root.** The scripts set `PYTHONDONTWRITEBYTECODE=1`, so a launch run as root
  does not leave root-owned `__pycache__` folders inside your checkout (`--symlink-install` runs the
  Python nodes straight from the repo).
- **Recordings go to your `~/rocko_logs`.** Run as root, `record:=true` would write to
  `/root/rocko_logs`, which `tools/fetch_logs.sh` cannot see. `pi/launch.sh` adds
  `log_root:=$HOME/rocko_logs` (your home) unless you pass your own `log_root:=`. The run folders are
  still **owned by root**: delete one with `sudo rm -r ~/rocko_logs/<run folder>`.

## Building

Build as yourself, not as root (a root-owned `build/` and `install/` in your checkout is a nuisance):

```bash
cd ~/ROCKO-env
./build_ros.sh
```

`build_ros.sh` uses the checkout as the workspace by default on the Pi. After a build, open a new
`pi/rocko_shell.sh` so the root shell sees it.

## What commonly goes wrong

| You see | Why | Do |
| --- | --- | --- |
| `waiting for service /controller_manager/list_controllers to become available...` | The command ran as an ordinary user, or with another `ROS_DOMAIN_ID`, so it cannot see the launch | Run it through `pi/rocko_shell.sh` or `pi/rocko_run.sh` |
| `rocko_env: note: no Python venv at /root/.venv` | The script ran as root without knowing your venv. Only happens if you start it as root yourself, or `sudo` yourself | Run the script as your user and let it call `sudo`; or `export ROCKO_VENV=/home/rocko/.venv` first |
| `rocko_env: note: .../install/setup.bash not found: build first` | Nothing is built here yet | `./build_ros.sh` |
| `sudo: a terminal is required` or no password prompt when you run a script through `ssh pi 'pi/launch.sh tank'` | `sudo` needs a terminal to ask for the password, and a one-command `ssh` has none (not checked on this Pi) | `ssh -t pi ...`, or log in first and run the script there |
| A WiringPi or GPIO error as the motor plugin starts | The launch was probably not run as root | Use `pi/launch.sh`, not a bare `ros2 launch` |
| A launch started with `sudo ros2 launch ...` by hand fails to find ROCKO | `sudo` resets the environment (ROS, the workspace overlay and the domain are lost) | Use `pi/launch.sh`; that is what it is for |
