# Run ROCKO on the Pi: the helper scripts

**Who this is for:** anyone who types a `ros2` command on the robot.

**Which machine:** the Pi. (Driving is from a controller plugged into the Pi: [the guide](drive-with-usb-controller.md).)

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
| `pi/launch.sh robot` | Launches `rocko_robot.launch.py` as root: the supervisor, the controller reader, the dashboard and Foxglove. The supervisor starts tank mode, **disabled**, and switches modes when asked | The normal way to run the robot, and what the boot service runs |
| `pi/launch.sh tank` or `pi/launch.sh balance` | Launches `rocko_tank_mode.launch.py` or `rocko.launch.py` by hand as root, **with no supervisor**. Anything after the mode goes to `ros2 launch`: `pi/launch.sh tank gui:=true` | Bench tests of one stack. The controller cannot enable these (the teleop nodes wait for the supervisor) |
| `pi/rocko_shell.sh` | Opens a root shell that already has the environment, in the workspace. `exit` leaves it. The prompt ends in `#` | One per extra terminal: `ros2 control list_controllers`, `ros2 topic echo ...` |
| `pi/rocko_run.sh <command>` | Runs one command as root in the environment | A single command without opening a shell: `pi/rocko_run.sh ros2 topic list` |
| `pi/install_boot_service.sh robot` | Installs (or with `status` / `remove`, checks or removes) the systemd service `rocko.service` that starts `pi/launch.sh robot` at boot, as root | Once: [below](#the-boot-service-the-supervisor). **While it runs, `pi/launch.sh` refuses to start another launch**: `sudo systemctl stop rocko.service` first |
| `pi/rocko_env.sh` | **Sourced**, not run: `source pi/rocko_env.sh`. Sets up the terminal as your user (no root) | Building: it needs no root |

Typical session, two terminals (SSH twice). Each command starts with `sudo`'s password prompt the
first time. If the robot starts at boot (`rocko.service`), stop it first with `sudo systemctl stop rocko.service`:
`pi/launch.sh` refuses to start while it runs.

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

## The boot service: the supervisor

`rocko.service` starts `pi/launch.sh robot` when the Pi boots. The supervisor inside it starts tank mode, **disabled**, so
the wheels cannot move until someone presses LB + RB on the controller or Enable on the dashboard
([Drive ROCKO with a USB controller](drive-with-usb-controller.md); design: [the robot supervisor](../architecture/robot-supervisor.md)).
**This has not been installed or run on the Pi yet** (written 2026-11-23; the install is the owner's).

On the **Pi**, as your own user (not root; the script calls `sudo` itself), from `~/ROCKO-env`, with the robot on blocks or
with space around it (it starts now as well):

```bash
pi/install_boot_service.sh robot
```

Success: `Installed /etc/systemd/system/rocko.service; enabled at boot; started.` Then check it:

```bash
pi/install_boot_service.sh robot status
```

Success: `enabled at boot: enabled`, `running now: active`, `restarts: 0`, and recent log lines from the supervisor.
Open `http://<robot-ip>:8080`: the state card says tank, DISABLED, phase ready.

To stop it starting at boot, and stop it now:

```bash
pi/install_boot_service.sh robot remove
```

- **Older Pis.** Until 2026-11-24 tank mode and the dashboard had boot services of their own (`rocko-tank.service`,
  `rocko-dashboard.service`; removed from the repo 2026-11-26). If either is still installed,
  `pi/install_boot_service.sh robot` stops and removes it and says so: two launches would fight over the motors.
- **The launch guard.** While `rocko.service` runs, `pi/launch.sh` refuses every
  launch, including `pi/launch.sh robot`, and prints how to stop it. This protects the motors from a second launch.
- **Before a hand launch or a bench test** (`pi/launch.sh tank`, `pi/launch.sh balance`, the
  [bench test](bench-test-tank-mode.md)): stop the service until the next boot, and check it stopped:

  ```bash
  sudo systemctl stop rocko.service
  ```

  ```bash
  pi/install_boot_service.sh robot status
  ```

  Success: `running now: inactive`. Bring it back with `sudo systemctl start rocko.service` (or reboot). A hand launch has
  no supervisor, so the controller cannot enable it: that is expected.
- **Stopping the service stops the whole stack.** The supervisor signals its child launch's whole process group, and
  systemd signals every process at once; this is deliberate (stopping only `ros2 launch` hung the motor plugin, finding C-1).
  If the launch has not exited 40 s after SIGINT, systemd kills it.
- If the launch fails to start, systemd retries at most 3 times in 2 minutes, then stays stopped (status shows it). A node
  that crashes **inside** a running launch does not restart the launch: the supervisor reports a fault on the dashboard
  (and, in tank mode, disables). Read the log: `sudo journalctl -u rocko.service`.
- The unit's template is `pi/rocko.service`; re-run the install after moving the checkout or the venv, because it writes
  your paths into the unit.

## Building

Once, before the first build: install [WiringPi](https://github.com/WiringPi/WiringPi), the library the motor plugin
uses for the Pi's pins. Without it the build stops with "WiringPi not found", on purpose: a robot must never be
built without its motor driver by accident.

Build as yourself, not as root (a root-owned `build/` and `install/` in your checkout is a nuisance):

```bash
cd ~/ROCKO-env
./build_ros.sh
```

`build_ros.sh` uses the checkout as the workspace by default on the Pi. After a build, open a new
`pi/rocko_shell.sh` so the root shell sees it.

The build uses `--symlink-install`: Python nodes, launch files and YAML are linked, not copied, so editing one needs
no rebuild. C++ and anything in `rocko_interfaces` do. After a `git pull` that removed or renamed files, build from
clean (`rm -rf build install log`, then `./build_ros.sh`): links to removed files stay behind in `install/`.
To rebuild one package only: `colcon build --packages-select rocko_env --symlink-install`.

## Handy debugging commands

In a terminal opened with `pi/rocko_shell.sh` (an ordinary terminal cannot see the launch):

```bash
ros2 topic list                          # what topics exist right now
ros2 topic echo /left_feedforward        # watch values on a topic live
ros2 control list_controllers            # which controllers loaded, and their state
ros2 control list_hardware_interfaces    # what the hardware plugins expose
```

A controller shown as `inactive`, or missing, is usually a wrong name or interface in the controllers YAML. The
supervisor keeps the cascade `inactive` on purpose while the robot is disabled.

To look at the robot model in RViz with no hardware (needs a display, so not over plain SSH):
`ros2 launch rocko_env view_robot.launch.py`. Not run since its default `description_package` changed
([#11](https://github.com/BiPed-Capstone/ROCKO-env/issues/11)); if it fails, add `description_package:=rocko_env`.

## What commonly goes wrong

| You see | Why | Do |
| --- | --- | --- |
| `waiting for service /controller_manager/list_controllers to become available...` | The command ran as an ordinary user, or with another `ROS_DOMAIN_ID`, so it cannot see the launch | Run it through `pi/rocko_shell.sh` or `pi/rocko_run.sh` |
| `rocko_env: note: no Python venv at /root/.venv` | The script ran as root without knowing your venv. Only happens if you start it as root yourself, or `sudo` yourself | Run the script as your user and let it call `sudo`; or `export ROCKO_VENV=/home/rocko/.venv` first |
| `rocko_env: note: .../install/setup.bash not found: build first` | Nothing is built here yet | `./build_ros.sh` |
| `sudo: a terminal is required` or no password prompt when you run a script through `ssh pi 'pi/launch.sh tank'` | `sudo` needs a terminal to ask for the password, and a one-command `ssh` has none (not checked on this Pi) | `ssh -t pi ...`, or log in first and run the script there |
| A WiringPi or GPIO error as the motor plugin starts | The launch was probably not run as root | Use `pi/launch.sh`, not a bare `ros2 launch` |
| `launch.sh: ROCKO is already running as the boot service (rocko.service)` | The launch guard: the supervisor already owns the motors | `sudo systemctl stop rocko.service`, as the message says |
| CMake stops with `WiringPi IS installed here` | This build folder was once used for a laptop-style build that left the motor plugin out (`ROCKO_SKIP_WIRINGPI`), and CMake remembers the flag | `rm -rf build/rocko_env`, then `./build_ros.sh` |
| A launch started with `sudo ros2 launch ...` by hand fails to find ROCKO | `sudo` resets the environment (ROS, the workspace overlay and the domain are lost) | Use `pi/launch.sh`; that is what it is for |
