# Verify a change without the robot

**Goal:** after you (or an AI agent) change something in this repo, find out whether the build still
works and the controllers still wire up, using only your laptop.

**What this does not tell you:** whether the robot balances. Nothing on a laptop can; see
[the verification ladder](../architecture/verification-ladder.md) for exactly what each step proves
and cannot show.

**Machine:** your laptop (macOS, or Windows inside WSL2) with Docker Desktop running. Every command
below is run from the **repo root on your laptop**. The ones that start with
`docker compose -f docker/compose.yml exec -T ros2` run a command *inside the container*; `-T`
because no terminal is attached (leave it off if you want an interactive shell).

## 1. Start the container

```bash
docker compose -f docker/compose.yml up -d
```

Success: the last lines say `Container rocko-env-dev  Started`. The first run downloads and builds an
image, which takes several minutes; later runs take seconds.

Check the container knows where it is:

```bash
docker compose -f docker/compose.yml exec -T ros2 bash -c 'echo $ROS2_WS $ROCKO_SKIP_WIRINGPI'
```

Success: it prints `/ros2_ws ON`. Anything else means the container predates those settings:
recreate it with `docker compose -f docker/compose.yml up -d`.

## 2. Build (Tier 1)

```bash
docker compose -f docker/compose.yml exec -T ros2 bash -c 'cd /ros2_ws && src/ROCKO-env/build_ros.sh'
```

What happens: it installs the dependencies that `package.xml` declares (`rosdep`), then builds both
packages. `-DROCKO_SKIP_WIRINGPI=ON` is passed for you, which leaves out the motor plugin because
WiringPi exists only on a Raspberry Pi.

Success: near the end you see `Finished <<< rocko_env`, `Summary: 2 packages finished` and `Done!`,
and the command exits 0. `2 packages had stderr output` is normal: CMake warnings go to stderr,
including the deliberate "building WITHOUT Motor12VoltQuadEncoder" notice. A failed build prints
`ERROR: colcon build failed` and exits 1.

Why `src/ROCKO-env/build_ros.sh` and not `./build_ros.sh`? The container also has a separate copy at
`/ros2_ws/build_ros.sh` that goes stale if the file is replaced while the container runs; the copy
inside `src/ROCKO-env/` is always the current one.

If it stops with `ERROR: workspace '...' not found`, the folder named by `ROS2_WS` does not exist.
In the container that should be `/ros2_ws`; if step 1's check printed something else, recreate the
container.

## 3. Static checks (Tier 0)

```bash
docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_urdf.sh
```

What happens: for balancing and tank mode, each with the mock off and on, it expands the xacro (a
*xacro* file is XML with macros), checks the result is a valid robot description, and then checks the
right hardware is in it: the two real motor plugins (and the IMU, in balancing mode) when the mock is
off, exactly one `MockRocko` system when it is on. Finally it checks the mock exports **exactly the
same interfaces** (joints, sensors and gpio, duplicates included) as the real hardware, so the mock
cannot quietly drift from the robot. The
content checks matter because a valid-looking description passes the first two checks whichever
hardware was selected. It needs step 2 first, because the xacro files find each other through the
installed package.

Success:

```
ok    rocko mock=false
ok    rocko mock=true
ok    rocko: the mock exports the same 12 interfaces as the real hardware
ok    rocko_tank_mode mock=false
ok    rocko_tank_mode mock=true
ok    rocko_tank_mode: the mock exports the same 6 interfaces as the real hardware
PASS: all four combinations expand, parse, hold the right hardware, and the mock matches the real interfaces.
```

A `FAIL` line names the combination and what it expected. Exit status is 0 on a pass, 1 on a
failure, 2 if there is no build yet.

The guard hook that stops an AI from editing PID gains or version pins has its own tests. Run them
on your laptop, no container needed:

```bash
.claude/hooks/test-guard-protected-files.sh
```

Success: every *case* line starts `ok` (the `---` lines are section headings), and the exit status
is 0. A `skip` line is fine: the one case about differently-cased paths only applies on a
case-insensitive filesystem (macOS, Windows), so Linux and WSL2 skip it.

## 4. Run the mock (Tier 2)

```bash
docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_mock.sh
docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_mock.sh tank
```

What happens: it launches ROCKO with `use_mock_hardware:=true`, waits for every controller that
launch file spawns to become `active`, checks the interfaces they need exist and are claimed, then
insists everything is still up a few seconds later. It is bounded to about 80 s even when something is wrong (plus a few seconds to shut down); the
usual bring-up time was not measured.

Success (balancing mode):

```
PASS (balance, mock hardware): all controllers active, interfaces present and claimed.
  controllers: left_add_feedforward_controller right_add_feedforward_controller pitch_pid_controller velocity_pid_controller
  Reminder: this is a wiring check with no physics. It does not show the robot balances.
```

Tank mode prints `PASS (tank, mock hardware)` and `controllers: diffbot_base_controller
joint_state_broadcaster`.

A failure prints `FAIL`, a bullet for each thing that is wrong, the controller and interface lists,
and the lines of the launch log that mention an error. **Start from the first bullet.** Exit status
is 1 on a failure and 2 if it could not run at all (no build yet, a mode other than `balance` or
`tank`, or a controller manager is already running: restart the container with `docker compose -f
docker/compose.yml restart ros2` and try again).

| Bullet | Usually |
| --- | --- |
| `controller 'X' is not active` plus `state interface 'Y' is not exported` | A name in `rocko_controllers.yaml` that the hardware layer does not export. Compare it with the `MockRocko` block in the `ros2_control` xacro |
| `node Joystick.py crashed` | A Python script that is not executable (`ls -l`), or a missing import. Re-run the launch by hand to read the full traceback (below) |
| `<no answer within 8 s ...>` in place of the controller list | The launch died before the controller manager came up; the launch log below it says why |

`verify_mock.sh` always launches with the mock on. If you launch by hand **without**
`use_mock_hardware:=true` in this container, expect `Could not create class of type
rocko_env::Motor12VoltQuadEncoder` followed by `Could not load and initialize hardware`: the
container build leaves that motor plugin out, so nothing can drive real motors here.

To read a full traceback, start a shell in the container and launch by hand:

```bash
docker compose -f docker/compose.yml exec ros2 bash
# then, inside the container:
source /opt/ros/jazzy/setup.bash && source /ros2_ws/install/setup.bash
ros2 launch rocko_env rocko.launch.py use_mock_hardware:=true
```

Use `rocko_tank_mode.launch.py` instead for tank mode. Press Ctrl+C to stop it. If a stray ROS
process is left over afterwards, restart the container to clear all of it:
`docker compose -f docker/compose.yml restart ros2`.

## 5. What to do next

A green run of steps 2–4 means the change builds and wires up. To say anything about whether the
robot *behaves*, a person has to test it on the robot, tethered: that is
[Reproducing results](reproducing-results.md) territory, and an AI change should come with its own
test procedure in the [run journal](../journal/README.md).

To stop the container: `docker compose -f docker/compose.yml down` (add `-v` to also delete the build
volume for a completely clean rebuild).
