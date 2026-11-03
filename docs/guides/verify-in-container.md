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

### Check the container is looking at *your* files

Three words first. A **checkout** is a folder on your laptop holding a copy of this repo's files; most
people have one. A second `git clone`, or a **git worktree** (an extra working folder for the same
repository, made with `git worktree add <folder> <branch>`; some AI tools make one per session), is
another checkout. The container **mounts** a folder: it shares that laptop folder into the container,
here at `/ros2_ws/src/ROCKO-env`.

There is one container per laptop, and it mounts the checkout whose `docker/compose.yml` last ran
`up -d`. So `up -d` is itself the switch: run from your checkout it points the container at you, and
it takes the container away from anything running in another checkout. If the container is already
up, you may skip `up -d`, and then it is still showing whichever checkout it had. In the wrong
checkout, steps 2 to 5 still run and the container commands can still pass, but they test the other
folder's files.

```bash
./check_container_mount.sh
```

Run it on the laptop, from the repo root, at the start of a session and whenever you change checkout.
Success: `ok    the container sees this checkout: <your folder>`, exit 0. It creates an empty marker
file in your checkout, asks the container whether it can see that file, and deletes it.

What `ok` does **not** say: that the build output inside the container came from your checkout. The
build folder survives a switch, and `make` (the tool CMake uses to compile) only recompiles a file
whose source is newer than its output. A checkout whose files are older than the last build is
therefore not recompiled. This was tested: appending a comment to a controller, building, then
restoring the original file with its old timestamp and building again recompiled nothing. After a
switch, rebuild from clean (the second command below).

If it prints `FAIL` (exit 1), it prints the commands to run. Add the same extra `-f` file you
started the container with (`compose.mac.yml` or `compose.windows.yml`), if any:

```bash
docker compose -f docker/compose.yml up -d --force-recreate
docker compose -f docker/compose.yml exec -T ros2 bash -c 'cd /ros2_ws && rm -rf build install log && src/ROCKO-env/build_ros.sh'
```

- `--force-recreate` is there because plain `up -d` is not always enough. If you deleted a checkout
  and cloned it again at the same path, compose sees nothing change, prints `Running`, and the
  container keeps showing the deleted folder. This was tested.
- A new container has none of the ROS packages the old one installed, and `build_ros.sh` installs them
  again, so the second command needs the network and takes minutes. The build volume itself is kept,
  which is why it has to be emptied by hand.
- Recreating stops anything running in the container, for another session too, and leaves that
  checkout failing this check until it does the same. Run the check again whenever you switch.

Exit 2 means it could not tell: no `docker` command (you are in the container, or on the Pi, rather
than on the laptop), Docker is not running, or the container is not up. Docker's own message is
printed above ours. Only Docker Desktop for macOS has been tried.

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

If you changed `Motor12VoltQuadEncoder.cpp`, also type-check it (the build above leaves it out):

```bash
docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/check_motor_plugin_compiles.sh
```

Success: exit 0. It checks syntax and types only, against hand-written stand-ins for the WiringPi
headers, so it does not show the plugin links, runs, or builds on a Pi. It needs the build above
first (exit 2 and a message if there is none). A compile error prints `FAIL (compile-only)` and exits
1. [The ladder](../architecture/verification-ladder.md) says what it proves.

If you changed `ImuFaultTracker.hpp`, also run its host test. It needs only a C++17 compiler (the laptop's
or the container's), run from the repo root:

```bash
g++ -std=c++17 -Wall -Wextra -I rocko_env/rocko_env/hardware/sensors/icm20948/include \
    tools/imu_fault_tracker_test.cpp -o /tmp/imu_fault_tracker_test && /tmp/imu_fault_tracker_test
```

Success: it prints `all ImuFaultTracker tests passed` and exits 0. It is not a tier and nothing runs it
automatically. It tests the counting logic of that one header, nothing on the sensor or the robot.

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

What happens: for balancing and tank mode, each with the mock off and on, and for balancing mode with
the simulated robot (`use_sim_plant:=true`, without and with `use_mock_hardware:=true`), and for tank
mode asked for the simulator (which tank mode ignores by design and must still come out as the plain
real tank robot) — seven expansions in all — it expands the xacro (a
*xacro* file is XML with macros), checks the result is a valid robot description, and then checks the
right hardware is in it: the two real motor plugins (and the IMU, in balancing mode) when the mock is
off, exactly one `MockRocko` system when it is on, exactly one `SimPlant` system (and nothing else)
for the sim. Finally it checks the mock, and the sim, export **exactly the same interfaces** (joints,
sensors and gpio, duplicates included) as the real hardware, so neither can quietly drift from the
robot. A last step runs `sim/check_sim_config.py`, which fails if `rocko_sim_controllers.yaml` differs
from `rocko_controllers.yaml` in anything but the gain numbers, or if a simulator setting's default is
not the same in `SimPlantModel.hpp`, `rocko.urdf.xacro` and `rocko_sim.launch.py` (it also checks that
every `<param>` is wired to the xacro arg of its own name). Exit 1 from it is drift; exit 2 means it
could not run (for example PyYAML is missing), and `verify_urdf.sh` says which. The
content checks matter because a valid-looking description passes the first two checks whichever
hardware was selected. It needs step 2 first, because the xacro files find each other through the
installed package.

Success:

```
ok    rocko mock=false
ok    rocko mock=true
ok    rocko: the mock exports the same 14 interfaces as the real hardware
ok    rocko_tank_mode mock=false
ok    rocko_tank_mode mock=true
ok    rocko_tank_mode: the mock exports the same 6 interfaces as the real hardware
ok    rocko sim=true mock=false
ok    rocko sim=true mock=false: the sim exports the same 14 interfaces as the real hardware
ok    rocko sim=true mock=true
ok    rocko sim=true mock=true: the sim exports the same 14 interfaces as the real hardware
ok    rocko_tank_mode sim=true (ignored by design)
      ok    17 wiring values equal, with their types, plus 1 allowed difference(s) (INTENTIONAL_WIRING_DIFFERENCES; gains maps: next check)
      ok    each controller's `gains` keys are its dof_names in both files; every gains key except the tunable numbers is equal
      ok    29 simulator settings: header, xacro and ros2_control params agree; 10 launch-file defaults match the xacro
      ok    IMU fault limit equal in ICM20948.hpp and SimPlantModel.hpp
ok    sim config: no drift
PASS: all seven expansions (four real/mock, two sim, one tank-with-sim-request) expand, parse, hold the right hardware, the mock and the sim match the real interfaces, tank mode ignores the sim request, and the sim config has not drifted.
```

That is twelve `ok` lines at the left margin (four more, indented, come from the config check) and `PASS`.

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

If you changed the git hooks in `.githooks/`, run `.githooks/test-git-hooks.sh` as well (also on the
laptop, also no container). Success: `PASS: every case behaved.` and exit status 0. It builds a
throwaway repository and remote in a temporary folder and never touches this one.

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

## 5. Run the simulated robot (Tier 3, partly built)

```bash
docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_sim.sh
```

What happens: it launches the same controllers against `rocko_env/SimPlant`, a physics **model** of the
robot, once per check (`rocko_sim.launch.py`), one launch after another. With no arguments it runs six
checks in 8 launches and took about 119 s in the arm64 container on 2026-11-02 (one measurement, taken
before `H4`'s recording window grew from 8 s to 16 s, so expect a little longer; each wait in the script is
bounded):

| Check | What it shows, in the model |
| --- | --- |
| `S1` (2 launches) | The four cascade controllers and `joint_state_broadcaster` go active with all 14 interfaces; the simulated pitch does not move when nothing pushes it (baseline) and does move when the wheels get a command |
| `S2a` | With the motors producing no torque the robot passes 35 deg within 0.48 s |
| `H4` | The tipover cutoff (finding H-4): started 5 deg past the limit, both wheel controllers trip in their first cycle, every command written past the limit is exactly 0.0, one `/tipover_rearm` message sent while still past the limit is refused by **both** wheel controllers, and the controllers stay active |
| `C3` | The teleop watchdog (finding C-3): when a fake gamepad goes silent, the velocity reference becomes exactly `[0, 0]` between 0.2 s and 0.3 s later and the balance loop keeps running; the log must show `teleop commands resumed` before the first `teleop command timed out` |
| `C2` (2 launches) | An IMU outage (finding C-2): a 0.03 s one causes no `ERROR`; a 0.2 s one makes `read()` return `ERROR` and `controller_manager` deactivate the hardware; the pitch is never exported as 0.0 or NaN |
| `H6` | The pitch PID's D term reads the exported gyro `pitch_rate` (finding H-6) |

To run only some, name them, in any order (the names are `S1 S2a H4 C3 C2 H6`):

```bash
docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_sim.sh H4 C3
```

An unknown name exits 2 and prints the known ones. Use this while working on one safeguard; run the
whole script before you report.

It needs step 2 first (the build).

Success: with no arguments it ends with `PASS (simulation: S1 S2a H4 C3 C2 H6)` and exit status 0,
followed by one line per check (with only some checks, the names that ran are listed instead). A failure
prints `FAIL` with the run that failed and why, and exits 1; exit 2 means it could not run or could not
judge (`COULD NOT JUDGE`: a probe saw no messages or no movement; read its reason and the kept run
directory) — e.g. no build yet, or a controller manager is already running: restart the container as in
step 4). Every S1 and S2a run must also show that the robot was *released* (SimPlant logs it) at least
3 s before the probe stopped, so a robot that never moved cannot pass the baseline. The run in
`H4`, `C3`, `C2` and `H6` also prints its own evidence (the log lines it found and the counts it
checked). In the `S1 command` and `S2a` runs the robot falls past 35 deg, so you will also see
`note: the tipover cutoff tripped in this run`; that is expected. In the long `C2` run the script prints
what `controller_manager` did after the `ERROR` (it deactivated the hardware and all five controllers);
that is the finding, not a failure. On anything but a pass the script keeps its run directory (launch
logs, `samples.csv` and `metrics.json` per run) and prints its path inside the container; on a pass it
deletes it.

**What a pass does not tell you.** Every physical number in the model (masses, inertia, motor
constants, the IMU delay) is an estimate; nothing was measured on the robot. No scenario that checks
the controllers *hold* the model upright exists yet (the safeguard checks above run for a few seconds
and look at the safeguard, not at balance), so a pass does not mean the cascade balances,
and it certainly does not mean the real robot balances or that any gain is good. The
[verification ladder](../architecture/verification-ladder.md) says what is and is not built, and
[the model's specification](../architecture/sim-plant.md) says what it leaves out.

To run the plant's own unit test (no ROS, any machine with a C++17 compiler; from the repo root):

```bash
g++ -std=c++17 -Wall -Wextra -I rocko_env/rocko_env/hardware/sim/include \
    -I rocko_env/rocko_env/hardware/sensors/icm20948/include \
    tools/sim_plant_model_test.cpp -o /tmp/sim_plant_model_test && /tmp/sim_plant_model_test
```

Success: it prints `all SimPlantModel tests passed` and exits 0. It is not a tier and nothing runs it
automatically.

The stage 4 safeguards have four more host tests of the same kind (no ROS, no container needed; from
the repo root; not tiers, and nothing runs them automatically):

```bash
g++ -std=c++17 -Wall -Wextra -I rocko_env/rocko_env/controllers/add_feedforward/include tools/tipover_latch_test.cpp -o /tmp/tipover_latch_test && /tmp/tipover_latch_test
python3 -I tools/teleop_watchdog_test.py
python3 -I tools/computer_joystick_heartbeat_test.py
python3 -I tools/imu_node_test.py
```

Success: `all TipoverLatch tests passed` (exit 0); the three Python tests print `PASS <name>` per check and
then `16/16 passed`, `5/5 passed` and `4/4 passed`, exit 0 (2026-11-02). What they do and do not prove is in the
[verification ladder](../architecture/verification-ladder.md#the-host-tests-of-the-stage-4-safeguards-not-tiers).

## 6. What to do next

A green run of steps 2–5 means the change builds, wires up, and passes the few checks of an estimated model that exist. To say anything about whether the
robot *behaves*, a person has to test it on the robot, tethered: that is
[Reproducing results](reproducing-results.md) territory, and an AI change should come with its own
test procedure in the [run journal](../journal/README.md).

To stop the container: `docker compose -f docker/compose.yml down` (add `-v` to also delete the build
volume for a completely clean rebuild).
