# Watch the robot in a browser: the dashboard

Use this to see what the robot is doing from another machine (pitch, wheel speeds, what the
controllers are asking for, warnings) and to start and stop a recording without a terminal on the Pi.

**What has been checked:** the dashboard was run against the simulated robot (`verify_tools.sh`, in
[the verification ladder](../architecture/verification-ladder.md)), and the page was looked at once in
a browser with made-up data. **It has not been run on the Pi.** What it costs the Pi's CPU while the
100 Hz balance loop runs is not known; that is why it is **off by default on the robot**. Read
[What is not known](#what-is-not-known) before using it during a balance test.

## Start it

On the **Pi**, from the repo root (`~/ROCKO-env`), add `dashboard:=true` to the launch. `pi/launch.sh`
runs it as root with the environment set up ([Run ROCKO on the Pi](run-on-the-pi.md)):

```bash
pi/launch.sh balance dashboard:=true
```

Success looks like this line in the launch output:

```text
[Dashboard.py-N] [INFO] [...] [rocko_dashboard]: dashboard: open http://<this machine's address>:8080 in a browser (read-only apart from Record/Stop; no login)
```

Then, on **any machine on the same network**, open `http://<robot-ip>:8080` in a browser (`hostname -I`
on the Pi prints its address). Nothing needs installing, and the page needs no internet.

`dashboard_port:=9000` moves it to another port. `rocko_tank_mode.launch.py` takes the same
arguments. In the dev container, `rocko_sim.launch.py` starts the dashboard by default; to open it
from the laptop's browser the container's port 8080 would have to be published in
`docker/compose.yml`, which it is not.

## What the page shows

| On the page | What it is | From |
| --- | --- | --- |
| Pitch [deg] | the lean the IMU reports, with whatever sign and offset it reports | `/dynamic_joint_states` |
| Target pitch [deg] | the lean the velocity loop is asking the pitch loop for | `/pitch_pid_controller/controller_state` (its reference) |
| Pitch rate [deg/s] | the gyro's rate, if the IMU node reads it; a dash if not | `/dynamic_joint_states` |
| Left wheel, Right wheel [m/s] | measured wheel velocity as the encoder node reports it | `/dynamic_joint_states` |
| Velocity reference [m/s] | what the operator's command became after the ramp | `/velocity_pid_controller/reference` |
| Pitch loop output [rad/s] | the wheel velocity the pitch loop commands, before the feedforward is added | `/pitch_pid_controller/controller_state` (its output) |
| Teleop link | `driving` while commands arrive; `no commands` when the teleop watchdog has zeroed the speed | `/diff_drive_controller/command_timed_out` |
| Tipover cutoff | `TRIPPED` while either wheel's controller has logged a trip and not a re-arm; `re-armed` after both have logged a re-arm; `no trip seen` if neither line has arrived since the dashboard started. **`no trip seen` is not "armed"**: a trip logged before the dashboard started cannot be known to it | `/rosout` |
| Three plots | the last 30 s of pitch and target pitch, the wheel speeds and the reference, and the pitch loop's output | the same topics, 20 points a second |
| Warnings and errors | the newest 40 log lines at WARN or above, from every node | `/rosout` |

A value turns grey when nothing has arrived on its topic for a second. An amber bar across the top
means no robot state (`/dynamic_joint_states`) is arriving at all; a red one means the tipover cutoff
tripped, or the dashboard itself stopped answering.

The values are what the controllers see, not the truth: the pitch includes the IMU node's offset, and
the wheel velocity is the encoder node's filtered figure ([sim-plant.md](../architecture/sim-plant.md), U-2).

## Record from the page

Fill in a label, the battery voltage and the surface, and press **Record**. Press **Stop** when the
test is over, and wait for "stopped". The run appears in the list with a **download** link (once it is
closed), which gives you a `.tar` of the run folder (`run.yaml` and the bag). It is the same recording, with the same
topics and the same `run.yaml`, as `record:=true` makes: see [Record a run](record-a-run.md). One
recording at a time. A run marked "open or not closed" is still being written, or was cut off.

## What the page can and cannot do

It can start and stop a recording and hand recorded runs to the browser. That is all. It publishes on
no topic and calls no service, so it cannot drive the robot, change a gain or re-arm the tipover cutoff.

**There is no login.** Anyone on the robot's network can open the page, press Record and download
runs. Use it on the lab's own network, not on a network you do not trust.

## What commonly goes wrong

| You see | Why | Do |
| --- | --- | --- |
| The browser cannot connect | The launch was started without `dashboard:=true`; the address is wrong; the machines are on different networks; a firewall blocks port 8080 | Check the launch output for the `dashboard: open ...` line; `ping <robot-ip>` |
| `OSError: [Errno 98] Address already in use` in the launch output | Another program, or a dashboard left from an earlier launch, has the port | `dashboard_port:=8081`, or stop the other program |
| Every value is a grey dash, amber bar on top | The controllers have not started, or have stopped | `ros2 control list_controllers` in a `pi/rocko_shell.sh` terminal on the Pi |
| Pitch rate is a dash | The IMU node does not read the gyro by default | Expected; see next-steps row 1.2 |
| "could not start: a recording is already running" | One is running, perhaps started from another browser | Press Stop first |
| Record does nothing and the launch output says `NOT recording: ... MB free` | The disk is nearly full | [Record a run](record-a-run.md), "What commonly goes wrong" |

## What is not known

- **The cost on the Pi.** The dashboard is a Python process that receives four 100 Hz topics. It keeps
  the newest message of each and decodes it 20 times a second, and it lowers its own priority, so the
  control loop is served first. In the dev container on a laptop it used about a fifth of one CPU core
  (one measurement, averaged over its first 11 s, start-up included). The Pi is slower. Before using
  it during a balance test, compare the spread of control-cycle times with and without
  `dashboard:=true` (`/controller_manager/statistics/full`); on the robot the loop already runs late at
  times (finding H-5).
- Whether the page stays responsive over the lab's Wi-Fi. It asks the robot for data about six times a
  second; each answer is small.
- How it looks on a phone. It was looked at in one desktop browser.

## Add a signal to the page

1. In `Dashboard.py`, return it from one of the `*_values` functions under a new name that ends in its
   unit, for example `"roll_deg"` (a topic the node does not subscribe to yet also needs a row in
   `fast_topics`).
2. In `dashboard.html`, add a row with that name to `TILES`, and to one plot's list in `PLOTS` if it
   should be drawn.
3. Rebuild and run `verify_tools.sh`.

The name is the only link between the two files, so the spelling must match exactly.

## How it works, in five sentences

`Dashboard.py` is a ROS node: it subscribes to the topics in the table above and keeps the newest
values and the last 60 s of them, sampled 20 times a second. In the same process, Python's built-in
web server answers four kinds of request: the page itself, the newest values, the history since a
given time, and the list of recorded runs. The page is one HTML file with its own small plotting code,
no outside scripts; it asks for the newest values every quarter of a second and for new history every
half second. Record and Stop are two more requests, which make the node start and stop `RecordRun.py`,
the program `record:=true` runs. The node creates no publisher and no service client, which is what
"cannot drive the robot" rests on, and `verify_tools.sh` checks it.
