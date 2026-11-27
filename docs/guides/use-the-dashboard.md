# Watch the robot in a browser: the dashboard

Use this to see what the robot is doing from another machine (the supervisor's state, pitch, wheel speeds, what
the controllers are asking for, warnings), to **enable, disable and switch mode** (tank or balance), to start and
stop a recording, and to see and mute the [robot extras](../architecture/robot-extras.md) (status screen, buzzer,
battery monitor), without a terminal on the Pi.

**What has been checked:** the dashboard was run against the simulated robot (`verify_tools.sh`, in
[the verification ladder](../architecture/verification-ladder.md)), and the page was looked at once in
a browser with made-up data. The supervisor the buttons ask was checked against the mock
(`verify_supervisor.sh`; the buttons themselves were not clicked in a browser, as far as the
[journal](../journal/2026-11-23-robot-supervisor.md) records), not on the Pi. **The dashboard has not been run on the Pi since the supervisor change
(2026-11-23).** What it costs the Pi's CPU while the 100 Hz balance loop runs is not known; that is why
`rocko.launch.py` run by hand leaves it **off by default**, whereas `rocko_robot.launch.py` (the boot launch) starts it.
Read [What is not known](#what-is-not-known) before using it during a balance test.

## Start it

**Normally you do not start it: the robot's boot service does.** `rocko.service` runs `pi/launch.sh robot`, which starts
the supervisor and this dashboard together ([Run ROCKO on the Pi](run-on-the-pi.md#the-boot-service-the-supervisor)).
By hand, on the **Pi**, from the repo root (`~/ROCKO-env`), stop the boot service first if it runs
(`sudo systemctl stop rocko.service`), then:

```bash
pi/launch.sh robot
```

`pi/launch.sh robot dashboard:=false` leaves the dashboard out. The older single-mode launches still take it as an
option: `pi/launch.sh tank` starts it (`dashboard:=false` leaves it out) and `pi/launch.sh balance dashboard:=true`
adds it. **Those launches have no supervisor**, so the state card says NO SUPERVISOR and its buttons cannot work.

Success looks like this line in the launch output:

```text
[Dashboard.py-N] [INFO] [...] [rocko_dashboard]: dashboard: open http://<this machine's address>:8080 in a browser (read-only apart from Record/Stop, the supervisor's Enable/Disable/mode buttons and the buzzer's Mute; no login)
```

Then, on **any machine on the same network**, open `http://<robot-ip>:8080` in a browser (`hostname -I`
on the Pi prints its address). Nothing needs installing, and the page needs no internet.

`dashboard_port:=9000` moves it to another port. In the dev container, `rocko_sim.launch.py` starts the dashboard by
default; to open it from the laptop's browser the container's port 8080 would have to be published in
`docker/compose.yml`, which it is not.

### It starts at boot with the robot

The boot service `rocko.service` starts the dashboard along with the supervisor; there is no separate dashboard
service any more. How to install, check and remove it:
[Run ROCKO on the Pi](run-on-the-pi.md#the-boot-service-the-supervisor).

How it behaves under the supervisor:

- It runs as root in the robot's `ROS_DOMAIN_ID` (17), inside the same launch as the supervisor, so it is there in
  every mode and survives a mode change.
- A recording started from its page records what the supervisor was told (`launch: ...` in `run.yaml`); use
  `record:=true` on the launch when the exact launch arguments matter. Recordings go to your `~/rocko_logs`, as with
  `pi/launch.sh`.
- **It is always on, and it also runs while the robot balances.** What it costs the Pi's CPU beside the 100 Hz balance
  loop has never been measured. Until it is, a balancing test runs with the page open on the boot launch's own
  dashboard; start the boot launch with `dashboard:=false` (`pi/launch.sh robot dashboard:=false` after stopping the
  service) if you want it off.
- **It is always on, with no login, and it can now enable the robot** (see below). Remove the service on a network you do
  not trust.

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
| State card (top of the page) | **the whole card is green while ENABLED, red while DISABLED, grey when unknown**, with the Enable button green and Disable red, so it reads at a glance. What the supervisor says: **mode** (tank or balance), **ENABLED** or **DISABLED**, **phase** (`starting`, `ready`, `stopping`, `fault`, and `-> balance` while a change is under way), the **message** (why the last request was refused, or what the fault is) and the **last source** of a change (`gamepad`, `dashboard`, or `supervisor` for an automatic disable). It is the only place that says whether the wheels may move. `NO SUPERVISOR` and `UNKNOWN` when no state has arrived for 2 s | `/rocko/state` ([the supervisor](../architecture/robot-supervisor.md)) |
| Battery, Buzzer, Screen on the robot (the three cards under the state card) | what each optional extra reports: **OK**, **WARN**, **ERROR** with its reason, or **NOT RUNNING** (no report for 5 s: never started, or crashed). The battery card shows the smoothed voltage, a rough percentage and LOW or CRITICAL; the buzzer card says muted or sound on and has the **Mute / Unmute** button (`/rocko/set_buzzer_muted`; the screen on the robot has the same button). None of them can stop the robot | `/diagnostics` ([the robot extras](../architecture/robot-extras.md)) |
| Nodes and controllers table | each node and controller the current mode expects, and whether it is up (`alive` or `MISSING`; a controller's actual state against its expected one). A wrong row is highlighted; during a mode change most rows are wrong for a few seconds, which is what the table is for | `/rocko/state` |
| Controller card | the USB game controller, drawn live: both sticks (the dot is the stick's position; up and left read +1), the triggers, every button, with **LB** and **RB** (the enable combo) marked, and next to it what the teleop node is doing. `driving` only while the robot is enabled **and** the sticks have been centred since; otherwise it says why (disabled, or sticks must centre); `No controller` when nothing arrives on `/joy`; `Teleop not running` when the current mode's teleop node is silent. The drawing turns grey when `/joy` is stale | `/joy`, `/tank_teleop/enabled` ([Drive ROCKO with a USB controller](drive-with-usb-controller.md)) |
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

It can **ask the supervisor** to enable, disable or change mode, start and stop a recording, and hand recorded runs to
the browser. It still cannot change a gain, re-arm the tipover cutoff by itself, or send a drive command: the sticks
are the controller's. Each button only *asks*; the supervisor decides and the card shows the answer (a refusal says why).

| Button | Clickable when | What it does |
| --- | --- | --- |
| **Enable** | the phase is `ready` and the robot is disabled | Activates the motion controllers. In balance mode only near upright (a refusal says so). After this the sticks must return to centre before the robot moves |
| **Disable** | always | Deactivates them. Never needs anything to go right first. In balance mode **the robot falls** |
| **Tank** / **Balance** | the robot is disabled, the phase is `ready` or `fault`, and it is not already in that mode | Stops the current stack and starts the other: several seconds, with no motor stack between |

Greyed buttons are the ones the supervisor would refuse. Record and Stop are as before.

**There is no login, and that now matters more.** Anyone on the robot's network can open the page, press Record,
download runs **and press Enable**. Until 2026-11-23 the page could not move the robot; now one click on a phone can
let the wheels move (the sticks must still be centred and then moved, but nobody needs to be at the robot). Use the dashboard only on the lab's own network, and press **Disable** (or LB + RB) before you pick
the robot up.

**Closing the page does not disable the robot.** The dashboard is not a deadman: if you enable from it and the browser
or the Wi-Fi goes away, the robot stays enabled. Disable with LB + RB on the controller, or reload the page.

## What commonly goes wrong

| You see | Why | Do |
| --- | --- | --- |
| The browser cannot connect | The launch was started with `dashboard:=false`, or it is a hand launch of `rocko.launch.py` (off by default; `rocko_robot.launch.py` and tank mode start it); the address is wrong; the machines are on different networks; a firewall blocks port 8080 | Check the launch output for the `dashboard: open ...` line; `ping <robot-ip>` |
| `OSError: [Errno 98] Address already in use` in the launch output | Another program, or a dashboard left from an earlier launch, has the port | `dashboard_port:=8081`, or stop the other program |
| Every value is a grey dash, amber bar on top | The controllers have not started, or have stopped | `ros2 control list_controllers` in a `pi/rocko_shell.sh` terminal on the Pi |
| Pitch rate is a dash | The IMU node does not read the gyro by default | Expected; see next-steps row 1.2 |
| State card says NO SUPERVISOR, Enable is refused or greyed | The launch was started by hand (`pi/launch.sh tank` or `balance`), the supervisor died, or the phase is not `ready` | Use `pi/launch.sh robot`; read the card's message and the nodes table |
| "could not start: a recording is already running" | One is running, perhaps started from another browser | Press Stop first |
| Record does nothing and the launch output says `NOT recording: ... MB free` | The disk is nearly full | [Record a run](record-a-run.md), "What commonly goes wrong" |

## What is not known

- **The cost on the Pi with a browser connected.** Without one, measured on 2026-11-24 with the balance stack
  running but disabled, the dashboard did not change the control loop's timing measurably; its CPU share, from one
  whole-system `top` sample, is in the same place ([loop timing](../journal/2026-11-24-loop-timing-on-the-pi.md)). With a page open it
  also answers about six requests a second; that, and the cost with the balance controllers active, are not measured.
  `tools/loop_timing.py` measures it: run it on the Pi with and without the page open.
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

`Dashboard.py` is a ROS node: it subscribes to the topics in the table above (including the supervisor's `/rocko/state`) and keeps the newest
values and the last 60 s of them, sampled 20 times a second. In the same process, Python's built-in
web server answers the page itself, the newest values, the history since a given time, the list of recorded runs, and
four kinds of request that do something. The page is one HTML file with its own small plotting code,
no outside scripts; it asks for the newest values every quarter of a second and for new history every
half second. Record and Stop make the node start and stop `RecordRun.py`, the program `record:=true` runs; Enable,
Disable and the mode buttons make it call the supervisor's `/rocko/set_enabled` and `/rocko/set_mode` services. The buzzer card's Mute
button calls the buzzer's `/rocko/set_buzzer_muted`. The node creates no publisher, and its only service clients are
those three, which is what "cannot drive the robot" rests on: the mute switch can only silence a buzzer.
