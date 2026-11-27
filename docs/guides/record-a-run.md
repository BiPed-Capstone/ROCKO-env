# Record a run

Use this whenever you test something on the robot and might want to look at it again: a balance
attempt, a gain change, a drive. One launch argument gives you a recording of the robot's signals
and a small file that says what was run, so the recording can be compared with another one later.

**What has been checked:** recording was run against the simulated robot (`verify_tools.sh`, in
[the verification ladder](../architecture/verification-ladder.md)). **It has not been run on the
Pi.** What it costs the Pi's CPU and SD card while the 100 Hz balance loop runs is not known; see
[What is not known](#what-is-not-known).

## Record

On the **Pi**, from the repo root (`~/ROCKO-env`), add `record:=true` to the launch you were going to run
anyway. `pi/launch.sh` runs it as root with the environment set up ([Run ROCKO on the Pi](run-on-the-pi.md)):

```bash
pi/launch.sh balance record:=true record_label:=pitch_step battery_v:=12.1 surface:=tile
```

| Argument | What it is | If you leave it out |
| --- | --- | --- |
| `record:=true` | turn recording on | nothing is recorded |
| `record_label:=...` | a short name for the test; letters, digits, `-` and `_` | `run` |
| `battery_v:=...` | battery voltage at the start, in volts | the file says `not recorded` |
| `surface:=...` | what the robot stands on: `carpet`, `tile`, `stand` | the file says `not recorded` |
| `log_root:=...` | where the run folder is made | `~/rocko_logs`; `pi/launch.sh` fills in your own `~/rocko_logs`, because the launch runs as root and `~` would otherwise be `/root` |

The same arguments work with `pi/launch.sh tank` and, in the dev container,
`rocko_sim.launch.py`.

The run folder is **owned by root** (the launch ran as root). Copying it to a laptop works as usual;
deleting one on the Pi needs `sudo rm -r ~/rocko_logs/<run folder>`.

Success looks like this line in the launch output, a second or two after the start:

```text
[ros2-N] RecordRun: recording to /home/<user>/rocko_logs/20261116_142501_pitch_step
```

Stop the launch with Ctrl-C as usual (the launch waits up to 20 s for the recording to close). The
recorder closes the file and prints
`RecordRun: stopped (signal SIGINT). The run is in ...`. **Wait for that line** before switching the
robot off: a recording that was not closed has no `bag/metadata.yaml`, and some tools will not open it.

To record without restarting a launch that is already running, start the recorder by hand in a second
terminal on the Pi, opened with `pi/rocko_shell.sh` (so it is root and in the same `ROS_DOMAIN_ID` as the
launch); Ctrl-C stops it. It records into root's `~/rocko_logs` (`/root/rocko_logs`) unless you add
`--log-root /home/<you>/rocko_logs`:

```bash
ros2 run rocko_env RecordRun.py --label pitch_step --battery-v 12.1 --surface tile
```

Started by hand like this, `run.yaml` has no launch file, hardware mode or config hash (each says
`not recorded`). Add `--launch <launch file> --hardware real --config <path of the controllers YAML>`
to fill them in (`ros2 run rocko_env RecordRun.py --help` lists every option), or note them yourself.

## What you get

```text
~/rocko_logs/20261116_142501_pitch_step/
    run.yaml          what was run
    bag/
        bag_0.mcap    the recording
        metadata.yaml written when the recording was closed properly
```

`run.yaml` holds the label, the start and stop time, the host, the **commit** the code was at and
whether the tree had uncommitted changes, the launch file, `real` / `mock` / `sim`, the controller
config's path and its SHA-256 (so you can tell exactly which gains were live), the battery voltage,
the surface, and the list of topics. It is most of the record that
[reproducing-results.md](reproducing-results.md) asks for; add what happened, in words, yourself.

The recording holds these signals (the list is `RECORDED_TOPICS` in
`rocko_env/rocko_env/recording/RecordRun.py`, the one place to change it):

| Topic | What is in it |
| --- | --- |
| `/dynamic_joint_states` | pitch [deg], pitch rate [deg/s], wheel position and velocity, and the wheel velocity commands, every control cycle |
| `/pitch_pid_controller/controller_state`, `/velocity_pid_controller/controller_state` | each loop's reference, feedback, error and output |
| `/diffbot_base_controller/cmd_vel`, `/velocity_pid_controller/reference`, `/left_feedforward`, `/right_feedforward`, `/joy` | what the operator asked for and what was passed to the cascade |
| `/rocko/state`, `/balance_teleop/enabled`, `/tank_teleop/enabled` | the supervisor's mode and enabled flag, and whether that mode's teleop node is passing the sticks on |
| `/diff_drive_controller/command_timed_out`, `/tipover_rearm` | the teleop watchdog's state and tipover re-arm requests |
| `/rosout` | every node's log lines, including the tipover and time-out messages |
| `/controller_manager/statistics/full`, `/controller_manager/activity`, `/diagnostics` | how long each control cycle took, controller state changes, and the optional extras' health |
| `/rocko/battery_raw`, `/rocko/battery` | the battery voltage: every reading, unsmoothed, and the smoothed once-a-second value. Only with the battery monitor fitted; the reading rate is in [the robot extras](../architecture/robot-extras.md#battery) |
| `/joint_states`, `/tf`, `/tf_static`, `/robot_description`, `/diffbot_base_controller/odom` | what a viewer needs to draw the robot; tank mode's odometry |

A topic that does not exist in the mode you launched is not written.

## Copy the runs to your laptop and look at them

On the **laptop**, from the repo root:

```bash
tools/fetch_logs.sh <user>@<robot-ip>
```

It copies every run in the Pi's `~/rocko_logs` into `./rocko_logs/` with `rsync` over `ssh`, asks for
the Pi's password unless you have set up a key, and never deletes anything on the Pi. It needs `rsync`
on both machines (`sudo apt install rsync` on the Pi if it is missing), and it copies only
`~/rocko_logs`: for a run made with another `log_root:=`, use `rsync` by hand.

Open `bag_0.mcap` in [Foxglove](https://foxglove.dev) (File, Open local file) and plot, for example,
`/dynamic_joint_states` against `/pitch_pid_controller/controller_state`. On a machine with ROS,
`ros2 bag info <run folder>/bag` lists the topics and message counts.

Then free the Pi's disk: `ssh <user>@<robot-ip> 'rm -r ~/rocko_logs/<run folder>'`.

## Look at a brown-out

A **brown-out** is the battery voltage sagging for a moment while the motors pull hard, for example when the
balance loop swings the wheels to catch a fall. To see one next to the control effort that caused it, the battery
monitor must be fitted ([Fit the screen, buzzer and battery monitor](fit-the-extras.md)).

**After the run, in Foxglove (laptop).** Open the run's `bag_0.mcap` as above, then load the layout
`foxglove/rocko_battery_layout.json` from the repo (Foxglove: the layout menu, Import from file). Three plots share one
time axis: the battery voltage (every reading, and the smoothed value), the control effort (the pitch loop's wheel
velocity command and the two feedforward terms, rad/s), and the pitch. A brown-out is a dip in the top plot under a
large swing in the middle one. `foxglove/rocko_balance_layout.json` is the general balancing layout. **The layout files
have not been opened in Foxglove** (a check in the simulation confirms every signal they name exists; whether
Foxglove imports them as written is not known). If the import fails, make the plots by hand from the names in the
file: `/rocko/battery_raw.voltage`, `/pitch_pid_controller/controller_state.dof_states[0].output`, and so on.

The battery readings are plotted at the time each was read (`header.stamp`); the control effort, which carries no
time of its own, at the time the recorder received it, a few milliseconds late at most. That is close enough for a
sag lasting tens of milliseconds.

**Live, on the dashboard.** The *Battery [V], lowest reading in each 50 ms* plot sits next to the *Wheel command*
plot; a sag shorter than one dashboard sample still shows as a dip, and a moment with no reading shows as a gap.

The control effort here is the wheel velocity command, not the PWM duty sent to the motors: the motor plugin turns
the command into duty and publishes nothing about it.

## What commonly goes wrong

| You see | Why | Do |
| --- | --- | --- |
| `RecordRun: NOT recording: ... MB free` | Less than 500 MB free on the disk holding `log_root`. A full SD card stops the whole robot, so the recorder refuses | Copy old runs off and delete them |
| `RecordRun: stopped (only ... MB free)` in the middle of a run | The same limit was reached while recording | The same; the part recorded so far is closed and usable |
| No `bag/metadata.yaml` | The power was cut, or the process was killed, before the recording closed | `ros2 bag reindex <run folder>/bag` on a machine with ROS is the tool meant for this (not tried here) |
| `run.yaml` says `commit: "unknown"` | The code is not running from a git checkout (it was copied, not cloned) | Note the commit by hand |
| `run.yaml` says `tree: "dirty"` | There were uncommitted changes, so the commit alone does not say what ran | Commit, or note what differed |
| A topic you wanted has no messages | It does not exist in that mode, or nothing published on it (the simulation has no `/joy` and no supervisor) | `ros2 topic list` while the robot runs |

## What is not known

- **The cost on the Pi.** Recording adds a process that subscribes to several 100 Hz topics and
  writes to the SD card. On the robot the control loop already runs late at times (finding H-5), and
  nobody has measured whether recording makes that worse. Compare the spread of loop periods with
  and without `record:=true` before trusting a recorded balance run to be representative. The cycle
  times are themselves in the recording (`/controller_manager/statistics/full`).
- **The size on the Pi.** In the simulation, on a laptop, with nobody driving, one run measured about
  42 MB per minute (one measurement, 2026-11-16). At that rate 500 MB is about 12 minutes.
- The recorder stores what the middleware delivers. Under load it can miss messages; a gap in a plot
  is not proof the robot's loop stopped.
