# The robot supervisor: modes, enable, and who starts what

**Status:** written 2026-11-23 as the design for the change in
[the journal](../journal/2026-11-23-robot-supervisor.md). What has been checked, and on which tier, is in that entry
and the [verification ladder](verification-ladder.md); this page says how it works and why. It runs on the robot
since 2026-11-24 (what was seen: [What is not true yet](#what-is-not-true-yet)). The decision and the alternatives are in [ADR 0006](../decisions/0006-robot-supervisor.md).

## The idea in one paragraph

One ROS node, the **supervisor** (`Supervisor.py`), starts when the Pi boots and stays up. It owns the robot's
**state** and is the only thing that starts and stops the motor stack. A person changes the state from the game
controller or the web dashboard. Both of those only *ask* the supervisor; the supervisor decides, then
publishes the state on `/rocko/state` for everyone to see. The dashboard always shows that state and which nodes and
controllers are running.

## The state

The state has independent parts (in robotics, *orthogonal* dimensions). Each one can change without the others:

| Part | Values today | What it means |
| --- | --- | --- |
| `mode` | `tank`, `balance` | Which motor stack runs. Tank: plain differential drive, balancing off. Balance: the cascade in [control-cascade.md](control-cascade.md) |
| `enabled` | `false` (disabled), `true` | Whether the motion controllers are **active**. Disabled: they are loaded but inactive, so nothing commands the wheels |
| `phase` | `starting`, `ready`, `stopping`, `fault` | Where the mode's stack is in its life. Only `ready` can be enabled |

Plus, for people reading the dashboard: `last_source` (who caused the last change: `gamepad`, `dashboard`,
`supervisor` for an automatic disable), `message` (the reason for the last refusal or fault), and the list of
expected **nodes** and **controllers** with whether each is up.

**The rules** (all in `rocko_env/supervisor/state_machine.py`, plain Python with no ROS, so it is tested on a laptop):

1. **Boot:** mode `tank`, disabled. The supervisor starts tank's stack straight away.
2. **Enable** only when `phase` is `ready` and disabled. In balance mode, also only when the robot is near
   upright: `|pitch| <= rearm_max_pitch_deg`, the tipover cutoff's own re-arm band, read from the controller so the
   number has one home (`add_feedforward_controller_parameters.yaml`); and only when the robot is nearly still:
   `|pitch_rate|` known and within `ENABLE_MAX_PITCH_RATE_DEG_S` (`state_machine.py`, PROPOSED), because once the
   pitch loop takes its D term from the gyro, the rate at enable stays in it as a constant offset (finding L-8,
   [proposal 0016](../proposals/0016-pitch-d-term-from-the-gyro.md)).
3. **Change mode** only when **disabled** and `phase` is `ready` or `fault`. A change stops the old stack, then
   starts the new one: `stopping` → `starting` → `ready`.
4. **Disable** is always allowed, in any phase. Disabling never needs anything to go right first.
5. **Automatic disable** (`last_source = supervisor`), which depends on what disabling does in the mode
   (`falls_when_disabled` in `modes.py`):
   - **Tank** (disabling just stops the wheels): the game controller goes silent for `JOY_TIMEOUT_S` (0.25 s,
     `teleop/gamepad.py`) after it had been heard while enabled, or any expected node or controller of the stack
     goes away (also `phase = fault`). A toggle has no "let go to stop", so losing the controller must stop the robot.
   - **Balance** (disabling means the robot **falls**): controller silence does **not** disable. The safe response to
     lost input on an inverted pendulum is a zero velocity command (finding C-3), which `BalanceTeleop.py` and
     `DiffDriveController.py`'s watchdog already give, so the robot holds still, upright. A lost helper node
     (teleop, `diff_drive_controller`, a sensor node) gives `fault` but **stays enabled**, and the message says so: a
     person holds the robot, then disables. Only a lost cascade controller disables, because then nothing is
     holding it up anyway. The tipover cutoff is unchanged and still zeroes the motors past its angle.
   - **An unanswered `controller_manager`** is **unknown**, not "controllers missing". After 3 s of no answer it is a
     `fault`. In tank mode the supervisor also disables, and if it is still unanswered 5 s later it kills the stack. In
     balance mode it stays enabled (a silent `controller_manager` is not a reason to drop the robot) and is never killed for
     it; a *disabled* balance stack is killed like tank's, since nothing is balancing.
   - **Every disable is confirmed.** After asking for the controllers to be deactivated, the supervisor checks through
     `list_controllers` that they are inactive, and retries. If that is not confirmed within 6 s (longer than `controller_manager`'s own 5 s switch limit) it kills the whole
     stack, which stops the wheels by taking the motor plugin down.
6. A refused request changes nothing and says why in `message`.

## How a person asks

| Action | Game controller (joy's `/joy`) | Dashboard |
| --- | --- | --- |
| Enable / disable (toggle) | press **LB + RB** together (triggers not pressed) | **Enable** / **Disable** buttons |
| Switch tank ⇄ balance (toggle) | press **LT + RT + RB** together (LB not pressed). Only while disabled | **Tank** / **Balance** buttons. Only while disabled |

A combo fires **once, the moment its last button goes down** (the rising edge). It cannot fire again until one of its
buttons has been released. A combo that is already held when the controller is first heard (start-up, or a
reconnect) does not fire: it must be released first. That is the same reason tank mode's old deadman had a re-arm
rule: the joy node keeps its last button state across an unplug. The combo logic is `ComboDetector` in
`teleop/gamepad.py`, with a test.

**What replaced the hold-LB deadman (owner's decision, 2026-11-23).** Tank mode used to drive only while LB was held.
Now **enable is a toggle**, the same in both modes. A toggle is weaker than a held button: letting go no longer
stops the robot. What is left to stop it: LB+RB or the dashboard's Disable; in tank mode, the supervisor's automatic disable when
the controller goes silent (in balance mode a silent controller gives zero velocity instead, rule 5); each teleop node's own zero after 0.25 s of silence; the velocity-command timeout
in each stack; and in balance mode the tipover cutoff. The dashboard is **not** a deadman: closing the page
does not disable the robot. [ADR 0006](../decisions/0006-robot-supervisor.md) records the decision.

**Stick re-centre after enable.** After every disabled → enabled change, the teleop nodes send zero until both
sticks have been seen inside the dead zone. A stick held over when you press enable does not make the robot lurch.

## How enable works

Enable and disable do **not** restart anything. Each mode's stack starts its *motion controllers* **inactive**.
Enable asks `controller_manager` to activate them (`/controller_manager/switch_controller`, strict); disable asks it
to deactivate them. Deactivating stops the wheels: `diff_drive_controller` zeroes its wheel commands on deactivate,
and `AddFeedforwardController` writes zero on deactivate (finding H-7). On the mock this is checkable, because
the mock echoes each command back as the wheel's velocity: after a disable, the wheel velocities read 0.

| Mode | Launch file | Always active | Motion controllers (enable = activate these) |
| --- | --- | --- | --- |
| tank | `rocko_tank_mode.launch.py` | `joint_state_broadcaster` | `diffbot_base_controller` |
| balance | `rocko.launch.py` | `joint_state_broadcaster` (supervised only: it publishes the pitch rule 2 reads) | the four cascade controllers, as one group |

In balance mode, enable also publishes `/tipover_rearm`, so a robot whose cutoff tripped while disabled starts
balancing once it is stood up and enabled. The controller refuses that re-arm outside the band (rule 2 checks first,
so a refusal there means the pitch changed in between).

This table lives in **one** place in code: `rocko_env/supervisor/modes.py`. Adding a mode is adding a row.

## How a mode change works (why the stack restarts)

Tank and balance are not two controller sets on one stack. They load **different URDFs** (balance also loads the IMU
plugin), **different controller files**, and run at **different loop rates** (50 Hz and 100 Hz). Rates and gains are
a human's decision (CLAUDE.md §3). So a mode change **stops the old stack and starts the new one**: the supervisor runs
each mode's existing launch file as a child process, with `supervised:=true`.

`supervised:=true` makes a mode's launch file leave out what the supervisor's base launch already runs: the joy node,
the dashboard and the Foxglove bridge. It also makes the launch file spawn the motion controllers inactive. With
`supervised:=false` (the default), each launch file behaves exactly as before, for bench tests by hand.

**Speed caps.** The supervisor passes each mode's caps on as `max_speed:=` and `max_turn:=`, taken from its own
launch arguments `<mode>_max_speed` and `<mode>_max_turn` (for example `pi/launch.sh robot tank_max_speed:=0.3`).
Each is a unitless fraction in [0, 1] (what of, is in `CAP_ARGS` in `modes.py`). Left empty (the default), the mode
launch's own default applies. A value that is not a number in [0, 1] becomes **0** with an error in the supervisor's
log: a bad `max_speed` means no driving, a bad `max_turn` no turning. Two other choices were worse: passed on, a word
fails the whole launch (and at boot nobody reads why); dropped, the default applies, and a typo such as `0,3` would
give tank mode full speed. The command is built by `launch_command()` in `modes.py`.

**Stopping a stack.** The supervisor sends SIGINT to the child's whole *process group* (every node at once, as
systemd's default does). Stopping only `ros2 launch` made `ros2_control_node` hang in the motor plugin's encoder
call (finding C-1; seen on 2026-11-22). If the group has not exited after `stop_timeout_s`, the supervisor sends
SIGKILL (it also does when the child launch dies on its own: the whole process group is killed, so no node of the stack is left running) and reports `fault` with that reason, because a killed motor plugin may leave a PWM pin at its last level
(H-7). A person should look at the robot when that message appears.

**Starting a stack.** `phase` stays `starting` until `controller_manager` lists every controller in the table in the
expected state (always-active ones `active`, motion ones `inactive`) and every expected node is up. If that has not
happened after `start_timeout_s`, the phase is `fault`.

A switch takes several seconds (stop, then the launch's own start-up). It was measured on the mock in the journal
entry, not on the Pi.

## What starts at boot

```
systemd: rocko.service ──> pi/launch.sh robot ──> ros2 launch rocko_env rocko_robot.launch.py
                                                    ├── Supervisor.py        (state, services, starts/stops stacks)
                                                    ├── game_controller_node (joy: /joy, always on, every mode)
                                                    ├── Dashboard.py         (web page, port 8080)
                                                    └── foxglove_bridge
                              Supervisor.py ──child──> ros2 launch rocko_env <mode's launch file> supervised:=true
```

What the child stack holds (read from the launch files with `supervised:=true` and `modes.py`; "inactive" means
loaded but not commanding until the supervisor's enable):

```
tank:    rocko_tank_mode.launch.py
           ├── ros2_control_node      motor plugins (no IMU: the tank xacro has no IMU block)
           │     ├── joint_state_broadcaster   active
           │     └── diffbot_base_controller   inactive (diff_drive_controller)
           ├── robot_state_publisher
           └── TankTeleop.py          /joy -> diffbot_base_controller/cmd_vel, only while enabled
balance: rocko.launch.py
           ├── ros2_control_node      motor plugins and the BNO055 IMU plugin
           │     ├── velocity_pid_controller, pitch_pid_controller,
           │     │   left/right_add_feedforward_controller   inactive (the cascade)
           │     └── joint_state_broadcaster   active (the supervisor reads pitch from it)
           ├── robot_state_publisher
           ├── BalanceTeleop.py       /joy -> diffbot_base_controller/cmd_vel, only while enabled
           └── DiffDriveController.py cmd_vel -> velocity_pid_controller/reference and the feedforward topics
```

Controllers are loaded by short-lived `spawner` processes; `joint_state_broadcaster` is spawned after the motion
controllers' spawner exits, so two spawners never switch at once. With `use_mock_hardware:=true` the plugins are
`mock_components/GenericSystem` instead, and the launch prints a "MOCK HARDWARE" line first.

`rocko.service` replaced the older `rocko-tank.service` and `rocko-dashboard.service` (removed from the repo
2026-11-26); installing it removes any copy still on the Pi, because two launches would fight over the motors. `pi/launch.sh` refuses any other launch
while it runs. Install and remove: [run-on-the-pi.md](../guides/run-on-the-pi.md).

## The ROS interfaces

All in `rocko_interfaces`:

| Name | Kind | What |
| --- | --- | --- |
| `/rocko/state` | topic, `rocko_interfaces/msg/RobotState`, reliable, transient-local (a new subscriber gets the last one) | The state above. Published on every change and at 2 Hz |
| `/rocko/set_enabled` | service, `rocko_interfaces/srv/SetEnabled` (`bool enabled`, `string source` → `bool success`, `string message`) | Enable or disable |
| `/rocko/set_mode` | service, `rocko_interfaces/srv/SetMode` (`string mode`, `string source` → `bool success`, `string message`) | Change mode. Returns as soon as the change is accepted or refused; watch `phase` for the rest |

The supervisor is **not** on the 100 Hz path: it is a separate process that only changes which controllers are active.
It never blocks its own callbacks (service calls to `controller_manager` are asynchronous), so a slow
`controller_manager` cannot freeze the state or the controller combos.

## Growing it: what the design leaves room for

- **More modes** (for example autonomous tank, autonomous balance): a row in `modes.py` with its launch file and
  controllers. The rules above apply unchanged.
- **Who is driving** (gamepad, app, autonomy): a future `control_source` part of the state, with the same pattern:
  ask, the supervisor decides, everyone reads `/rocko/state`. Not built.
- **Optional capabilities** (a lidar, a camera): each a launch file started and stopped as another child process,
  listed in the state as on/off. Not built.

## What is not true yet

- On the robot (2026-11-24, owner present): `rocko.service` boots into tank, disabled; LB+RB and LT+RT+RB worked from
  the 8BitDo; balance mode reached `ready` and was left disabled. Balance mode was enabled on the floor on 2026-11-25 and did
  not balance ([journal](../journal/2026-11-25-level-pitch-offset-and-lean-clamp.md)). The speed caps (above) have run on the mock only. With the cascade inactive the base launch did not
  change the loop's timing measurably ([journal](../journal/2026-11-24-loop-timing-on-the-pi.md)); with it enabled,
  unmeasured.
- Balance mode has never balanced on the robot. The balancing queue is in [next-steps.md](../next-steps.md).
- The trigger "pressed" reading (|axis| > 0.5) is inferred from the joy node's code, not seen on the 8BitDo.
