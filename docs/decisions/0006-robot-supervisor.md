# ADR 0006 — A supervisor node owns the robot's state and starts the motor stacks

- **Status:** Proposed. The owner chose the parts marked "owner" on 2026-11-23, in the session recorded in
  [the journal](../journal/2026-11-23-robot-supervisor.md)
- **Date:** 2026-11-23
- **Deciders:** the owner (requirements and the safety trade-off), the agent (design)

## Context

Until now a mode was a launch file. Tank mode started at boot (`rocko-tank.service`); to balance, a person stopped
that service over SSH and ran `pi/launch.sh balance`. The dashboard was its own service and could only display.
The owner asked for: switching tank ⇄ balance from the controller (LT+RT+RB) or the dashboard; a **disabled** state,
entered and left with LB+RB or dashboard buttons, the same in both modes, with mode changes allowed only while
disabled; the dashboard always showing the mode and which nodes are running; boot into tank mode; and room to grow
(autonomous modes, app versus controller driving, optional sensors such as a lidar).

The two modes do not share a motor stack: different URDFs (balance loads the IMU plugin), different controller
files and different loop rates (50 Hz, 100 Hz), and rates and gains are a human's call (CLAUDE.md §3).

## Decision

1. **One supervisor node** (`Supervisor.py`) starts at boot inside `rocko_robot.launch.py` and owns the state:
   `mode`, `enabled`, `phase`. Everything else asks it through two services and reads one latched topic.
   Rules and interfaces: [robot-supervisor.md](../architecture/robot-supervisor.md).
2. **A mode change restarts the motor stack** (owner chose this, 2026-11-23): the supervisor runs the mode's existing
   launch file as a child process and stops its whole process group with SIGINT, with a SIGKILL after a timeout.
3. **Enable/disable is controller activation** inside the running stack, not a restart, so it takes milliseconds.
4. **Enable is a toggle, not a held button** (owner, 2026-11-23). This **replaces tank mode's hold-LB deadman**, a
   safety behavior, which CLAUDE.md §3 leaves to a human: the owner made that call in this session. To make up for
   it: an automatic disable when the controller goes silent, the teleop nodes' own zero on silence, stick re-centre
   after enable, and the velocity-command timeouts and the tipover cutoff, which are unchanged.
5. **The joy node, dashboard and Foxglove bridge move to the base launch**, so they survive a mode change and the
   controller combos work in every state.

## Alternatives considered

- **One controller_manager with both modes loaded, switching controllers** (the usual ros2_control way, switches in
  under 100 ms). Rejected for now: it needs one URDF, one controller file and one loop rate for both modes, all
  human-gated, and it puts the IMU plugin (finding C-1) under tank mode as well. It remains the better design once
  C-1 is fixed (proposal 0010) and the rates are settled; `modes.py` and the services would stay as they are.
- **systemd units per mode, switched by `systemctl`**: no ROS interface for the dashboard or controller to ask
  through, and the state lives in systemd rather than anywhere the robot can publish it.
- **A behavior tree or a state-machine library (SMACH, FlexBE)**: more machinery than two modes and an enable need,
  and harder for a student to read than one plain Python file with a test. Worth reconsidering when autonomous
  modes arrive.
- **Keep the hold-LB deadman, add a disabled state on top**: safer, and what the agent would have chosen; the owner
  chose the toggle.

## Consequences

- A mode change takes seconds, not milliseconds, and briefly has no motor stack at all, which is safe because it
  happens only while disabled.
- A stack that will not stop is SIGKILLed after a timeout; the supervisor reports it as a fault, because H-7 means
  a PWM pin may be left at its last level.
- The dashboard can now change the robot's state. It still has no login (see use-the-dashboard.md).
- Disabling in balance mode means the robot falls. That is what disable means; it is not a soft landing.
