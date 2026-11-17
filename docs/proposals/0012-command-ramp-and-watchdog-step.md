# Proposal 0012 — confirm the drive command's ramp limit, and decide whether the watchdog's zero should ramp too

- **Status:** Open
- **Gate:** hardware limit (an acceleration limit on the drive command) / changing a safety behavior
  (the teleop watchdog, finding C-3)
- **Raised:** 2026-11-16, built on `8af19dd`, run journal:
  [2026-11-16](../journal/2026-11-16-teleop-dashboard-logging.md), stage A. Finding
  [H-13](../claude-analysis/balance-loop-findings.md).

## The change

Two decisions. The first is already in the code as a proposed value; the second is not in the code.

1. **Confirm or change `DEFAULT_MAX_LINEAR_ACCEL_M_S2`** in
   `rocko_env/rocko_env/controllers/diffdrive/DiffDriveController.py`. The value is written in the
   node and mirrored as the default of `rocko_sim.launch.py`'s argument; `sim/check_sim_config.py`
   (check 7) fails if the two differ. It is the most the forward drive command may change per second.
2. **Decide whether the watchdog's zero should ramp.** Today, when no command has arrived for
   `command_timeout_s`, the velocity reference and the feedforward go to zero in one cycle. The
   alternative: ramp them to zero at a limit of their own (a "stop" limit, probably larger than the drive
   limit). Check `C3` would then have to judge "ramping down by the deadline and zero by a later one"
   instead of "exactly zero".

## Why

1. A drive command used to arrive as a step, and in the model a 0.5 m/s step leaned the robot 21 deg
   (finding H-13). The ramp removes the step. Its value was picked in the sim, which is a model with
   estimated parameters; nobody has seen what the robot does with it.
2. A step **to zero** is the same disturbance in the other direction. If the laptop's link drops for
   more than 0.2 s while the robot is moving, the watchdog itself may be what tips it. Against that: a
   ramp means the robot keeps moving for longer after the operator has lost control of it, and the
   watchdog's promise ("zero within the timeout") is the simplest one to check and to explain.

## Evidence (and what is not known)

All of it is Tier 3: the model, sim gains, `encoder_sign:=-1` assumed, one run per line unless said.
Scenario `K` is a 0.5 m/s forward command held for 8 s; its limits are a lean of 10 deg and an
overshoot of 30 %.

The sweep of the ramp limit (seven values from "off" to 0.25 m/s^2, with the lean, overshoot and rise
time of each) is in the journal's stage A, *Choosing the value*: with the ramp off the model leans
21 deg; the lean falls steadily as the limit is lowered and is under the 10 deg limit from 0.7 m/s^2
down.

The value in the code is the one the journal names; it was chosen for margin under the 10 deg limit.

For decision 2: in three runs the test harness stalled for longer than 0.2 s, the watchdog fired in the
middle of the drive, and the command stepped to zero and ramped up again. Those runs leaned 16.3, 19.3
and 20.4 deg (ramp limits 0.5, 0.25 and 0.25). Scenario `K` now reports such a run as *could not judge*.

Not known: anything about the robot. Whether the robot's lean for a given ramp resembles the model's.
What a turn step does (the sim is planar; the turn command is not ramped). Whether 0.2 s link gaps
happen on the robot's network.

## The test a human should run

After the tethered sequence of [proposal 0008](0008-candidate-gains-and-tethered-test.md) has the robot
standing: tethered, a person holding the tether, power cut within reach.

Everything below is typed on the **Pi**, in a shell that has run `source /opt/ros/jazzy/setup.bash` and
`source ~/ros2_ws/install/setup.bash`; the robot is started with `ros2 launch rocko_env rocko.launch.py`.

1. With the gamepad or the keyboard, command the smallest forward speed and hold it for 3 s, then
   release. Success: the robot leans a few degrees, moves off, and settles; no lean near 15 deg.
   Abort on the tipover cutoff, a growing oscillation, or a lean that keeps increasing.
2. Record a bag (pitch, the velocity reference, both feedforwards) and note the peak lean. If it is
   large, stop the launch, lower `DEFAULT_MAX_LINEAR_ACCEL_M_S2` in `DiffDriveController.py` and the
   matching default in `rocko_sim.launch.py`, rebuild (`cd ~/ros2_ws && src/ROCKO-env/build_ros.sh`)
   and launch again. Do not start a second `DiffDriveController.py` with `ros2 run`: the launch already
   runs one, and two would publish on the same topics.
3. For decision 2, only once step 1 is calm: while holding the smallest forward speed, stop the laptop
   sender (Ctrl-C). The watchdog fires 0.2 s later. Note the lean. If it is as large as the model's, say
   so on this proposal and the agent writes the ramped stop and its check.
