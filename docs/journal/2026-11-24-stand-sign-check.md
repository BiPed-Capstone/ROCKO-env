# Stand sign check on the robot: the encoder reads positive under a positive command (proposal 0006, `s_v`)

- **Date / commit:** 2026-11-24, on `99dc204` (no code change; the commit that adds this file holds the record)
- **Branch:** ai-forward
- **Environment:** the robot's Pi 5, `rocko.service` in tank mode (50 Hz, `diff_drive_controller`, open loop), the
  per-wheel encoder service of `6974714`. Robot on a stand, both wheels clear. The owner enabled with LB+RB and drove
  with the 8BitDo; the agent logged over SSH with the owner's permission.

## Goal and success criteria

Measure proposal 0006's `s_v`: the sign of `/joint_states` velocity when a positive wheel command turns the wheels.
Done when positive and negative commands each give a consistent encoder sign on both wheels.

## What was recorded

`docs/journal/data/2026-11-24-stand-sign-check/stand_cmd.csv` (`/diffbot_base_controller/cmd_vel`, the teleop's
command) and `stand_js.csv` (`/joint_states`, both wheels), 45 s, `ros2 topic echo --csv`. One line in
`stand_js.csv` reads "A message was lost!!!" (the echo tool dropped a message); it is skipped when reading. The
capture began while the owner was already driving, so it opens mid-motion; the owner moved the stick in short
bursts (full forward, full back, turns both ways) rather than one steady hold.

## Result

| Command | Left encoder velocity | Right encoder velocity |
| --- | --- | --- |
| `linear.x` +15 m/s (full stick), t = 4.2 to 4.6 s | +0.92, then +1.20 m/s | +1.41, then +1.69 m/s |
| `linear.x` -12 to -15 m/s, t = 0.6 to 1.4 s and 5.0 to 5.4 s | negative (-1.34 to -1.93 m/s) | negative (-1.38 to -1.76 m/s) |
| `angular.z` -85 to -94 (turn right) | positive | negative |
| `angular.z` +94 (turn left) | negative | positive |

So **`s_v` is positive** on both wheels, and the turn split is right. With `s_θ` measured "down" on 2026-11-23
([gyro check](2026-11-23-gyro-check.md)), proposal 0006's step 2 applies: one sign must be flipped (option A, the
velocity-loop gains in the gated `rocko_controllers.yaml`; or option B, the encoder). Owner's decision.

The hand-turn test of 2026-11-23 ([encoder journal](2026-11-23-encoder-service-u2.md)) showed a forward hand turn
also reads positive, so a positive command turns the wheels forward.

## What this does not show

- Speed accuracy: the bursts were about 0.4 s, shorter than the encoder filter's ~0.4 s time constant at 50 Hz and
  `diff_drive_controller`'s ramp, so the measured 1.2 to 1.9 m/s is not a steady speed and says nothing about the
  15 m/s command scale. A steady-hold check against a stopwatch is still open.
- Anything about balancing.
