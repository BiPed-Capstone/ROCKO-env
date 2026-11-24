# Gyro check on the robot: which gyro axis is pitch, its sign and unit (proposal 0008, precondition 3)

- **Date / commit:** 2026-11-23, built on `bc68de0` (the commit that adds this file holds the change)
- **Branch:** ai-forward
- **Environment:** the robot's Raspberry Pi 5 (Ubuntu 24.04, Jazzy), BNO055 on I²C bus 1,
  `adafruit-circuitpython-bno055` 5.4.22 (installed by the owner today, with `gpiod`); the agent over SSH with the
  owner's permission; the owner at the robot, tilting it by hand. `rocko.service` running in tank mode, disabled
  (tank mode does not use the IMU, so the logger had the bus to itself).

## Goal and success criteria

Settle the three unknowns of finding H-6 before the pitch loop's D term may use the gyro: (1) which gyro axis is the
robot's pitch, (2) its sign, defined as "positive while pitch increases" (`Icm20948Data.srv`), (3) its unit
(deg/s after the node's `degrees()`, not 57 times off). Done when one axis's integral tracks the Euler pitch through
a large tilt with a factor near +1 or -1, and the other two do not.

## Plan

Log the raw sensor (Euler angles and all three gyro axes) with a small script instead of the node, so every axis is
recorded at once: 4 s at rest, then 40 s while the owner tilts forward, back, backward, back by hand (asked for
about 20 deg; the peaks reached were -55 and +42.5 deg).

## What changed

- `tools/imu_tilt_log.py` (new): the logger, runnable on the Pi.
- `docs/journal/data/2026-11-23-gyro-check/rest.csv`, `tilt.csv`: the raw logs (pitch is the raw sensor value,
  without the node's +7.6 deg offset).
- `ICM20948.py`: `pitch_rate_sign` default **-1** (was the guess +1), with the evidence in the comment; the start-up
  warning no longer says "unverified". `read_pitch_rate` stays false: using the rate needs the gated YAML change
  (proposal 0008 precondition 4).
- `tools/imu_node_test.py`: the two checks that assumed +1. Proposals 0008 and 0004, next-steps and the ladder now say
  measured.

## Verification

| What | Command | Result |
| --- | --- | --- |
| at rest, 4 s | `~/.venv/bin/python3 /tmp/imu_tilt_log.py 4 /tmp/rest.csv` | 90 samples; raw pitch -2.38 deg (+7.6 = the 5.16 deg the dashboard showed), steady; gyro noise (sd) 0.10, 0.13, 0.09 deg/s on X, Y, Z, means within 0.04 deg/s of 0. The first samples read 0 while the sensor started |
| tilt, 40 s | same, 40 s | 902 samples, one every 44 ms. Raw pitch went to **-55 deg** in the first tilt and **+42.5 deg** in the second, then back to -0.5 deg |
| which axis | least-squares fit of the pitch change to each gyro axis's integral | X: factor **-0.966**, rms error **0.9 deg**; Y: +2.2, rms 19.3 deg; Z: -2.0, rms 19.3 deg. So **axis 0 (X)**, **sign -1**, unit correct (|factor| near 1, not 57). Method: the 3 start-up rows where pitch and roll are both 0 dropped (905 rows -> 902); each axis integrated with the trapezoid rule; pitch change measured from the first valid row; a least-squares fit through the origin (a reviewer recomputed it with rectangle steps: -0.967, 0.92 deg) |
| lag | cross-correlation of the pitch's own rate with -gyro X | best at 0 ms (correlation 0.993), so any lag of the fused angle behind the gyro is below what 44 ms sampling resolves |
| host | `python3 -I tools/imu_node_test.py` | 7/7 after updating the two checks that assumed +1 |

Not run: no tier of the ladder changes with a node parameter default except the host test. The node itself with
`read_pitch_rate:=true` was not run on the robot (the logger read the sensor directly, through the same driver
calls the node makes: `.gyro` and `.euler`).

## What this also says (for proposal 0006, not settled)

If the first tilt was **forward** as asked (toward where the robot drives when the left stick is pushed up, which is a
positive `linear.x`), then **pitch decreases when the robot leans forward**. That is half of proposal 0006's sign check
(`s_θ`). The owner confirmed afterwards that the first tilt was forward, so this is now recorded in proposal 0006's
Decision section: pitch decreases when the robot leans forward.

## Not verified — needs a human

- The rate through the real node and plugin (`read_pitch_rate:=true`), which comes with proposal 0008 precondition 4.
- Re-measure if the IMU is moved or remounted.
