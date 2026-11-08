# Proposal 0007 — measure the robot's physical parameters, then replace the model's estimates

- **Status:** Open
- **Gate:** on-robot step (the IMU delay measurement runs the IMU node alone and the encoder-scale
  measurement runs the robot's software, both on a stand; the rest is bench work)
- **Raised:** 2026-11-04, built on `77664f4`, run journal:
  [final report, stages 4 to 6](../journal/2026-11-04-final-report-stage-4-6.md)

## The change

No code or config changes in this proposal. A human measures eight things, following
[measure-inertia.md](../guides/measure-inertia.md), and records each in the format of
[reproducing-results.md](../guides/reproducing-results.md):

| # | Measure | Replaces (an estimate today) |
| --- | --- | --- |
| 1 | wheel mass, body mass | `wheel_mass_kg` 0.3, `body_mass_kg` 2.0 |
| 2 | axle-to-centre-of-mass distance | `com_height_m` 0.20 |
| 3 | body pitch inertia (bifilar swing, pendulum cross-check) | the box inertia from `body_height_m` 0.40, `body_depth_m` 0.10 |
| 4 | wheel inertia (optional; no parameter yet) | solid disc |
| 5 | wheel rolling radius, belt ratio | `wheel_radius_m` 0.06, `belt_ratio` 1.355 |
| 6 | motor stall torque, no-load speed, currents, the voltage used, battery voltage | the five motor parameters and `battery_voltage_V` 12.0 |
| 7 | IMU delay: motion to a reply at a client (a lower bound; the node alone, no timestamp in the reply) | `imu_delay_ms` 20 |
| 8 | encoder metres per wheel turn | the code's 521.2 counts per metre |

**After the human reports**, the agent updates the `ESTIMATE, NOT MEASURED` table in
[sim-plant.md §4.1](../architecture/sim-plant.md) and its copy in
[the gap page](../architecture/sim-to-real-gap.md), changes the defaults in the three places
`sim/check_sim_config.py` keeps equal, and reruns the scenarios and the ±30 % sweep.

**The wheel radius (step 5) is shared** with [issue #16](https://github.com/BiPed-Capstone/ROCKO-env/issues/16)
and next-steps row 0.7. The repo holds four values (0.06, an effective 0.0813, 0.127 and 0.015 m);
this proposal does not pick one, and neither will the agent: the measured rolling radius is the
human's answer to both.

## Why

Row 2.2 of [next-steps](../next-steps.md). Every gain result so far is "balances in simulation under
an estimated model" ([gap page](../architecture/sim-to-real-gap.md)); measurement is the only thing
that makes the model about this robot. The measurements also give the numbers the tethered test of
[proposal 0008](0008-candidate-gains-and-tethered-test.md) needs as a precondition.

## Evidence

None yet: nothing was measured. Not verified: that the guide's formulas give the right numbers on
this robot (they are standard formulas, applied to a robot nobody here has touched), and that the
IMU-delay method's uncertainty (about ±20 ms at 60 fps, ±35 ms at 30 fps, and a lower bound on the
real delay because the control cycle is not in it) is small enough to be useful against the 20 ms estimate.

## Risk

Steps 1 to 5 are done with the battery disconnected. Step 6 runs a motor at stall from a bench
supply (2 s runs, a current limit). Step 7 runs the IMU node alone (no controllers, no motor
command); step 8 runs the robot's software on a stand with the wheels clear. Keep the power cut
within reach. A wrong radius, mass or voltage entered into the
model would give confident but wrong simulation results, hence the repeats and the spread.

## Test for a human

Follow the guide. Success looks like: eight records, each with its raw readings, three or more
repeats, and the reproducibility block filled in or marked `unknown`. Abort a bench step if a
motor or driver gets hot, or a wheel runs away on the stand.

## Decision

(Human fills in: date, who, accepted/declined, where the records are.)
