# Robustness sweep of the sim gains (+/-30 %): results

Written by `sim/parameter_sweep.py` from `sim/results/stage5c2-sweep.csv`; do not edit by hand. Explained in [the stage 5c-2 journal](../../docs/journal/2026-11-03-stage5c2-sweep.md).

**A MODEL WITH ESTIMATED, UNMEASURED PARAMETERS** ([sim-plant.md §4](../../docs/architecture/sim-plant.md)), with `encoder_sign:=-1` **ASSUMED** (proposal 0006). PASS means "balances in simulation under an estimated model"; nothing here is about the robot.

```
Report written: 2026-11-04
Commit(s):      77f8ee1 (the commit the runs were made on; +dirty = uncommitted changes)
Command:        verify_sim.sh scenarios U I V, then verify_sim.sh scenarios V x2, per case,
                with SIM_PLANT_OVERRIDES="body_mass_kg:=.. com_height_m:=.. motor_stall_torque_Nm:=.. imu_delay_ms:=.."
Launch (U):     rocko_sim.launch.py teleop:=false initial_pitch_deg:=2.0 encoder_sign:=-1 + the case's four overrides
Launch (I):     rocko_sim.launch.py teleop:=false encoder_sign:=-1 impulse_time_s:=2.0 impulse_angular_velocity_deg_per_s:=100 + the case's four overrides
Launch (V):     rocko_sim.launch.py teleop:=true encoder_sign:=-1 (fake gamepad: 0, then 0.1 m/s from 1 s) + the case's four overrides
Config:         rocko_sim_controllers.yaml: pitch p 15.0 i 0.0 d 1.0 (D on pitch_rate: True), velocity p 25.0 i 0.0 d 0.2
Thresholds:     U_SETTLE_FROM_S=5, U_SETTLED_DEG=2.0, U_MAX_SPEED_M_S=0.05, I_SETTLE_S=3.0, I_SETTLED_DEG=2.0, V_RISE_FRACTION=0.9, V_RISE_S=3.0, V_OVERSHOOT_PCT=30, V_PITCH_DEG=10.0, TIPOVER_DEG=35, SCENARIO_ENCODER_SIGN=-1 (verify_sim.sh, unchanged)
Case verdict:   FAIL if any run failed; MARGINAL if all passed but V's overshoot > 27 % in a run
Battery:        n/a (sim; SimPlant's battery_voltage_V 12.0)
Surface:        n/a (sim; no floor model)
Hardware:       SimPlant (simulated), not the mock, not the robot
Logs:           verify_sim.sh output per call, inside the dev container (column `log` of the CSV)
```

m = body_mass_kg (2.0 kg), h = com_height_m (0.20 m), k = motor_stall_torque_Nm (0.774725 N m), d = imu_delay_ms (20 ms); `-` = -30 %, `+` = +30 %. U: largest |pitch| and |v| from 5 s on. I: settle time after the push (peak |pitch|). V: overshoot min-max over the repeats, rise to 90 %. Linear model: `cascade_linearisation.py` with the same four changes (largest \|z\| per 10 ms cycle, < 1 = stable; time screen with SimPlant's write quirks).

| Case | Kind | m kg | h m | k N m | d ms | U | I | V | **Verdict** | Linear model | Linear vs sim |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `nominal` | nominal | 2.0 | 0.2 | 0.774725 | 20 | 0.06485 deg, 0.001702 m/s | 0.7409 s (peak 6.056 deg) | **FAIL** 23.2-32.6 %, rise 0.89-0.93 s (3 runs) | **FAIL (V overshoot 1/3)** | \|z\| 0.9837; U pass, I pass, V 13.9 % | V: sim FAIL, linear pass |
| `m-` | one at a time | 1.4 | 0.2 | 0.774725 | 20 | 0.06768 deg, 0.00182 m/s | 0.7609 s (peak 5.718 deg) | 18.9-20.4 %, rise 0.92-1.03 s (3 runs) | **PASS** | \|z\| 0.9847; U pass, I pass, V 13.8 % | agree |
| `m+` | one at a time | 2.6 | 0.2 | 0.774725 | 20 | 0.05782 deg, 0.001309 m/s | 0.7902 s (peak 6.976 deg) | 19.4-20.4 %, rise 0.84-0.87 s (3 runs) | **PASS** | \|z\| 0.9830; U pass, I pass, V 13.9 % | agree |
| `h-` | one at a time | 2.0 | 0.14 | 0.774725 | 20 | 0.0399 deg, 0.0009682 m/s | 0.7102 s (peak 4.958 deg) | **FAIL** 18.0-33.8 %, rise 1.07-1.12 s (3 runs) | **FAIL (V overshoot 1/3)** | \|z\| 0.9858; U pass, I pass, V 13.8 % | V: sim FAIL, linear pass |
| `h+` | one at a time | 2.0 | 0.26 | 0.774725 | 20 | 0.05843 deg, 0.001928 m/s | 0.8494 s (peak 8.23 deg) | 19.3-19.9 %, rise 0.35-0.38 s (3 runs) | **PASS** | \|z\| 0.9826; U pass, I pass, V 13.9 % | agree |
| `k-` | one at a time | 2.0 | 0.2 | 0.5423075 | 20 | 0.04854 deg, 0.001318 m/s | 0.8036 s (peak 7.024 deg) | 19.5-28.7 %, rise 0.85-0.87 s (3 runs) | **MARGINAL** | \|z\| 0.9831; U pass, I pass, V 13.9 % | agree |
| `k+` | one at a time | 2.0 | 0.2 | 1.0071425 | 20 | 0.08254 deg, 0.00266 m/s | 0.7598 s (peak 5.368 deg) | 19.5-22.3 %, rise 0.95-0.99 s (3 runs) | **PASS** | \|z\| 0.9841; U pass, I pass, V 13.8 % | agree |
| `d-` | one at a time | 2.0 | 0.2 | 0.774725 | 14 | 0.05674 deg, 0.001477 m/s | 0.7604 s (peak 6.063 deg) | 20.0-25.9 %, rise 0.92-0.94 s (3 runs) | **PASS** | \|z\| 0.9835; U pass, I pass, V 13.8 % | agree |
| `d+` | one at a time | 2.0 | 0.2 | 0.774725 | 26 | 0.07435 deg, 0.001532 m/s | 0.7629 s (peak 6.391 deg) | **FAIL** 19.6-38.4 %, rise 0.90-0.92 s (3 runs) | **FAIL (V overshoot 1/3)** | \|z\| 0.9839; U pass, I pass, V 13.9 % | V: sim FAIL, linear pass |
| `m+h+k+d+` | corner | 2.6 | 0.26 | 1.0071425 | 26 | 0.07513 deg, 0.002227 m/s | 0.821 s (peak 8.619 deg) | **FAIL** 24.0-88.3 %, rise 0.32-0.33 s (3 runs) | **FAIL (V overshoot 1/3)** | \|z\| 0.9826; U pass, I pass, V 13.9 % | V: sim FAIL, linear pass |
| `m+h+k+d-` | corner | 2.6 | 0.26 | 1.0071425 | 14 | 0.07561 deg, 0.00239 m/s | 0.8412 s (peak 7.784 deg) | 19.3-28.5 %, rise 0.83-0.87 s (3 runs) | **MARGINAL** | \|z\| 0.9822; U pass, I pass, V 13.8 % | agree |
| `m+h+k-d+` | corner | 2.6 | 0.26 | 0.5423075 | 26 | 0.04054 deg, 0.001456 m/s | 1.401 s (peak 12.35 deg) | **FAIL/FAIL/FAIL** 48.9-50.9 %, rise 0.31-0.32 s (3 runs) | **FAIL (V overshoot 3/3)** | \|z\| 0.9872; U pass, I pass, V 44.8 % FAIL | agree |
| `m+h+k-d-` | corner | 2.6 | 0.26 | 0.5423075 | 14 | 0.04077 deg, 0.001129 m/s | 0.9517 s (peak 10.15 deg) | 20.3-28.9 %, rise 0.35-0.36 s (3 runs) | **MARGINAL** | \|z\| 0.9807; U pass, I pass, V 16.2 % | agree |
| `m+h-k+d+` | corner | 2.6 | 0.14 | 1.0071425 | 26 | 0.0616 deg, 0.001423 m/s | 0.7322 s (peak 5.199 deg) | 19.1-20.3 %, rise 1.09-1.11 s (3 runs) | **PASS** | \|z\| 0.9855; U pass, I pass, V 13.9 % | agree |
| `m+h-k+d-` | corner | 2.6 | 0.14 | 1.0071425 | 14 | 0.05368 deg, 0.001219 m/s | 0.6698 s (peak 4.58 deg) | 19.0-19.6 %, rise 1.07-1.08 s (3 runs) | **PASS** | \|z\| 0.9852; U pass, I pass, V 13.7 % | agree |
| `m+h-k-d+` | corner | 2.6 | 0.14 | 0.5423075 | 26 | 0.03954 deg, 0.0008269 m/s | 0.7899 s (peak 7.012 deg) | 19.2-20.3 %, rise 0.92-0.99 s (3 runs) | **PASS** | \|z\| 0.9845; U pass, I pass, V 13.9 % | agree |
| `m+h-k-d-` | corner | 2.6 | 0.14 | 0.5423075 | 14 | 0.0326 deg, 0.0008188 m/s | 0.6413 s (peak 5.803 deg) | 18.5-20.4 %, rise 0.98-1.00 s (3 runs) | **PASS** | \|z\| 0.9841; U pass, I pass, V 13.9 % | agree |
| `m-h+k+d+` | corner | 1.4 | 0.26 | 1.0071425 | 26 | 0.05854 deg, 0.001539 m/s | 0.8309 s (peak 7.693 deg) | 19.2-20.3 %, rise 0.78-0.87 s (3 runs) | **PASS** | \|z\| 0.9840; U pass, I pass, V 13.9 % | agree |
| `m-h+k+d-` | corner | 1.4 | 0.26 | 1.0071425 | 14 | 0.0524 deg, 0.001655 m/s | 0.8196 s (peak 6.843 deg) | 18.8-20.5 %, rise 0.89-0.94 s (3 runs) | **PASS** | \|z\| 0.9836; U pass, I pass, V 13.7 % | agree |
| `m-h+k-d+` | corner | 1.4 | 0.26 | 0.5423075 | 26 | 0.05469 deg, 0.001624 m/s | 0.8844 s (peak 9.659 deg) | 18.1-22.3 %, rise 0.33-0.34 s (3 runs) | **PASS** | \|z\| 0.9833; U pass, I pass, V 14.0 % | agree |
| `m-h+k-d-` | corner | 1.4 | 0.26 | 0.5423075 | 14 | 0.05178 deg, 0.001339 m/s | 0.8677 s (peak 8.199 deg) | 19.1-21.0 %, rise 0.87-0.94 s (3 runs) | **PASS** | \|z\| 0.9828; U pass, I pass, V 13.9 % | agree |
| `m-h-k+d+` | corner | 1.4 | 0.14 | 1.0071425 | 26 | 0.09353 deg, 0.001823 m/s | 0.6801 s (peak 4.216 deg) | 18.8-19.8 %, rise 1.18-1.23 s (3 runs) | **PASS** | \|z\| 0.9871; U pass, I pass, V 13.8 % | agree |
| `m-h-k+d-` | corner | 1.4 | 0.14 | 1.0071425 | 14 | 0.09323 deg, 0.001629 m/s | 0.6795 s (peak 3.877 deg) | **FAIL** 18.7-30.7 %, rise 1.20-1.23 s (3 runs) | **FAIL (V overshoot 1/3)** | \|z\| 0.9868; U pass, I pass, V 13.6 % | V: sim FAIL, linear pass |
| `m-h-k-d+` | corner | 1.4 | 0.14 | 0.5423075 | 26 | 0.05245 deg, 0.00117 m/s | 0.7209 s (peak 5.072 deg) | **FAIL** 19.0-63.4 %, rise 1.19-1.20 s (3 runs) | **FAIL (V overshoot 1/3)** | \|z\| 0.9866; U pass, I pass, V 13.9 % | V: sim FAIL, linear pass |
| `m-h-k-d-` | corner | 1.4 | 0.14 | 0.5423075 | 14 | 0.05752 deg, 0.0012 m/s | 0.6909 s (peak 4.444 deg) | 18.9-28.6 %, rise 1.12-1.17 s (3 runs) | **MARGINAL** | \|z\| 0.9863; U pass, I pass, V 13.8 % | agree |

**Cases by verdict:** FAIL 7, MARGINAL 4, PASS 14. Of the FAIL cases, 7 failed only on V's overshoot (no fall, U and I passed).

## Runs that did not pass

- `nominal` V run 3: FAIL: overshoot 32.6 % (limit 30.0 %)
- `h-` V run 1: FAIL: overshoot 33.8 % (limit 30.0 %)
- `d+` V run 3: FAIL: overshoot 38.4 % (limit 30.0 %)
- `m+h+k+d+` V run 2: FAIL: overshoot 88.3 % (limit 30.0 %)
- `m+h+k-d+` V run 1: FAIL: overshoot 48.9 % (limit 30.0 %)
- `m+h+k-d+` V run 2: FAIL: overshoot 50.9 % (limit 30.0 %)
- `m+h+k-d+` V run 3: FAIL: overshoot 49.1 % (limit 30.0 %)
- `m-h-k+d-` V run 3: FAIL: overshoot 30.7 % (limit 30.0 %)
- `m-h-k-d+` V run 3: FAIL: overshoot 63.4 % (limit 30.0 %)
