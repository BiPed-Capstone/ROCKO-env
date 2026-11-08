# Robustness sweep of the sim gains (+/-30 %): results

Written by `sim/parameter_sweep.py` from `sim/results/stage5c2-sweep-vmetric2.csv`; do not edit by hand. Explained in [the stage 5c-2 journal](../../docs/journal/2026-11-03-stage5c2-sweep.md).

**A MODEL WITH ESTIMATED, UNMEASURED PARAMETERS** ([sim-plant.md §4](../../docs/architecture/sim-plant.md)), with `encoder_sign:=-1` **ASSUMED** (proposal 0006). PASS means "balances in simulation under an estimated model"; nothing here is about the robot.

```
Report written: 2026-11-08
Commit(s):      03238ca (the commit the runs were made on; +dirty = uncommitted changes)
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
| `nominal` | nominal | 2.0 | 0.2 | 0.774725 | 20 | 0.05839 deg, 0.001509 m/s | 0.7693 s (peak 6.179 deg) | 15.0-24.3 %, rise 0.93-0.93 s (3 runs) | **PASS** | \|z\| 0.9837; U pass, I pass, V 13.9 % | agree |
| `m-` | one at a time | 1.4 | 0.2 | 0.774725 | 20 | 0.04186 deg, 0.001209 m/s | 0.7703 s (peak 5.787 deg) | 15.2-22.1 %, rise 0.92-1.07 s (3 runs) | **PASS** | \|z\| 0.9847; U pass, I pass, V 13.8 % | agree |
| `m+` | one at a time | 2.6 | 0.2 | 0.774725 | 20 | 0.05661 deg, 0.001416 m/s | 0.7848 s (peak 6.921 deg) | **FAIL** 15.5-68.0 %, rise 0.87-0.88 s (3 runs) | **FAIL (V overshoot 1/3)** | \|z\| 0.9830; U pass, I pass, V 13.9 % | V: sim FAIL, linear pass |
| `h-` | one at a time | 2.0 | 0.14 | 0.774725 | 20 | 0.04817 deg, 0.001275 m/s | 0.6998 s (peak 4.838 deg) | 15.0-15.8 %, rise 1.12-1.22 s (3 runs) | **PASS** | \|z\| 0.9858; U pass, I pass, V 13.8 % | agree |
| `h+` | one at a time | 2.0 | 0.26 | 0.774725 | 20 | 0.05598 deg, 0.001587 m/s | 0.849 s (peak 8.571 deg) | 15.4-15.9 %, rise 0.35-0.37 s (3 runs) | **PASS** | \|z\| 0.9826; U pass, I pass, V 13.9 % | agree |
| `k-` | one at a time | 2.0 | 0.2 | 0.5423075 | 20 | 0.04732 deg, 0.001219 m/s | 0.78 s (peak 6.987 deg) | 15.3-21.6 %, rise 0.86-0.86 s (3 runs) | **PASS** | \|z\| 0.9831; U pass, I pass, V 13.9 % | agree |
| `k+` | one at a time | 2.0 | 0.2 | 1.0071425 | 20 | 0.03634 deg, 0.0009888 m/s | 0.7622 s (peak 5.944 deg) | 15.9-21.5 %, rise 0.95-0.98 s (3 runs) | **PASS** | \|z\| 0.9841; U pass, I pass, V 13.8 % | agree |
| `d-` | one at a time | 2.0 | 0.2 | 0.774725 | 14 | 0.04793 deg, 0.001373 m/s | 0.7411 s (peak 5.914 deg) | 15.8-19.1 %, rise 0.91-0.94 s (3 runs) | **PASS** | \|z\| 0.9835; U pass, I pass, V 13.8 % | agree |
| `d+` | one at a time | 2.0 | 0.2 | 0.774725 | 26 | 0.07222 deg, 0.001627 m/s | 0.7806 s (peak 6.871 deg) | **COULD NOT JUDGE/COULD NOT JUDGE** nan-nan %, rise 0.93-0.97 s (3 runs) | **COULD NOT JUDGE** | \|z\| 0.9839; U pass, I pass, V 13.9 % | agree |
| `m+h+k+d+` | corner | 2.6 | 0.26 | 1.0071425 | 26 | 0.08824 deg, 0.002273 m/s | 0.842 s (peak 9.088 deg) | 15.7-18.6 %, rise 0.32-0.33 s (3 runs) | **PASS** | \|z\| 0.9826; U pass, I pass, V 13.9 % | agree |
| `m+h+k+d-` | corner | 2.6 | 0.26 | 1.0071425 | 14 | 0.05008 deg, 0.001234 m/s | 0.8294 s (peak 7.727 deg) | 15.3-19.1 %, rise 0.83-0.87 s (3 runs) | **PASS** | \|z\| 0.9822; U pass, I pass, V 13.8 % | agree |
| `m+h+k-d+` | corner | 2.6 | 0.26 | 0.5423075 | 26 | 0.03737 deg, 0.001086 m/s | 1.39 s (peak 12.16 deg) | **FAIL/FAIL/FAIL** 45.3-46.7 %, rise 0.32-0.33 s (3 runs) | **FAIL (V overshoot 3/3)** | \|z\| 0.9872; U pass, I pass, V 44.8 % FAIL | agree |
| `m+h+k-d-` | corner | 2.6 | 0.26 | 0.5423075 | 14 | 0.1307 deg, 0.003281 m/s | 0.9531 s (peak 10.18 deg) | 17.2-27.7 %, rise 0.35-0.37 s (3 runs) | **MARGINAL** | \|z\| 0.9807; U pass, I pass, V 16.2 % | agree |
| `m+h-k+d+` | corner | 2.6 | 0.14 | 1.0071425 | 26 | 0.07041 deg, 0.001657 m/s | 0.6909 s (peak 4.969 deg) | 16.7-18.9 %, rise 1.04-1.10 s (3 runs) | **PASS** | \|z\| 0.9855; U pass, I pass, V 13.9 % | agree |
| `m+h-k+d-` | corner | 2.6 | 0.14 | 1.0071425 | 14 | 0.05877 deg, 0.001059 m/s | 0.6901 s (peak 4.581 deg) | 17.1-29.3 %, rise 1.09-1.11 s (3 runs) | **MARGINAL** | \|z\| 0.9852; U pass, I pass, V 13.7 % | agree |
| `m+h-k-d+` | corner | 2.6 | 0.14 | 0.5423075 | 26 | 0.03897 deg, 0.0009419 m/s | 0.791 s (peak 6.975 deg) | 15.0-16.5 %, rise 0.90-1.02 s (3 runs) | **PASS** | \|z\| 0.9845; U pass, I pass, V 13.9 % | agree |
| `m+h-k-d-` | corner | 2.6 | 0.14 | 0.5423075 | 14 | 0.0325 deg, 0.0009452 m/s | 0.6317 s (peak 5.802 deg) | 15.1-27.9 %, rise 0.82-0.98 s (3 runs) | **MARGINAL** | \|z\| 0.9841; U pass, I pass, V 13.9 % | agree |
| `m-h+k+d+` | corner | 1.4 | 0.26 | 1.0071425 | 26 | 0.05641 deg, 0.001335 m/s | 0.8189 s (peak 7.734 deg) | 15.2-16.0 %, rise 0.91-0.97 s (3 runs) | **PASS** | \|z\| 0.9840; U pass, I pass, V 13.9 % | agree |
| `m-h+k+d-` | corner | 1.4 | 0.26 | 1.0071425 | 14 | 0.06481 deg, 0.001607 m/s | 0.8289 s (peak 6.672 deg) | 15.4-21.0 %, rise 0.92-0.95 s (3 runs) | **PASS** | \|z\| 0.9836; U pass, I pass, V 13.7 % | agree |
| `m-h+k-d+` | corner | 1.4 | 0.26 | 0.5423075 | 26 | 0.05436 deg, 0.00136 m/s | 0.8803 s (peak 9.717 deg) | 15.6-18.4 %, rise 0.33-0.34 s (3 runs) | **PASS** | \|z\| 0.9833; U pass, I pass, V 14.0 % | agree |
| `m-h+k-d-` | corner | 1.4 | 0.26 | 0.5423075 | 14 | 0.05088 deg, 0.00141 m/s | 0.86 s (peak 5.554 deg) | 15.2-21.1 %, rise 0.41-0.87 s (3 runs) | **PASS** | \|z\| 0.9828; U pass, I pass, V 13.9 % | agree |
| `m-h-k+d+` | corner | 1.4 | 0.14 | 1.0071425 | 26 | 0.0713 deg, 0.001665 m/s | 0.6707 s (peak 4.317 deg) | 15.3-17.6 %, rise 1.20-1.26 s (3 runs) | **PASS** | \|z\| 0.9871; U pass, I pass, V 13.8 % | agree |
| `m-h-k+d-` | corner | 1.4 | 0.14 | 1.0071425 | 14 | 0.06498 deg, 0.001063 m/s | 0.671 s (peak 3.891 deg) | 15.3-16.9 %, rise 1.23-1.24 s (3 runs) | **PASS** | \|z\| 0.9868; U pass, I pass, V 13.6 % | agree |
| `m-h-k-d+` | corner | 1.4 | 0.14 | 0.5423075 | 26 | 0.05999 deg, 0.001237 m/s | 0.7113 s (peak 5.22 deg) | 15.1-23.3 %, rise 1.15-1.21 s (3 runs) | **PASS** | \|z\| 0.9866; U pass, I pass, V 13.9 % | agree |
| `m-h-k-d-` | corner | 1.4 | 0.14 | 0.5423075 | 14 | 0.0499 deg, 0.001217 m/s | 0.6988 s (peak 4.417 deg) | 14.7-29.9 %, rise 1.16-1.18 s (3 runs) | **MARGINAL** | \|z\| 0.9863; U pass, I pass, V 13.8 % | agree |

**Cases by verdict:** COULD NOT JUDGE 1, FAIL 2, MARGINAL 4, PASS 18. Of the FAIL cases, 2 failed only on V's overshoot (no fall, U and I passed).

## Runs that did not pass

- `m+` V run 2: FAIL: overshoot 68.1 % (limit 30.0 %; highest exported sample 67.0 %)
- `d+` V run 1: COULD NOT JUDGE: two plant samples have the same or decreasing stamps, so a period is unknown
- `d+` V run 3: COULD NOT JUDGE: two plant samples have the same or decreasing stamps, so a period is unknown
- `m+h+k-d+` V run 1: FAIL: overshoot 45.3 % (limit 30.0 %; highest exported sample 45.1 %)
- `m+h+k-d+` V run 2: FAIL: overshoot 46.7 % (limit 30.0 %; highest exported sample 47.1 %)
- `m+h+k-d+` V run 3: FAIL: overshoot 46.1 % (limit 30.0 %; highest exported sample 48.0 %)
