# Robustness sweep of the sim gains (+/-30 %): results

Written by `sim/parameter_sweep.py` from `sim/results/2026-11-25-measured-signs-sweep.csv`; do not edit by hand. Explained in [the stage 5c-2 journal](../../docs/journal/2026-11-03-stage5c2-sweep.md).

**A MODEL WITH ESTIMATED, UNMEASURED PARAMETERS** ([sim-plant.md §4](../../docs/architecture/sim-plant.md)), with `encoder_sign:=1` as MEASURED on the robot (proposal 0006). PASS means "balances in simulation under an estimated model"; nothing here is about the robot.

```
Report written: 2026-11-25
Commit(s):      49edea5+dirty (the commit the runs were made on; +dirty = uncommitted changes)
Command:        verify_sim.sh scenarios U I V, then verify_sim.sh scenarios V x2, per case,
                with SIM_PLANT_OVERRIDES="body_mass_kg:=.. com_height_m:=.. motor_stall_torque_Nm:=.. imu_delay_ms:=.."
Launch (U):     rocko_sim.launch.py teleop:=false initial_pitch_deg:=2.0 encoder_sign:=1 + the case's four overrides
Launch (I):     rocko_sim.launch.py teleop:=false encoder_sign:=1 impulse_time_s:=2.0 impulse_angular_velocity_deg_per_s:=100 + the case's four overrides
Launch (V):     rocko_sim.launch.py teleop:=true encoder_sign:=1 (fake gamepad: 0, then 0.1 m/s from 1 s) + the case's four overrides
Config:         rocko_sim_controllers.yaml: pitch p 15.0 i 0.0 d 1.0 (D on pitch_rate: True), velocity p -12.5 i 0.0 d -0.2
Thresholds:     U_SETTLE_FROM_S=5, U_SETTLED_DEG=2.0, U_MAX_SPEED_M_S=0.05, I_SETTLE_S=3.0, I_SETTLED_DEG=2.0, V_RISE_FRACTION=0.9, V_RISE_S=3.0, V_OVERSHOOT_PCT=30, V_PITCH_DEG=10.0, TIPOVER_DEG=35, SCENARIO_ENCODER_SIGN=1 (verify_sim.sh, unchanged)
Case verdict:   FAIL if any run failed; MARGINAL if all passed but V's overshoot > 27 % in a run
Battery:        n/a (sim; SimPlant's battery_voltage_V 12.0)
Surface:        n/a (sim; no floor model)
Hardware:       SimPlant (simulated), not the mock, not the robot
Logs:           verify_sim.sh output per call, inside the dev container (column `log` of the CSV)
```

m = body_mass_kg (2.0 kg), h = com_height_m (0.2 m), k = motor_stall_torque_Nm (0.774725 N m), d = imu_delay_ms (20 ms); `-` = -30 %, `+` = +30 %. U: largest |pitch| and |v| from 5 s on. I: settle time after the push (peak |pitch|). V: overshoot min-max over the repeats, rise to 90 %. Linear model: `cascade_linearisation.py` with the same four changes (largest \|z\| per 10 ms cycle, < 1 = stable; time screen with SimPlant's write quirks).

| Case | Kind | m kg | h m | k N m | d ms | U | I | V | **Verdict** | Linear model | Linear vs sim |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `nominal` | nominal | 2.0 | 0.2 | 0.774725 | 20 | 0.0502 deg, 0.001449 m/s | 0.9011 s (peak 4.545 deg) | -5.6--4.8 %, rise 1.74-1.77 s (3 runs) | **PASS** | \|z\| 0.9775; U pass, I pass, V -5.7 % | agree |
| `m-` | one at a time | 1.4 | 0.2 | 0.774725 | 20 | 0.05887 deg, 0.001481 m/s | 0.881 s (peak 4.136 deg) | -4.7-4.9 %, rise 1.75-1.83 s (3 runs) | **PASS** | \|z\| 0.9796; U pass, I pass, V -5.0 % | agree |
| `m+` | one at a time | 2.6 | 0.2 | 0.774725 | 20 | 0.05034 deg, 0.001576 m/s | 0.8906 s (peak 4.747 deg) | -5.1-4.5 %, rise 1.64-1.73 s (3 runs) | **PASS** | \|z\| 0.9758; U pass, I pass, V -4.6 % | agree |
| `h-` | one at a time | 2.0 | 0.14 | 0.774725 | 20 | 0.1582 deg, 0.003433 m/s | 0.8799 s (peak 3.657 deg) | -5.4--3.7 %, rise 1.72-2.04 s (3 runs) | **PASS** | \|z\| 0.9814; U pass, I pass, V -5.4 % | agree |
| `h+` | one at a time | 2.0 | 0.26 | 0.774725 | 20 | 0.04952 deg, 0.002075 m/s | 0.9169 s (peak 5.688 deg) | -5.0-5.5 %, rise 1.53-1.63 s (3 runs) | **PASS** | \|z\| 0.9757; U pass, I pass, V -4.8 % | agree |
| `k-` | one at a time | 2.0 | 0.2 | 0.5423075 | 20 | 0.03563 deg, 0.001548 m/s | 0.8985 s (peak 4.817 deg) | -6.0--4.9 %, rise 1.68-1.70 s (3 runs) | **PASS** | \|z\| 0.9761; U pass, I pass, V -6.0 % | agree |
| `k+` | one at a time | 2.0 | 0.2 | 1.0071425 | 20 | 0.05218 deg, 0.001573 m/s | 0.903 s (peak 4.454 deg) | -4.6-5.6 %, rise 1.46-1.69 s (3 runs) | **PASS** | \|z\| 0.9782; U pass, I pass, V -5.0 % | agree |
| `d-` | one at a time | 2.0 | 0.2 | 0.774725 | 14 | 0.04416 deg, 0.001364 m/s | 0.9077 s (peak 4.473 deg) | -5.3-2.6 %, rise 1.65-1.68 s (3 runs) | **PASS** | \|z\| 0.9768; U pass, I pass, V -5.4 % | agree |
| `d+` | one at a time | 2.0 | 0.2 | 0.774725 | 26 | 0.05378 deg, 0.002061 m/s | 0.8877 s (peak 4.557 deg) | -6.0--5.5 %, rise 1.71-1.95 s (3 runs) | **PASS** | \|z\| 0.9781; U pass, I pass, V -6.1 % | agree |
| `m+h+k+d+` | corner | 2.6 | 0.26 | 1.0071425 | 26 | 0.05633 deg, 0.002002 m/s | 0.9179 s (peak 5.885 deg) | -6.0--3.9 %, rise 1.60-1.65 s (3 runs) | **PASS** | \|z\| 0.9759; U pass, I pass, V -6.1 % | agree |
| `m+h+k+d-` | corner | 2.6 | 0.26 | 1.0071425 | 14 | 0.04646 deg, 0.001471 m/s | 0.912 s (peak 5.733 deg) | -4.5-6.0 %, rise 1.49-1.71 s (3 runs) | **PASS** | \|z\| 0.9745; U pass, I pass, V -5.4 % | agree |
| `m+h+k-d+` | corner | 2.6 | 0.26 | 0.5423075 | 26 | 0.03497 deg, 0.001259 m/s | 0.8677 s (peak 7.599 deg) | -6.9--5.7 %, rise 1.27-1.36 s (3 runs) | **PASS** | \|z\| 0.9733; U pass, I pass, V -7.1 % | agree |
| `m+h+k-d-` | corner | 2.6 | 0.26 | 0.5423075 | 14 | 0.1332 deg, 0.003826 m/s | 0.8674 s (peak 6.635 deg) | -6.7--5.0 %, rise 1.24-1.31 s (3 runs) | **PASS** | \|z\| 0.9716; U pass, I pass, V -6.9 % | agree |
| `m+h-k+d+` | corner | 2.6 | 0.14 | 1.0071425 | 26 | 0.1073 deg, 0.001844 m/s | 0.8598 s (peak 3.659 deg) | -5.2--4.9 %, rise 1.86-1.94 s (3 runs) | **PASS** | \|z\| 0.9810; U pass, I pass, V -5.6 % | agree |
| `m+h-k+d-` | corner | 2.6 | 0.14 | 1.0071425 | 14 | 0.2079 deg, 0.003359 m/s | 0.8709 s (peak 3.749 deg) | -5.1-0.2 %, rise 1.84-1.88 s (3 runs) | **PASS** | \|z\| 0.9802; U pass, I pass, V -4.8 % | agree |
| `m+h-k-d+` | corner | 2.6 | 0.14 | 0.5423075 | 26 | 0.117 deg, 0.002349 m/s | 0.8796 s (peak 4.359 deg) | -6.4-1.3 %, rise 1.90-2.00 s (3 runs) | **PASS** | \|z\| 0.9791; U pass, I pass, V -7.0 % | agree |
| `m+h-k-d-` | corner | 2.6 | 0.14 | 0.5423075 | 14 | 0.02904 deg, 0.0007266 m/s | 0.8917 s (peak 3.96 deg) | -6.4--5.5 %, rise 1.75-1.77 s (3 runs) | **PASS** | \|z\| 0.9779; U pass, I pass, V -6.3 % | agree |
| `m-h+k+d+` | corner | 1.4 | 0.26 | 1.0071425 | 26 | 0.2425 deg, 0.007348 m/s | 0.9289 s (peak 5.091 deg) | -4.7-9.7 %, rise 1.61-1.63 s (3 runs) | **PASS** | \|z\| 0.9786; U pass, I pass, V -6.3 % | agree |
| `m-h+k+d-` | corner | 1.4 | 0.26 | 1.0071425 | 14 | 0.04197 deg, 0.001383 m/s | 0.9331 s (peak 5.138 deg) | -4.6--3.7 %, rise 1.26-1.66 s (3 runs) | **PASS** | \|z\| 0.9776; U pass, I pass, V -4.7 % | agree |
| `m-h+k-d+` | corner | 1.4 | 0.26 | 0.5423075 | 26 | 0.04211 deg, 0.001256 m/s | 0.939 s (peak 6.038 deg) | -6.6--6.3 %, rise 1.67-1.80 s (3 runs) | **PASS** | \|z\| 0.9772; U pass, I pass, V -6.7 % | agree |
| `m-h+k-d-` | corner | 1.4 | 0.26 | 0.5423075 | 14 | 0.0328 deg, 0.001242 m/s | 0.929 s (peak 5.643 deg) | -5.8--5.1 %, rise 1.52-1.65 s (3 runs) | **PASS** | \|z\| 0.9760; U pass, I pass, V -5.7 % | agree |
| `m-h-k+d+` | corner | 1.4 | 0.14 | 1.0071425 | 26 | 0.08994 deg, 0.002102 m/s | 0.8309 s (peak 3.213 deg) | -3.6--3.2 %, rise 1.72-1.89 s (3 runs) | **PASS** | \|z\| 0.9838; U pass, I pass, V -3.5 % | agree |
| `m-h-k+d-` | corner | 1.4 | 0.14 | 1.0071425 | 14 | 0.07518 deg, 0.002121 m/s | 0.8449 s (peak 3.158 deg) | -3.3--2.9 %, rise 1.77-1.86 s (3 runs) | **PASS** | \|z\| 0.9833; U pass, I pass, V -3.4 % | agree |
| `m-h-k-d+` | corner | 1.4 | 0.14 | 0.5423075 | 26 | 0.04757 deg, 0.001332 m/s | 0.8819 s (peak 3.489 deg) | **FAIL** -5.3--4.8 %, rise 2.08-3.11 s (3 runs) | **FAIL (V limit 1/3)** | \|z\| 0.9829; U pass, I pass, V -6.1 % | V: sim FAIL, linear pass |
| `m-h-k-d-` | corner | 1.4 | 0.14 | 0.5423075 | 14 | 0.04498 deg, 0.00132 m/s | 0.8711 s (peak 3.466 deg) | -5.4-0.1 %, rise 1.96-2.42 s (3 runs) | **PASS** | \|z\| 0.9823; U pass, I pass, V -5.5 % | agree |

**Cases by verdict:** FAIL 1, PASS 24. Of the FAIL cases, 0 failed only on V's overshoot (no fall, U and I passed).

## Runs that did not pass

- `m-h-k-d+` V run 2: FAIL: the mean measured velocity did not reach 0.09 m/s within 3.0 s of the step (reached after 3.1074328422546387 s)
