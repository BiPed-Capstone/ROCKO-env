# Proposal 0016 — Take the pitch loop's D term from the gyro on the robot (finding H-6, with L-8 handled)

- **Status:** Accepted (owner, 2026-11-24), applied by the agent the same day; robot test pending
- **Gate:** the owner's rule: the wiring of a gain-bearing controller in the real `rocko_controllers.yaml` (next-steps
  handoff item 5, as ordered by the owner on 2026-11-24, says the owner approves this change). It is not a gain key,
  and the hook does not guard it.
- **Raised:** 2026-11-24, built on `a27e9a3`, run journal:
  [2026-11-24 pitch D term from the gyro](../journal/2026-11-24-pitch-d-term-from-the-gyro.md)

## The change

`rocko_env/rocko_env/bringup/config/rocko_controllers.yaml`, `pitch_pid_controller`, line 29:

```yaml
    reference_and_state_interfaces: ["pitch"]
```

becomes

```yaml
    # Finding H-6: with the second entry the D term is d * (pitch_rate reference - pitch_rate), from the
    # gyro, instead of d * (change in pitch error) / dt from the fused angle. The rate reference is the
    # gyro's reading at enable (finding L-8): the supervisor enables balance only while the robot is
    # nearly still (ENABLE_MAX_PITCH_RATE_DEG_S in supervisor/state_machine.py).
    reference_and_state_interfaces: ["pitch", "pitch_rate"]
```

Nothing else in the YAML changes: **no gain changes**. `d` keeps its value (0.1), its units (deg/s) and its sign, but
not exactly its meaning: today the D term differentiates the whole error, `d(pitch target)/dt - d(pitch)/dt`, and the
target (the velocity loop's output) changes every cycle; afterwards it is `rate reference (constant) - pitch_rate`, so
the D term no longer reacts to changes of the target lean (no "derivative kick" when the velocity loop moves the
target). That is probably better, but it is a behaviour change, which the test below looks at. After the edit the agent removes the matching entry from `INTENTIONAL_WIRING_DIFFERENCES`
in `sim/check_sim_config.py`, which leaves that dict empty (the sim and the robot are then wired alike, and the check
demands it), confirms `sim/check_sim_config.py` still passes, and reruns the ladder.

## Why

The pitch loop's D term today differentiates the BNO055's fused pitch angle: a filtered, delayed signal, so the
derivative arrives late and noisy ([H-6](../claude-analysis/balance-loop-findings.md)). The gyro measures the rate
directly, in the same 12-byte read as the angle, so it is the same age as the angle and not differentiated. The
simulation has run this wiring since 2026-11-02 (check `H6`), and the candidate gains of
[proposal 0008](0008-candidate-gains-and-tethered-test.md) assume it. Doing it as its own step, with today's gains,
separates "the D term's source changed" from "the gains changed".

### How the rate reference is handled (finding L-8)

`pid_controller` sets the `pitch_rate` reference once, to the measured rate when the controller activates, and nothing
writes it afterwards ([L-8](../claude-analysis/balance-loop-findings.md#l-8--pid_controller-sets-the-rate-reference-once-at-activation)).
Two ways that could go wrong, and what now prevents each:

1. **A NaN reference**, from activating before the IMU had a good reading. It would make `pid_controller` silently fall
   back to differentiating the angle. **Prevented by C-1** (2026-11-24): the IMU plugin's `on_configure` only succeeds
   after two good samples in a row, and after that it never reports NaN (a failed read holds the last good values)
   ([how the IMU is read](../architecture/imu-i2c-read.md)). The controllers activate long after the hardware is
   configured.
2. **A constant offset of `d x rate at enable`** for the whole run, if the robot was moving when it was enabled.
   **Bounded by a new supervisor rule** (written in this run, not gated: it adds a safeguard): balance mode enables only
   if the pitch rate is known and within `ENABLE_MAX_PITCH_RATE_DEG_S` (`supervisor/state_machine.py`, PROPOSED; its
   value is in the [next-steps](../next-steps.md) proposed-values table). The offset is then at most
   `d x that limit` in the pitch loop's output (wheel-speed units), small with today's `d = 0.1`. The bound is on the
   rate when the enable is requested (at most 0.2 s old); the reference is taken a moment later, at the controller
   switch. This holds **only under the supervisor** (`rocko.service`, or `pi/launch.sh robot`): a standalone
   `pi/launch.sh balance` activates the cascade when it is spawned, so the reference is the rate at spawn and nothing
   checks it. The test below uses the supervised stack, and the velocity loop's integral term trims a
   steady offset out. Every enable re-activates the controller, so the reference is taken afresh each time.

## Evidence

| Check | Tier | Result |
| --- | --- | --- |
| The sim with this exact wiring: the D term's derivative equals the exported `pitch_rate` every cycle, and the error derivative is its negative (rate reference 0) | 3 (`verify_sim.sh` check `H6`) | passing since 2026-11-02; last passed in the full ladder recorded in the [C-1 encoder journal](../journal/2026-11-24-c1-encoders-in-cpp.md#verification) (2026-11-24) |
| The robot exports `pitch_rate`, with the measured axis and sign (gyro X, sign -1), read in the same burst as the angle, 0 failures in about 30 000 reads | 4, measured | [gyro check](../journal/2026-11-23-gyro-check.md); [C-1 IMU](../journal/2026-11-24-c1-imu-i2c-and-zero-on-deactivate.md); [loop timing](../journal/2026-11-24-loop-timing-on-the-pi.md) |
| The supervisor refuses a balance enable with an unknown, non-finite or too-large pitch rate | host test `tools/supervisor_state_machine_test.py` | pass; red with the rule removed |
| This YAML change on the robot | **none** | not done: it is the change being proposed |

What no tier shows: whether the gyro's D term balances better or worse than today's on the real robot. That is the
tethered test below, and nothing here claims it.

## Risk

- A wrong gyro sign would make the D term push the wrong way. The sign was measured on the robot
  (2026-11-23, rate against d(pitch)/dt correlation 0.99); step 2 below re-checks it in the controller itself.
- The gyro is noisier than a filtered angle at rest; `d = 0.1` times the gyro's noise (about 1/16 deg/s resolution) is
  small. A larger `d` later (proposal 0008) would need a look at that noise.
- Enabling now needs the robot held still. If the limit is too tight to enable by hand, the supervisor says so
  ("pitch rate ... deg/s; hold the robot still"); raise the constant, do not remove the rule.

## The test, for a human

On the stand first, then tethered. Record the commit, battery voltage and surface for each run, and use
`record:=true` or the dashboard's Record ([record a run](../guides/record-a-run.md)).

1. Apply the edit, then on the Pi: `cd ~/ROCKO-env && git pull && ./build_ros.sh`, and
   `sudo systemctl restart rocko.service`.
2. **Stand, wheels off the ground, balance mode** (LT+RT+RB while disabled). Hold the robot upright and still and
   enable (LB+RB); if the supervisor refuses with "pitch rate ...", hold it stiller. (The controller only publishes
   its state while active, that is, while enabled.) In `pi/rocko_shell.sh`:
   `ros2 topic echo /pitch_pid_controller/controller_state` (the field names are `pid_controller`'s; not tried on the
   robot). Tilt the robot a few degrees forward and back by hand. Success: `feedback_dot` follows the pitch rate
   (positive while pitch increases) and `error_dot` is its negative; the wheels turn the way that would roll under the
   lean, as before the change, and react sooner to a quick tilt than to a slow one. Disable (LB+RB).
   **Abort** (LB+RB, or cut power) if a wheel runs away, turns the wrong way, or the wheels buzz or oscillate at rest.
3. *(merged into step 2)*
4. **Tethered on the floor:** [proposal 0008](0008-candidate-gains-and-tethered-test.md), section "C. Tether, wheels
   on the floor", with today's gains and with that section's abort criteria. Compare with a run before the
   change if one was recorded.

## Outcome

(Human fills in: date, who, which steps, accepted or declined, where the logs are.)
