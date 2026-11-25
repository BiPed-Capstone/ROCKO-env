# Proposal 0017 — Clamp the velocity loop's target lean to ±5 degrees

- **Status:** Open
- **Gate:** hardware gains and limits (`i_clamp_*`, `u_clamp_*` in `rocko_controllers.yaml`; the hook guards them)
- **Raised:** 2026-11-25, built on `2772ac5`, run journal:
  [2026-11-25 level pitch offset and lean clamp](../journal/2026-11-25-level-pitch-offset-and-lean-clamp.md)

## The change

`rocko_env/rocko_env/bringup/config/rocko_controllers.yaml`, `velocity_pid_controller`, the two `gains` lines:

```yaml
      pitch_pid_controller/left_wheel_joint:   {p: -2.0, i: -5.0, d: -0.2, i_clamp_max: 150.0, i_clamp_min: -150.0}
      pitch_pid_controller/right_wheel_joint:   {p: -2.0, i: -5.0, d: -0.2, i_clamp_max: 150.0, i_clamp_min: -150.0}
```

become

```yaml
      # The output is the TARGET LEAN [deg] handed to the pitch loop. Clamped to +-5 deg (proposal 0017): a target
      # past the 10 deg re-arm band is never a lean to balance at, and the old +-150 let the integral term wind the
      # target up until it drowned out the real lean (owner's floor test, 2026-11-25). i_clamp limits the integral
      # term's share to the same +-5 deg (legacy anti-windup, the current default).
      pitch_pid_controller/left_wheel_joint:   {p: -2.0, i: -5.0, d: -0.2, i_clamp_max: 5.0, i_clamp_min: -5.0, u_clamp_max: 5.0, u_clamp_min: -5.0}
      pitch_pid_controller/right_wheel_joint:   {p: -2.0, i: -5.0, d: -0.2, i_clamp_max: 5.0, i_clamp_min: -5.0, u_clamp_max: 5.0, u_clamp_min: -5.0}
```

Replace the two lines; the comment already above them stays, merged with the new one. `p`, `i` and `d` are
unchanged. After the edit the agent puts the same four clamp values in `rocko_sim_controllers.yaml`
(`sim/check_sim_config.py` requires every non-tuned gain key to match) and updates that file's comment saying a 150 deg
clamp "never limits anything that matters", reruns the whole ladder, and reports whether any simulation scenario now
fails because of the clamp (the sim's velocity loop has `i = 0`, so only `u_clamp` can matter there).

## Why

On 2026-11-25 the owner enabled balance on a flat floor and leaned the robot by hand. Whichever way it leaned, the
wheels spun the same way, until the tipover cutoff tripped (twice, in both directions). The service log has the trips;
there was no recording, so what follows is the reasoning, not a measurement:

- The velocity loop turns wheel speed into a target lean. Its integral term (`i = -5`) adds 5 deg of target lean per
  second at 1 m/s of wheel speed, up to `i_clamp` ±150 deg, which is no limit at all.
- The pitch loop (`pitch_pid_controller`, `p = 10`) commands about `10 x (target lean - pitch)`. With the wheels spinning, the target keeps growing, which
  spins them faster. On a balancing robot the body would lean and slow them; held by hand, or falling, it cannot. So
  within seconds `10 x target` is far larger than anything the real lean contributes: one direction, whatever the
  lean.
- Which direction it picks would depend on the first error (reasoning, not seen): until 2026-11-25 the robot
  standing level read +10.1 deg (the pitch offset was wrong; it is fixed in the same run), so the first command would
  have had the same sign every time.

A ±5 deg clamp keeps the target lean inside what a balancing robot ever needs (the re-arm band is 10 deg, the cutoff 35)
and caps the integral's share at the same 5 deg. With the target at most 5 deg, the pitch loop's P term is at most
`10 x (5 + |pitch|)`, so beyond 5 deg the real lean decides the direction. How `u_clamp` and `legacy` anti-windup act is
taken from the parameter descriptions in the installed `pid_controller_parameters.hpp` and `control_toolbox`'s
`pid.hpp`; it was not read from their source or tested.

## Evidence

| Check | Tier | Result |
| --- | --- | --- |
| The keys exist in this `pid_controller` (4.42.1, Jazzy, `control_toolbox` 4.11.1) | read from the installed `pid_controller_parameters.hpp` in the dev container | `u_clamp_max/min` (default ±infinity), `i_clamp_max/min`, `antiwindup_strategy` (default `legacy`) |
| The owner's test: one-direction spin in both lean directions, cutoff trips both ways | 4, the owner, 2026-11-25 | service log only (two trips at 35 deg); **no recording** |
| The simulation's tuned velocity loop uses `i = 0` | `rocko_sim_controllers.yaml` | so the sim never showed this windup; the clamp's effect there is checked after acceptance |
| That the clamp fixes the one-direction spin on the robot | **none** | not run: it is the change being proposed |

## Risk

- 5 deg might be too little lean for a quick start or stop. A balancing robot this size needs only a few degrees to
  accelerate, so it should not bind in normal driving; if a tethered run shows the target sitting at ±5, raise it.
- With `legacy` anti-windup `i_clamp` is described as bounding the integral (not tested here), so the steady lean the integral can hold
  against a centre-of-mass offset is at most 5 deg (the offset just measured is about 0 deg at level).
- This does not tune anything: the velocity gains are still the untuned starting values (proposal 0008 re-derives
  them; the sim's candidate has `i = 0`).

## The test, for a human

Wheels off the ground first, recorded ([record a run](../guides/record-a-run.md): the dashboard's Record, or
`record:=true`), battery voltage and surface noted.

1. Apply the edit; on the Pi, as the `rocko` user, `cd ~/ROCKO-env && git pull && ./build_ros.sh`, then
   `sudo systemctl restart rocko.service` ([run on the Pi](../guides/run-on-the-pi.md)).
   **Before the first enable**, hold the robot at the point where it balances by itself (centre of mass over the
   axle) and read the pitch on the dashboard: since the 2026-11-25 offset, level reads 0, but the balance point may
   not. If it reads near 10 deg either way, stop: the enable band and this clamp both assume a balance point near 0.
2. **Stand, wheels off the ground:** balance mode, hold the robot level and still, enable (LB+RB). The wheels will turn
   (there is no floor to balance on); success is that they settle to a bounded speed instead of running up to full,
   and that leaning the robot past 5 deg either way reverses them. Disable after about 10 s.
3. **Floor, tethered:** the same as the owner's 2026-11-25 test. Success: the wheels follow the lean (lean forward,
   wheels drive forward) instead of always one way.
4. **Abort** (LB+RB or cut power) if a wheel runs away, the wheels drive against the lean, or anything buzzes.

## Outcome

(Human fills in: date, who, which steps, accepted or declined, where the logs are.)
