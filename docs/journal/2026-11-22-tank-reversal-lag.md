# First USB-controller drive on the robot, and why tank mode reverses slowly

- **Date / commit:** 2026-11-22, robot running `784022a`; the files this entry describes are added by the commit that
  adds it
- **Branch:** `ai-forward`
- **Environment:** the Pi (Ubuntu 24.04, Jazzy, `diff_drive_controller` 4.43.1, `ros-jazzy-joy` 3.3.0, an 8BitDo
  Ultimate 2C on USB), reached by the agent over SSH (key login) **at the owner's request, with the owner at the robot,
  robot on blocks**; the dev container for the mock measurements.

## Goal and success criteria

The owner asked, in order: (1) why tank mode did not drive after updating; (2) to check a stand test; (3) to
investigate why full forward to full reverse is slow ("wind down, then wind up"), control loop or hardware.

1. The robot runs `784022a`'s tank launch (both controller nodes up). Checked by its log.
2. The stand test's commands match the sticks and the LB rule. Checked by a capture.
3. The cause of the slow reversal is identified with a measurement that separates the controller's command from
   the wheels' motion; any fix that touches a guarded key is a proposal, with its effect measured.

## 1. Why it did not drive: a failed build on the Pi

The launch log showed `Joystick.py` and neither new node. The checkout was at `784022a` and clean; the **install** was
the Nov 18 one: plain copies, no `TankTeleop.py`. The owner's rebuild that day had failed:
`rocko_interfaces` stopped with `failed to create symbolic link ... existing path cannot be removed: Is a directory`
(the Nov 18 build had made copies; `build_ros.sh` now uses `--symlink-install`) and colcon skipped `rocko_env`
(`JobSkipped`). `build_ros.sh` printed `ERROR: colcon build failed`, easy to miss. Fix, run by the agent on the Pi:
`rm -rf build install log && ./build_ros.sh`: both packages built (`Done!`, exit 0); the installed launch file and
`TankTeleop.py` became links to the source. Then `pi/launch.sh tank max_speed:=0.4 max_turn:=0.3` (sudo needs no
password on this Pi): `Controller Found: device_id=0, device_name=8BitDo Ultimate 2C Wireless Controller`,
`controller heard on /joy`; `/joy` at rest had 6 axes near 0 and 21 buttons at 0 (the layout `gamepad.py` expects).

## 2. The stand test (owner drove; agent captured)

Owner's report: driving "worked completely and was good"; **the unplug worked too**; a little sluggish. From a capture
of `/joy`, `/tank_teleop/enabled` and `cmd_vel` (on the Pi, `ros2 topic echo` to files):

- Stick direction and command sign agreed in 1394 of 1394 one-stick samples (left up = forward; right stick right =
  negative `angular.z`, a right turn). LB pressed twice; no non-zero command while LB was released; a released LB gave a
  zero command after 19 ms and 11 ms. `/joy` arrived at about 30 Hz with no gap over 0.10 s during the test (one 0.25 s
  gap 5 s after the capture processes started; cause not shown).
- **The suggested caps made turning hum:** `max_speed` and `max_turn` multiply, so a full turn was 0.4 x 0.3 = 12 % of
  full wheel speed (`angular.z` 11.25 rad/s; each wheel 1.8 m/s, 14.2 rad/s), about 14 % duty, below the 21 to 28 % breakaway. The guide now says `max_turn:=1.0`
  and why.
- Not exercised in the capture: moving the sticks without LB (they read exactly 0 whenever LB was released).

## 3. The slow reversal

**Measured on the robot** (second capture: `cmd_vel`, `cmd_vel_out`, `/dynamic_joint_states`; the wheel speed is
differentiated from the left encoder's raw position over ±0.1 s, because the node's own velocity is low-pass filtered).
One full forward to full reverse, every 0.1 s:

| t after the flip [s] | 0.0 | 0.2 | 0.4 | 0.5 | 0.6 | 0.8 | 0.9 | 1.0 | 1.1 | 1.2 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| stick target [m/s] | -3.81 | -6.00 | -6.00 | -6.00 | -6.00 | -6.00 | -6.00 | -6.00 | -6.00 | -6.00 |
| limited command [m/s] | +5.60 | +3.61 | +1.61 | +0.60 | -0.39 | -2.40 | -3.39 | -4.39 | -5.40 | -6.00 |
| wheel [encoder units/s] | +1.19 | +0.84 | +0.46 | +0.10 | -0.04 | -0.14 | -0.35 | -0.81 | -1.13 | -1.15 |

(Encoder units are not calibrated to m/s; the timing is what matters. Before the flip the wheel ran at +1.3, after at -1.2.)

- **Cause: the controller's ramp.** The limited command moves exactly 1.0 m/s per 0.1 s, the YAML's
  `linear.x.max_acceleration: 10.0`: 1.2 s from +6 to -6 m/s, 3.0 s at full speed.
- **The wheels track the ramp** (zero crossing 0.55 s vs the command's 0.56 s, full reverse by 1.1 s), on the way down
  too, so neither inertia nor the driver adds a lag of note. **Not hardware.**
- **A dead spot of about 0.4 s** (0.5 to 0.9 s) while the command crosses the breakaway band (proposal 0013).

**Measured on the mock** (container `diff_drive_controller` 4.42.1; the Pi has 4.43.1) with a new script, `tools/measure_reversal_ramp.sh` (container; launches tank mode on the
mock, sets proposed limits with `ros2 param set`, reconfigures the controller, steps the command +V to -V and times
`cmd_vel_out`). As is: zero 0.60 s, 90 % 1.14 s (the robot: 0.56 s, 1.09 s). Findings on the way:

| Setting (linear.x) | 6 m/s: zero / 90 % | 15 m/s: zero / 90 % |
| --- | --- | --- |
| as is (`max_acceleration` 10) | 0.60 / 1.14 s | 1.52 / 2.86 s |
| `min_acceleration` -30 | 0.60 / 1.14 s (**no effect**) | 1.52 / 2.86 s (**no effect**) |
| `max_deceleration` -30, `max_deceleration_reverse` 30 | 0.22 / 0.72 s | 0.52 / 1.82 s |
| `max_acceleration` 30, `min_acceleration` -30 | 0.20 / 0.40 s | 0.50 / 0.96 s |
| proposal 0015 option B as written: `max_acceleration` 30, `max_deceleration` -30, `max_deceleration_reverse` 30 | 0.22 / 0.38 s | 0.50 / 0.96 s |

- `diff_drive_controller` reads its limits **only when configured**: a runtime `ros2 param set` alone changed nothing,
  so the script reconfigures the controller.
- The YAML's `min_acceleration: -10.0` has no effect in 4.43.1.

The fix touches guarded keys: [proposal 0015](../proposals/0015-tank-mode-faster-reversal.md) (options A and B, recommended B).

**Also found:** `QuadEncoders.py` answers both motor plugins with the **left** encoder's position and divides by a fixed
0.01 s although it is called twice per cycle at 50 Hz (tank) or 100 Hz (balancing): next-steps row 4.10. Tank mode
drives open loop, so this does not cause the lag; balancing's velocity loop does use that velocity.

## Verification

| What | Command | Result |
| --- | --- | --- |
| Pi build | `rm -rf build install log && ./build_ros.sh` (agent, over SSH) | pass, `Done!`, exit 0 |
| Robot launch | `pi/launch.sh tank max_speed:=0.4 max_turn:=0.3` | both new nodes up, controller found |
| Stand test (Tier 4, owner) | capture + `/tmp` analysis scripts (not committed) | as section 2 |
| Reversal (Tier 4, owner) | capture + analysis (not committed) | as section 3 |
| New tool | `tools/measure_reversal_ramp.sh` (container, mock) | reproduces the robot's ramp; first version hung at exit (a background launch ignores SIGINT; fixed with `setsid env --default-signal=INT`) and left a process behind (cleaned up); final version leaves none |
| Docs | `python3 -I tools/check_doc_links.py` | 0 broken |

**Reproducibility (CLAUDE.md section 9):** commit `784022a`; `rocko_tank_mode.launch.py` with
`rocko_tank_mode_controllers.yaml`, `max_speed:=0.4 max_turn:=0.3`; surface: on blocks; **battery voltage not
recorded**; the captures were in the Pi's `/tmp` (not kept; they are lost at the next reboot).

## Review rounds

One round for this entry and the same day's [dashboard boot service](2026-11-22-dashboard-boot-service.md), reviewed together.

| Reviewer | Findings | Fixed / declined (why) |
| --- | --- | --- |
| `ros2-control-reviewer` | **High:** the boot service had no `HOME`, so ROS logging would fail at start-up and the service would restart every 10 s (reproduced in the container with `HOME` unset). Medium: the always-on dashboard also runs during balancing, whose CPU cost was never measured. Low: `Dashboard.py`'s clean exit on a taken port could hide a stale dashboard in `verify_tools.sh`; the "no effect" claim was labelled 4.43.1 but measured on 4.42.1; the script's wait matched "inactive"; option B's measured YAML differed from the proposed one. Confirmed from the installed source: keys, signs, `min_acceleration` deprecated and unused, option A's and B's timings | Fixed: `Environment=HOME=/root` and a restart count in `status`; the guide says to remove the service before balancing tests until the cost is measured; the version labels; `grep -w` plus a timeout error; option B re-measured exactly as proposed (0.38 s / 0.96 s). Declined: the taken-port exit in `verify_tools.sh` (that script already fails when any process outlives a launch, and `check_dashboard.py` requires the launch name `rocko_sim.launch.py` and `sim` in `/api/state`, which a stale tank or boot dashboard would not show) |
| `explainability-reviewer` | Why `setsid`/`--default-signal`, bare numbers in the script (startup wait, settle, window, send period), the default 6.0, the embedded probe, why `sed` templating, why `RestartSec=10`, which script reads the unit's variables, the "?" in the log line, a noise variable in the test | All fixed (named constants, comments, the log reads "usually the boot service's dashboard"). The probe stays embedded, with a comment saying what it does |
| `docs-reviewer` | Numbers repeated in four pages (CLAUDE.md section 7); a pointer to a review section that did not exist; option B's YAML not the one measured; a unit label (11.25 rad/s is `angular.z`, not wheel speed); an unmeasured 0.4 s filter lag in row 4.10; the stand-test result stated without "battery voltage not recorded" and "capture not kept" | All fixed: proposal 0015 holds the numbers, the guide and row 4.9 link to it; this section; option B measured; the label; row 4.10 says the scale and lag are unknown until measured; the guide's caveats |

## Not verified — needs a human

- Proposal 0015 on the robot (its test is in the proposal), and on the floor.
- Motor current and temperature during reversals: never measured.
- The encoder service's velocity scale and timing (row 4.10).

Stopped because: the cause is measured and the fix is a guarded decision for the owner.
