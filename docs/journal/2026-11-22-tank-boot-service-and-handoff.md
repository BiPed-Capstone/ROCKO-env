# Tank mode starts at boot; the 160 m/s² result; handoff to balance mode

- **Date / commit:** 2026-11-22, built on `b77df84` (added by the commit that adds the service)
- **Branch:** `ai-forward`
- **Environment:** macOS laptop; the dev container; the Pi over SSH (key login), with the owner present

This entry closes the 2026-11-22 session. A new session working on balance mode starts with the HANDOFF box at the top
of [next-steps.md](../next-steps.md) and the last section here.

## Goal and success criteria

The owner asked: "make tank mode the default mode that starts when the robot boots", then to clean up and document what
a new session needs for balance mode.

1. One command installs a systemd service that runs `pi/launch.sh tank` at boot, stops it the way Ctrl-C does, and does
   not restart a crashing launch forever.
2. While it runs, no second launch can start by accident (a balancing launch would fight it for the motors).
3. Installed and running on the robot; a reboot test is the owner's.
4. The handoff names every way the current robot set-up affects balance work.

## What changed

| File | What and why |
| --- | --- |
| `pi/rocko-tank.service` (new) | Runs `pi/launch.sh tank log_root:=<your ~/rocko_logs>` as root; `HOME=/root` (systemd gives none; ROS logging needs one); `ROCKO_BOOT_SERVICE=1` (lets the launch guard through); `KillSignal=SIGINT` and `TimeoutStopSec=30` (a ros2 launch shutdown; whether the motors are left at 0 % is finding H-7, unverified); `Restart=on-failure` (the launch failing, not a node inside it), at most 3 starts in 120 s (`StartLimit*` in `[Unit]`) |
| `pi/install_boot_service.sh` (renamed from `install_dashboard_service.sh`) | Now `dashboard` or `tank`, then `install`, `status`, `remove`, `print` |
| `pi/launch.sh` | Refuses (exit 2) to start while `rocko-tank.service` is active, unless it is that service; says how to stop it |
| `tools/pi_scripts_test.sh` | Both rendered units; an unknown name refused; the guard refuses and lets the service through (22 checks; each guard check red on its deliberate break) |
| Docs | USB guide "Start tank mode at boot"; dashboard guide and run-on-the-pi table renamed; the bench-test guide says to stop the service; next-steps row 4.12 and the new HANDOFF box; proposal 0015's 160 m/s² robot result |

## The 160 m/s² result (robot, on blocks)

Owner-run, commit `b77df84`, `max_speed:=0.4 max_turn:=1.0`, battery voltage not recorded. One full forward to full
reverse: the command reached -6 m/s within one 0.1 s sample; the wheel (left encoder position) crossed zero at about
0.05 s and reached reverse speed by about 0.2 s (it was about 0.6 s at 30 m/s² and 1.1 s at 10 m/s²). What remains is the
motors' own response. Recorded in [proposal 0015](../proposals/0015-tank-mode-faster-reversal.md); the floor test is
still to do.

## Verification

| What | Command | Result |
| --- | --- | --- |
| host | `bash tools/pi_scripts_test.sh` | 22/22; with the guard disabled, 21/22 (`launch.sh balance while rocko-tank.service runs` FAIL); with the service exemption removed, 21/22 (`launch.sh tank from the service itself` FAIL) |
| Pi | install and status | filled in after the install |

## Review rounds

| Reviewer | Findings | Fixed / declined (why) |
| --- | --- | --- |
| `ros2-control-reviewer` | **High:** `StartLimitIntervalSec` under `[Service]` is ignored (only `[Unit]` takes it), so the crash limit would not work and a failing launch could restart forever. Medium: `Restart=on-failure` does not cover a node crashing inside a running launch; "motors to 0 %" on stop is unverified (H-7). Low: `~/rocko_logs` is made only by the installer. Clean: the SIGINT path through `exec`, the guard, the environment, boot order, no motor command at boot | Fixed: both `StartLimit*` keys moved to `[Unit]` (checked on the Pi with `systemd-analyze verify`, below); the unit, guide and this entry now say what the restart covers and that the stop's 0 % is H-7, open. Declined: making the launch exit when `ros2_control_node` dies (a launch-file design change for later). The log folder: made by the installer, which is the documented way to install |
| `explainability-reviewer` | A comment that called `HOME=/root` "the user's"; reasons for `TimeoutStopSec`, `RestartSec`; the fake `systemctl` in the test; why `print` may run under sudo | All fixed |
| `docs-reviewer` | **High:** the USB guide still said the ramp was "three times steeper"; README and run-on-the-pi did not warn that a manual launch is refused after a boot; the handoff lacked the temporary stop, check and restore commands, the dashboard stop, and what the dead band means for balancing. Low: the bench guide's check step; 0015's log location | All fixed |

## Handoff: what a balance-mode session needs to know

**The robot as left:** the Pi runs the latest `ai-forward`. Two systemd services start at boot: `rocko-tank.service`
(tank mode, the 8BitDo drives, hold LB) and `rocko-dashboard.service` (the page on port 8080). The controller is an 8BitDo
Ultimate 2C on USB. `ros-jazzy-joy` is installed. `sudo` asks for no password on this Pi.

**Before a balancing launch:** the exact commands (stop, check, restore, remove) are in the HANDOFF box of
[next-steps.md](../next-steps.md): stop the tank service (`pi/launch.sh` refuses otherwise) and, until its CPU cost is
measured, the dashboard service. Both come back at boot unless removed.

**Open items that bear on balancing** (all in [next-steps.md](../next-steps.md)):

- Row 4.10: `QuadEncoders.py` answers both wheels with the **left** encoder's position, divides by a fixed 0.01 s and is
  called twice per control cycle. The balance velocity loop uses that velocity: fix or measure before row 2.7.
- Row 4.8: balancing has **no driving input**. The owner wants it decoupled from tank mode, sharing only
  `rocko_env/teleop/gamepad.py`. Joystick.py's TCP server is still there with no sender.
- Rows 4.6 / proposal 0013: the motors do not turn below 21 to 28 % duty; a balance loop's small corrections around zero
  will feel that dead band.
- The proposal 0008 queue, unchanged (the HANDOFF box lists it in order).

**How this session worked with the robot** (only when the owner asks and is at the robot; CLAUDE.md section 3):

- `ssh -o BatchMode=yes rocko '...'` (key login; BatchMode means it can never wait at a password prompt). Robot commands
  through `pi/rocko_run.sh` / `pi/launch.sh`, as on the Pi.
- A launch left running from SSH: `ssh -f rocko 'cd ~/ROCKO-env && nohup setsid pi/launch.sh tank ... > /tmp/x.log 2>&1 < /dev/null &'`;
  stop it with `sudo kill -INT` on the `ros2 launch` PID.
- Measurements: `ros2 topic echo` of the topics into `/tmp` files for a fixed time (`timeout 240`), then a short Python
  script parsing the YAML, run on the Pi (`ssh rocko python3 - < script.py`). Use encoder **position**, not
  `QuadEncoders.py`'s filtered velocity, for timing.
- Pitfalls met: a Pi build that switches from copies to `--symlink-install` fails (`existing path cannot be removed`):
  `rm -rf build install log && ./build_ros.sh`. The laptop's shell is zsh (no word-splitting of `$VAR` commands) and
  macOS has no `timeout`. A launch backgrounded inside a script ignores SIGINT (`setsid env --default-signal=INT`).
  `pkill -f pattern` inside `bash -c` can match itself (write the pattern as `[p]attern`).

**Small tidy-up for the owner:** the comment above the tank YAML's acceleration limits still says 30 m/s² (the values are
160); the file is guarded, so only the owner can reword it.
