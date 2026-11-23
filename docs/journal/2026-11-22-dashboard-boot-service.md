# Start the web dashboard at boot with a systemd service

- **Date / commit:** 2026-11-22, built on `784022a` (added by the commit that adds the service)
- **Branch:** `ai-forward`
- **Environment:** macOS laptop; the dev container; the Pi over SSH (key login), at the owner's request

## Goal and success criteria

The owner asked (2026-11-22): "make it so the web dashboard starts on boot via a linux service".

1. One command on the Pi installs a systemd service that starts `Dashboard.py` at boot, as root in the robot's
   `ROS_DOMAIN_ID`, with the user's venv and log folder (root's HOME is `/root`, so they are written in).
2. It coexists with launches that start their own dashboard (tank mode does): the second one on port 8080 steps aside
   cleanly instead of "process has died"; the service takes the port back when a launch's dashboard held it.
3. Status and removal are one command each; the guide says it is always on with no login.
4. Installed and running on the robot; a reboot test is the owner's call.

## What changed

| File | What and why |
| --- | --- |
| `pi/rocko-dashboard.service` (new) | The unit template: `ExecStart` through `pi/rocko_run.sh` (already root under systemd, so no sudo), `Restart=always` with `RestartSec=10`, `launch`/`hardware` parameters saying "unknown" (a boot service cannot know the launch) |
| `pi/install_dashboard_service.sh` (new) | `install` (render the template with this checkout, `ROCKO_VENV`, `~/rocko_logs`, `ROCKO_DOMAIN_ID`; enable and start), `status`, `remove`, `print`; refuses to run under sudo |
| `webserver/Dashboard.py` | On `EADDRINUSE` it logs why and exits 0 instead of raising |
| `tools/pi_scripts_test.sh` | Two checks: the rendered unit names the caller's paths and leaves no `@PLACEHOLDER@`; install under sudo is refused |
| Docs | use-the-dashboard.md "Start it at boot", run-on-the-pi.md's script table, next-steps row 4.11 |

## Verification

| What | Command | Result |
| --- | --- | --- |
| host | `bash tools/pi_scripts_test.sh` | 18/18; red (17/18) with the `@LOG_ROOT@` substitution removed |
| container | two `Dashboard.py` on port 8091 | the second exited **0** with `port 8091 is already in use ...`; the first still answered HTTP 200 |
| Pi, install (agent over SSH, owner's request) | `git pull` to `4fc28ff`, `./build_ros.sh`, `pi/install_dashboard_service.sh`, then `... status` | `Installed ...; enabled at boot; started`; status: `enabled`, `active`, **restarts 0**; `curl http://127.0.0.1:8080/` HTTP 200; `/api/state` says `launch: unknown (dashboard boot service)`, `hardware: unknown`, no controller data (no launch running) |
| Pi, with a launch | `pi/launch.sh tank max_speed:=0.4 max_turn:=1.0` while the service runs | the launch's `Dashboard.py` logged `port 8080 is already in use by another process ...` and `process has finished cleanly`; the service's dashboard then showed `/joy` 0.03 s old, `teleop_enabled` False, `joint_states` 0.02 s old; restarts still 0 |

## Review rounds

Reviewed together with the same day's reversal investigation; the table is in
[that journal](2026-11-22-tank-reversal-lag.md#review-rounds).

## Not verified — needs a human

- A reboot of the Pi with the service enabled (the real "on boot" test): not done yet; the owner decides when.
- Its CPU cost on the Pi, alone or beside the balance loop: not measured (the guide says to remove it before
  balancing tests until it is).
