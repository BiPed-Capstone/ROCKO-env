# Helper scripts for running ROCKO on the Pi as root, and the Pi workspace name

- **Date / commit:** 2026-11-19, built on `5d1119c` (the commit that adds this file holds the change)
- **Branch:** `ai-forward`
- **Environment:** laptop (macOS, bash 3.2) and the dev container (Ubuntu, ROS Jazzy, bash 5.2). **Not the Pi.**

## Goal and success criteria

The first tank-mode run failed to show the controllers in a second terminal: `ros2 control list_controllers`
waited forever. Cause, found by the owner: the launch needs root (WiringPi) and the second terminal was an
ordinary user. The owner asked for helper scripts in the repo (their old ones were a few `source` lines
and `ROS_DOMAIN_ID=17`), and for the docs to use them, noting that the Pi's workspace is not `~/ros2_ws`.

1. One command launches tank or balance mode as root, in the environment, with arguments passed on.
2. One command opens a root shell in the same environment for the other terminals.
3. The scripts reproduce the old helper's environment (ROS, venv, workspace overlay, domain 17, cd).
4. Recordings still land where `tools/fetch_logs.sh` looks.
5. Every Pi-facing instruction in the docs uses the scripts, and none names `~/ros2_ws` for the Pi.
6. `build_ros.sh` works on the Pi, where the checkout is the workspace (no `src/`).

## Plan

`pi/` with four scripts; `tools/pi_scripts_test.sh` (stub `sudo`, `ros2` and ROS setup) wired into
`tools/run_ladder.sh`; `build_ros.sh` default workspace; docs sweep. Risk: nothing can be run on the Pi
from here, so the root and DDS behaviour is reasoned, not observed.

## What changed

- `pi/rocko_env.sh` (sourced), `pi/rocko_run.sh`, `pi/rocko_shell.sh`, `pi/launch.sh`: new.
  `launch.sh` also adds `log_root:=$HOME/rocko_logs` (the launch runs as root, so `~` would be `/root`).
  The run and launch scripts refuse to run under `sudo`, where `$HOME` is `/root`.
- `build_ros.sh`: the workspace defaults to the folder the script is in (was `~/ros2_ws`); `rosdep` looks
  at `src/` if it exists, otherwise at the two package folders. The container is unchanged (compose sets `ROS2_WS`).
- `tools/pi_scripts_test.sh`, `tools/run_ladder.sh` (one added step), `tools/fetch_logs.sh` (comment: `sudo rm`).
- Docs: new [run-on-the-pi.md](../guides/run-on-the-pi.md); README, the guides, proposals 0006, 0008, 0010
  and 0012, `sim-plant.md`, the ladder, `CLAUDE.md` section 8 and the `build-verify` skill now use the scripts.
  `verify_*.sh` still default to `$HOME/ros2_ws`, but only the container runs them and compose sets
  `ROS2_WS`; left alone.

## Verification

| Tier | Command | Result |
| --- | --- | --- |
| 0 static | `tools/run_ladder.sh --no-sim` | PASS: `verify_urdf.sh`, hook tests, sim config drift, doc links, every host test |
| 0 host test | `bash tools/pi_scripts_test.sh` (macOS bash 3.2 and container bash 5.2) | 16 passed, 0 failed |
| made it fail | seven deliberate breaks (sudo dropped; tank mapped to the balance file; domain not exported; `build_ros.sh` back to `~/ros2_ws`; venv not handed to root; `log_root` default dropped; sudo guard removed) | each went red, then restored. A first version of the test used a fake `sudo` that kept the environment, so dropping the venv hand-over would not have failed: the explainability review caught it, the fake now resets the environment like the real one |
| 1 build | `build_ros.sh` in the container (via the ladder) | PASS, 0 compiler warnings (an incremental build, 3 s) |
| 2 mock | `verify_mock.sh`, `verify_mock.sh tank` | PASS |
| 3 sim | | not run (`--no-sim`; nothing here touches the model) |
| 4 robot | | **not run.** The scripts were never run on the Pi |

## Review rounds

| Round | Reviewer | Findings | Fixed / declined (why) |
| --- | --- | --- | --- |
| 1 | `explainability-reviewer` | 8: the test's fake sudo kept the environment (High); root-owned `~/rocko_logs`; `sudo pi/launch.sh` loses the venv; an array idiom only old bash needs; the `env` block copied twice; an unproven symptom stated as fact; a line-number `usage`; an undocumented `ROCKO_ROS_SETUP` | All fixed. `rocko_shell.sh` now calls `rocko_run.sh`; `launch.sh` makes `~/rocko_logs` as you |
| 1 | `docs-reviewer` | 9: a README line still said `~/ros2_ws`; bare `source` / `ros2 launch` in proposals 0006, 0008, 0010, 0012, `sim-plant.md`, two guides; no journal; the check count copied to three pages; a wrong bash claim; a self-contradicting table row; `fetch_logs.sh` `rm`; an overlong line | All fixed except the first finding below |
| 1 | `docs-reviewer` (finding 1) | The owner's uncommitted edit to `rocko_tank_mode.launch.py` turns the dashboard on by default and reformats the file | **Not touched, not committed**: the owner's change. Reported to the owner |

## Not verified: needs a human

On the Pi, from `~/ROCKO-env`, with the wheels off the ground:

1. `pi/launch.sh tank` asks for the `sudo` password and starts without a WiringPi error.
2. In a second terminal, `pi/rocko_shell.sh`, then `ros2 control list_controllers` lists `joint_state_broadcaster`
   and `diffbot_base_controller` as `active` (this is step 1 of the
   [tank-mode bench test](../guides/bench-test-tank-mode.md)).
3. The venv at `~/.venv` is found (no `rocko_env: note: no Python venv` line).
4. `ssh pi 'pi/launch.sh tank'` without `-t`: does `sudo` ask for a password? (guessed in the guide.)
5. `pi/launch.sh tank record:=true`: the run folder appears in `~/rocko_logs` (root-owned), and
   `tools/fetch_logs.sh` finds it.

Whether a non-root `ros2` command can see a root launch is **assumed**, from the owner's report. The
scripts avoid the question by running every `ros2` command as root.

## Reproduce

```bash
./check_container_mount.sh
tools/run_ladder.sh --no-sim
bash tools/pi_scripts_test.sh
```

## Later the same day: tank mode starts the dashboard by default

The owner's edit to `rocko_tank_mode.launch.py` (kept at their request) changes `dashboard_arguments(default=...)`
from `"false"` to `"true"`; the rest of that diff is re-wrapping (checked: the Python syntax trees of the old and new
file differ only in that one string). `rocko.launch.py` stays `"false"`. The comment in the launch file, the
`launch_dashboard.py` and `Dashboard.py` docstrings, the README, `use-the-dashboard.md` and `next-steps.md` row 4.3
now say so.

| Tier | Command | Result |
| --- | --- | --- |
| 0 | `verify_urdf.sh` | PASS |
| 2 | `verify_mock.sh tank` (starts the dashboard beside the controllers) | PASS |
| review | `ros2-control-reviewer` | no behavior finding; three stale sentences, fixed |
| 4 robot | | not run. The dashboard's CPU cost on the Pi is unmeasured, and the page has no login |

`joystick/keyboard_teleop.py` has an uncommitted edit by the owner (`STEPS_TO_FULL` 5 to 10, plus re-wrapping). It is
**not** in this commit, and `drive-with-keyboard.md` still says five steps; update both together when it settles.
