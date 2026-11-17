#!/usr/bin/env bash
# Check the operator tools against the simulated robot: today, that record:=true makes a usable
# recording. Run INSIDE the dev container, after a Tier 1 build:
#   docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_tools.sh
#
# What it does: starts rocko_sim.launch.py with record:=true, lets it run RECORD_S seconds, stops it
# with SIGINT to the launch process alone, and then reads the run folder with sim/check_recording.py.
# PASS means: nothing the launch started outlived it; one run folder was made; its run.yaml names the commit, the launch file, "sim" and
# the config; the bag closed properly; and every topic in REQUIRED_TOPICS has messages in it.
# It says NOTHING about the robot, or about what recording costs the Pi's CPU or SD card.
#
# Exit status: 0 = pass, 1 = a check failed, 2 = could not run.
set -u
RECORD_S=8             # [s] recorded: long enough for the cascade to activate (about 3 s) and for a few
                       #   hundred 100 Hz samples after it
STOP_WAIT_S=30         # [s] how long the launch may take to shut down and close the bag
# Topics that MUST have messages in a simulated run with nobody driving. The recorder's own list
# (RECORDED_TOPICS in RecordRun.py) is longer: the rest need a gamepad, tank mode or a fault.
REQUIRED_TOPICS="/dynamic_joint_states /joint_states /pitch_pid_controller/controller_state
  /velocity_pid_controller/controller_state /velocity_pid_controller/reference /left_feedforward
  /right_feedforward /diff_drive_controller/command_timed_out /rosout /robot_description"
MIN_STATE_MESSAGES=200 # /dynamic_joint_states is 100 Hz: 200 messages is 2 s, well under RECORD_S minus start-up

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ws="${ROS2_WS:-/ros2_ws}"
set +u; source /opt/ros/jazzy/setup.bash; source "$ws/install/setup.bash" 2>/dev/null; set -u
if ! ros2 pkg prefix rocko_env >/dev/null 2>&1; then
  echo "ERROR: rocko_env is not built. Run build_ros.sh first." >&2; exit 2
fi
if pgrep -x ros2_control_no >/dev/null 2>&1 || pgrep -f ros2_control_node >/dev/null 2>&1; then
  echo "ERROR: a controller_manager is already running in this container; stop it first." >&2; exit 2
fi

out="$(mktemp -d)"
echo "=== record: rocko_sim.launch.py record:=true record_label:=verify_tools battery_v:=12.3 surface:=simulated log_root:=$out/logs"
# env --default-signal=INT: a program started in the background by a script inherits "ignore SIGINT",
# and `ros2 launch` then never reacts to it (seen: still running 15 s after the signal). In a
# terminal SIGINT is not ignored; this puts the launch in that state.
setsid env --default-signal=INT ros2 launch rocko_env rocko_sim.launch.py record:=true record_label:=verify_tools \
  battery_v:=12.3 surface:=simulated log_root:="$out/logs" >"$out/launch.log" 2>&1 &
launch_pid=$!
sleep "$RECORD_S"
# SIGINT to the LAUNCH PROCESS ONLY, not to its process group. Ctrl-C in a terminal signals every
# process in the group, which hides a recorder that the launch itself fails to stop; a launch ended
# by `kill`, by a service manager or by one of its processes dying signals only what it started.
kill -INT "$launch_pid" 2>/dev/null
for _ in $(seq 1 "$STOP_WAIT_S"); do kill -0 "$launch_pid" 2>/dev/null || break; sleep 1; done
if kill -0 "$launch_pid" 2>/dev/null; then
  kill -KILL -- "-$launch_pid" 2>/dev/null
  echo "FAIL: the launch did not shut down within $STOP_WAIT_S s of SIGINT (log: $out/launch.log)"; exit 1
fi
# Nothing the launch started may outlive it: a recorder left running keeps writing until the disk guard.
sleep 1
if pgrep -f "RecordRun.py|ros2 bag record|ros2_control_node" >/dev/null 2>&1; then
  echo "FAIL: processes outlived the launch:"; pgrep -af "RecordRun.py|ros2 bag record|ros2_control_node" | cut -c1-120
  pkill -INT -f "RecordRun.py" 2>/dev/null; sleep 3; pkill -KILL -f "RecordRun.py|ros2 bag record|ros2_control_node" 2>/dev/null
  echo "FAIL (tools: record). Kept as evidence (inside the dev container): $out"; exit 1
fi

# shellcheck disable=SC2086  # REQUIRED_TOPICS is a list of words on purpose
if python3 "$here/sim/check_recording.py" "$out/logs" --label verify_tools --hardware sim \
     --launch rocko_sim.launch.py --battery-v 12.3 --surface simulated \
     --min-state-messages "$MIN_STATE_MESSAGES" --require $REQUIRED_TOPICS; then
  echo "PASS (tools: record). This is the SIMULATION: it shows the recording works, not what it costs on the Pi."
  rm -rf "$out"; exit 0
fi
echo "FAIL (tools: record). Kept as evidence (inside the dev container): $out"
grep -n "RecordRun" "$out/launch.log" | tail -5
exit 1
