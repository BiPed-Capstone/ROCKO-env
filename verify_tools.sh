#!/usr/bin/env bash
# Check the operator tools against the simulated robot: that record:=true makes a usable recording,
# that the web dashboard shows the robot's values and can record, and that the Foxglove bridge
# starts and the committed Foxglove layout names only signals that exist. Run INSIDE the dev container,
# after a Tier 1 build:
#   docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_tools.sh
#
# What it does: starts rocko_sim.launch.py with record:=true, lets it run RECORD_S seconds, stops it
# with SIGINT to the launch process alone, and then reads the run folder with sim/check_recording.py.
# PASS means: nothing the launch started outlived it; one run folder was made; its run.yaml names the commit, the launch file, "sim" and
# the config; the bag closed properly; and every topic in REQUIRED_TOPICS has messages in it.
# Then it starts the launch again with the dashboard and runs sim/check_dashboard.py (what that
# checks is in its header) and judges the recording the dashboard made the same way.
# It says NOTHING about the robot, or about what recording or the dashboard cost the Pi's CPU or SD
# card, or about how the page LOOKS (no browser here).
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
FOXGLOVE_PORT=8765     # foxglove_bridge's port (its launch file's default)
DASH_PORT=8080         # the dashboard's default port
DASH_START_S=7         # [s] waited before checking the dashboard: the cascade is up after about 3 s
DASH_RECORD_S=7        # [s] the recording started over HTTP runs this long. `ros2 bag record` needs about
                       #   3 s to find the topics (a 4 s recording held 122 state messages), and the
                       #   bag must reach MIN_STATE_MESSAGES
DASH_TOLERANCE_DEG=0.5 # [deg] dashboard pitch vs the robot's own at the same moment. The two are
                       #   sampled up to 10 ms apart and the start-up lean moves at a few deg/s
DASH_MIN_PITCH_DEG=1.0 # [deg] the history must show the start-up lean (the launch starts at 2 deg), or
                       #   comparing two flat lines at zero would prove nothing
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
# What the launches here start, as a pattern for pgrep: none of it may be running afterwards.
LAUNCHED="RecordRun.py|ros2 bag record|ros2_control_node|Dashboard.py|foxglove_bridge"

# start_launch <args...>: rocko_sim.launch.py in the background; sets launch_pid.
# env --default-signal=INT: a program started in the background by a script inherits "ignore SIGINT",
# and `ros2 launch` then never reacts to it (seen: still running 15 s after the signal). In a
# terminal SIGINT is not ignored; this puts the launch in that state.
start_launch() {
  echo "=== rocko_sim.launch.py $*"
  setsid env --default-signal=INT ros2 launch rocko_env rocko_sim.launch.py "$@" >>"$out/launch.log" 2>&1 &
  launch_pid=$!
}
# stop_launch: SIGINT to the LAUNCH PROCESS ONLY, not to its process group. Ctrl-C in a terminal
# signals every process in the group, which hides a recorder that the launch itself fails to stop; a
# launch ended by `kill`, by a service manager or by one of its processes dying signals only what
# it started. Then: nothing the launch started may outlive it (a recorder left running keeps
# writing until the disk guard). Exits 1 on either failure.
stop_launch() {
  kill -INT "$launch_pid" 2>/dev/null
  for _ in $(seq 1 "$STOP_WAIT_S"); do kill -0 "$launch_pid" 2>/dev/null || break; sleep 1; done
  if kill -0 "$launch_pid" 2>/dev/null; then
    kill -KILL -- "-$launch_pid" 2>/dev/null
    echo "FAIL: the launch did not shut down within $STOP_WAIT_S s of SIGINT (log: $out/launch.log)"; exit 1
  fi
  sleep 1
  if pgrep -f "$LAUNCHED" >/dev/null 2>&1; then
    echo "FAIL: processes outlived the launch:"
    pgrep -af "$LAUNCHED" | cut -c1-120
    pkill -INT -f "RecordRun.py" 2>/dev/null; sleep 3
    pkill -KILL -f "$LAUNCHED" 2>/dev/null
    echo "FAIL (tools). Kept as evidence (inside the dev container): $out"; exit 1
  fi
}

# --- 1. record:=true ---
start_launch record:=true record_label:=verify_tools battery_v:=12.3 surface:=simulated \
  log_root:="$out/logs" dashboard:=false
sleep "$RECORD_S"
stop_launch

# shellcheck disable=SC2086  # REQUIRED_TOPICS is a list of words on purpose
if python3 "$here/sim/check_recording.py" "$out/logs" --label verify_tools --hardware sim \
     --launch rocko_sim.launch.py --battery-v 12.3 --surface simulated \
     --min-state-messages "$MIN_STATE_MESSAGES" --require $REQUIRED_TOPICS; then
  echo "    record: ok"
else
  echo "FAIL (tools: record). Kept as evidence (inside the dev container): $out"
  grep -n "RecordRun" "$out/launch.log" | tail -5
  exit 1
fi

# --- 2. the dashboard: its values against the robot's, and Record/Stop over HTTP ---
# initial_pitch_deg:=2.0 with the scenarios' encoder sign: the robot starts leaning and stands up,
# so there is a pitch transient to compare (see DASH_MIN_PITCH_DEG).
# foxglove:=true: the same launch also serves part 3.
start_launch dashboard:=true dashboard_port:="$DASH_PORT" log_root:="$out/dashlogs" \
  initial_pitch_deg:=2.0 encoder_sign:=-1 foxglove:=true
sleep "$DASH_START_S"
python3 "$here/sim/check_dashboard.py" --port "$DASH_PORT" --record-s "$DASH_RECORD_S" \
  --tolerance-deg "$DASH_TOLERANCE_DEG" --min-pitch-deg "$DASH_MIN_PITCH_DEG"
dash_status=$?
# --- 3. Foxglove: the bridge listens, and every message path in the layout is real ---
python3 "$here/sim/check_foxglove_layout.py" "$here/foxglove/rocko_balance_layout.json" --bridge-port "$FOXGLOVE_PORT"
fox_status=$?
stop_launch
if [ "$dash_status" -eq 2 ]; then echo "COULD NOT JUDGE (tools: dashboard). Kept: $out"; exit 2; fi
# shellcheck disable=SC2086
if [ "$fox_status" -ne 0 ]; then
  echo "FAIL (tools: foxglove). Kept as evidence (inside the dev container): $out"; exit 1
fi
if [ "$dash_status" -eq 0 ] && python3 "$here/sim/check_recording.py" "$out/dashlogs" --label dashboard \
     --hardware sim --launch rocko_sim.launch.py --battery-v 12.3 --surface simulated \
     --min-state-messages "$MIN_STATE_MESSAGES" --require $REQUIRED_TOPICS; then
  echo "PASS (tools: record dashboard foxglove). This is the SIMULATION: it shows the tools work, not what they cost on the Pi."
  rm -rf "$out"; exit 0
fi
echo "FAIL (tools: dashboard). Kept as evidence (inside the dev container): $out"
grep -n "rocko_dashboard\|Dashboard.py\|RecordRun" "$out/launch.log" | tail -8 | cut -c1-200
exit 1
