#!/usr/bin/env bash
# Tier 3 check, criteria S1 and S2a: do the real controllers run against the SIMULATED robot
# (rocko_env/SimPlant), does the simulated pitch respond to the wheels, and does the simulated robot
# fall when its motors give no torque?
#
# Run inside the Docker dev container, after build_ros.sh. From the repo root on your laptop:
#   docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_sim.sh
#
# THE SIMULATOR IS A MODEL WITH ESTIMATED, UNMEASURED PARAMETERS (docs/architecture/sim-plant.md).
# A pass says the plumbing and the model behave as specified. It says NOTHING about whether the real
# robot balances, and nothing about whether any gain is good.
#
# Three launches of rocko_sim.launch.py teleop:=false, one after another. In each, sim/sim_probe.py
# records pitch from /dynamic_joint_states, a velocity reference of 0 is published so the whole
# cascade is running, and the wiring is checked (the four cascade controllers and
# joint_state_broadcaster active, the SimPlant component active, all 12 interfaces present, both
# wheel commands claimed):
#   S1 baseline   upright, no wheel command beyond the cascade's own: pitch must stay within
#                 S1_STILL_DEG of its start (the model's upright equilibrium is exact). A robot that
#                 was never released also stays still, so every run must show the robot was released
#                 at least RELEASED_MIN_S before the probe's last sample (below).
#   S1 command    the same, plus a feedforward of FEEDFORWARD_RAD_S on both wheels: pitch must
#                 move at least S1_CHANGE_DEG. Pitch changes BECAUSE the wheels moved.
#   S2a           disable_torque:=true, initial_pitch_deg:=2.0: |pitch| must pass TIPOVER_DEG within
#                 FALL_LIMIT_S of the moment it starts to move.
#
# Exit status:  0 = PASS   1 = FAIL (what was wrong is printed)
#               2 = could not run (no build, or a controller_manager is already running)
# On anything but PASS the run directory (launch logs, probe samples.csv and metrics.json for each
# run) is KEPT, and its path inside the container is printed: it is the evidence of the failure.
#
# No `set -e`: grep and timeout are EXPECTED to "fail" while we wait for things to come up.
set -uo pipefail

# --- Timing. Generous bounds, as in verify_mock.sh; bring-up time in the container is not measured.
STARTUP_TIMEOUT=45   # s to wait for every controller to become active
CALL_TIMEOUT=8       # s for one `ros2 control` call; without a timeout it can wait forever
GRACE=15             # s to let a launch shut down before forcing it
PROBE_SECONDS=8      # s of /dynamic_joint_states the probe records in each run (from its first
                     #   message; the robot is released a second or two later, when the cascade
                     #   activates, so at least ~5 s of each run is after release)
RELEASED_MIN_S=3     # s: the release must be logged at least this long before the probe's last sample, so
                     #   a pass is never a robot that stood still because it was never released

# --- Thresholds. Why each number (measured values are in the 2026-10-31 stage 3 journal):
FEEDFORWARD_RAD_S=5.0  # [rad/s] wheel command added by S1's command run: 6 % duty through the
                       #   write() copy, well clear of its 0.05 rad/s deadband, far from saturation
S1_STILL_DEG=0.01      # [deg] the baseline must stay closer than this to its start. The model's
                       #   upright equilibrium is exact (unit test #10), so the measured baseline is 0
S1_CHANGE_DEG=0.5      # [deg] the command run must move at least this much: 50x S1_STILL_DEG, and
                       #   far below what the command produces (several degrees, measured). A plant
                       #   whose pitch ignored the wheels would give exactly 0
TIPOVER_DEG=35         # [deg] the tipover angle of finding H-4
FALL_LIMIT_S=0.48      # [s] the model falls 2 -> 35 deg in 0.4273 s with no torque (unit test #6);
                       #   the probe sees it to within one 10 ms cycle. 0.48 adds ~40 ms for timing
                       #   jitter and stays below 0.50 s, the fall time with the motors merely
                       #   braked at 0 % duty, so "torque not disabled" cannot pass

WS="${ROS2_WS:-$HOME/ros2_ws}"
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ROS's setup scripts read variables that may be unset, so `set -u` is off while sourcing them.
set +u
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$WS/install/setup.bash" 2>/dev/null \
  || { echo "ERROR: no build in $WS/install. Run src/ROCKO-env/build_ros.sh first." >&2; exit 2; }
set -u

# Keep this run off other machines' ROS graphs, and refuse to run next to another controller
# manager: `ros2 control` talks to whichever one answers.
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
command -v pgrep >/dev/null || { echo "ERROR: pgrep (procps) is missing." >&2; exit 2; }
controller_manager_running() { pgrep -f ros2_control_node >/dev/null; }
if controller_manager_running; then
  echo "ERROR: a controller_manager (ros2_control_node) is already running. Stop it first." >&2
  exit 2
fi

controllers="joint_state_broadcaster left_add_feedforward_controller right_add_feedforward_controller pitch_pid_controller velocity_pid_controller"
states=""
for j in left_wheel_joint right_wheel_joint; do
  for s in position velocity yaw pitch roll; do states="$states $j/$s"; done
done
commands="left_wheel_joint/velocity right_wheel_joint/velocity"

out="$(mktemp -d)"
groups=()   # process-group ids started by this script; cleanup signals each whole group
# start_group <log> <command...>: run it in its own process group (setsid; see verify_mock.sh).
start_group() { local log="$1"; shift; setsid "$@" >"$log" 2>&1 & groups+=("$!"); }
stop_groups() {
  # SIGTERM first (a background job ignores SIGINT), then force, so this script never hangs.
  for g in "${groups[@]}"; do kill -TERM -- "-$g" 2>/dev/null; done
  for _ in $(seq 1 "$GRACE"); do
    alive=0; for g in "${groups[@]}"; do kill -0 "$g" 2>/dev/null && alive=1; done
    [ "$alive" -eq 0 ] && break; sleep 1
  done
  for g in "${groups[@]}"; do kill -KILL -- "-$g" 2>/dev/null; wait "$g" 2>/dev/null; done
  groups=()
  # The next run refuses to start next to a leftover controller manager, so wait for it to go.
  for _ in $(seq 1 "$GRACE"); do controller_manager_running || return 0; sleep 1; done
  return 1
}
passed=0
cleanup() {
  stop_groups
  if [ "$passed" -eq 1 ]; then
    rm -rf "$out"
  else
    echo "Run directory KEPT as evidence (inside the dev container): $out" >&2
  fi
}
trap cleanup EXIT

# has_interface <command|state> <name> [text the line must also contain]. As in verify_mock.sh:
# velocity is both a command and a state interface, so awk tracks which section a line is in.
has_interface() {
  awk -v sec="$1" -v name="$2" -v flag="${3:-}" '
    /^command interfaces/ { cur = "command"; next }
    /^state interfaces/   { cur = "state";   next }
    cur == sec && $1 == name && (flag == "" || index($0, flag)) { found = 1 }
    END { exit !found }' <<<"$ifaces"
}

# wiring_problems: look once; fills `problems` (empty = all is well).
wiring_problems() {
  problems=()
  ctl="$(timeout -k 2 "$CALL_TIMEOUT" ros2 control list_controllers 2>/dev/null)"
  ifaces="$(timeout -k 2 "$CALL_TIMEOUT" ros2 control list_hardware_interfaces 2>/dev/null)"
  hw="$(timeout -k 2 "$CALL_TIMEOUT" ros2 control list_hardware_components 2>/dev/null)"
  for c in $controllers; do
    grep -Eq "^$c[[:space:]].*[[:space:]]active([[:space:]]|$)" <<<"$ctl" \
      || problems+=("controller '$c' is not active")
  done
  grep -q "plugin name: rocko_env/SimPlant" <<<"$hw" \
    || problems+=("no hardware component uses the rocko_env/SimPlant plugin")
  grep -q "label=active" <<<"$hw" || problems+=("the SimPlant hardware component is not active")
  for s in $states; do
    has_interface state "$s" || problems+=("state interface '$s' is not exported")
  done
  for c in $commands; do
    has_interface command "$c" "[claimed]" || problems+=("command interface '$c' is not claimed")
  done
}

# released_problem <launch log> <metrics.json>: prints why there is no evidence that the robot was
# released at least RELEASED_MIN_S before the probe's LAST SAMPLE, or nothing if there is. The
# release time is the [INFO] [<epoch seconds>] stamp of SimPlant's log line; the last sample's time
# is last_stamp_s in metrics.json, from the same system clock (not a time taken by this script after
# the run, which would count the seconds spent in the wiring checks as released time).
released_problem() {
  local release last
  release="$(grep -m1 "releasing the simulated robot" "$1" \
    | sed -n 's/.*\[INFO\] \[\([0-9]*\.[0-9]*\)\].*/\1/p')"
  last="$(python3 -c 'import json, sys; print(json.load(open(sys.argv[1]))["last_stamp_s"])' "$2" 2>/dev/null)"
  if [ -z "$release" ]; then
    echo "SimPlant never logged 'releasing the simulated robot': it was held still the whole run"
  elif [ -z "$last" ]; then
    echo "the probe's metrics.json has no last_stamp_s, so the release time cannot be compared with the last sample"
  elif ! awk -v r="$release" -v e="$last" -v m="$RELEASED_MIN_S" 'BEGIN {exit !(e - r >= m)}'; then
    echo "SimPlant released the robot less than $RELEASED_MIN_S s before the probe's last sample (release $release, last sample $last)"
  fi
}

failed=0
could_not=0
# run <label> <feedforward rad/s, or "-" for none> <probe mode> <probe threshold> <launch args...>
run() {
  local label="$1" ff="$2" mode="$3" threshold="$4"; shift 4
  local dir="$out/$label"; mkdir -p "$dir"
  echo "=== $label: rocko_sim.launch.py teleop:=false $*"

  # The probe first, so it is listening before the cascade starts and the robot is released.
  start_group "$dir/probe.log" python3 "$here/sim/sim_probe.py" --duration "$PROBE_SECONDS" \
    --out "$dir" --mode "$mode" --threshold-deg "$threshold" --within "$FALL_LIMIT_S" \
    --wait "$STARTUP_TIMEOUT"
  local probe_pid="$!"
  start_group "$dir/launch.log" ros2 launch rocko_env rocko_sim.launch.py teleop:=false "$@"
  # A velocity reference of 0, so velocity_pid_controller (and so the whole cascade) computes.
  # Published at 20 Hz, not once: a single message sent before the controller subscribes is lost.
  start_group "$dir/reference.log" ros2 topic pub -r 20 /velocity_pid_controller/reference \
    control_msgs/msg/MultiDOFCommand "{dof_names: [left_wheel_joint, right_wheel_joint], values: [0.0, 0.0]}"
  if [ "$ff" != "-" ]; then
    for side in left right; do
      start_group "$dir/ff_$side.log" ros2 topic pub -r 20 "/${side}_feedforward" \
        std_msgs/msg/Float64 "{data: $ff}"
    done
  fi

  local deadline=$((SECONDS + STARTUP_TIMEOUT))
  wiring_problems
  while [ ${#problems[@]} -ne 0 ] && [ $SECONDS -lt $deadline ]; do sleep 1; wiring_problems; done

  wait "$probe_pid"; local probe_status=$?
  # Look at the wiring once more after the probe: it must still be up at the end of the run.
  [ ${#problems[@]} -eq 0 ] && wiring_problems
  sed 's/^/    /' "$dir/probe.log"

  # No metrics.json means the probe saw no message at all; it then reports "could not judge" itself.
  local not_released=""
  [ -s "$dir/metrics.json" ] && not_released="$(released_problem "$dir/launch.log" "$dir/metrics.json")"
  if [ ${#problems[@]} -ne 0 ]; then
    echo "  FAIL ($label): the wiring is not right:"
    printf '    - %s\n' "${problems[@]}"
    echo "  --- ros2 control list_controllers"; echo "${ctl:-<no answer>}" | sed 's/^/    /'
    echo "  --- launch log, lines mentioning an error"
    grep -E "ERROR|FATAL|died|[Ff]ailed|Traceback" "$dir/launch.log" | tail -20 | sed 's/^/    /'
    failed=1
  elif [ -n "$not_released" ]; then
    echo "  FAIL ($label): no evidence the robot was released during the probe window: $not_released"
    failed=1
  elif [ "$probe_status" -eq 0 ]; then
    echo "  ok   ($label)"
  elif [ "$probe_status" -eq 2 ]; then
    echo "  COULD NOT JUDGE ($label): see the probe's reason above"; could_not=1
  else
    echo "  FAIL ($label): see the probe's reason above"; failed=1
  fi
  if ! stop_groups; then
    echo "ERROR: the controller_manager from '$label' did not exit; cannot start the next run." >&2
    exit 2
  fi
}

run "S1-baseline" - still "$S1_STILL_DEG"
run "S1-command" "$FEEDFORWARD_RAD_S" change "$S1_CHANGE_DEG"
run "S2a-no-torque" - exceed "$TIPOVER_DEG" disable_torque:=true initial_pitch_deg:=2.0

if [ "$failed" -ne 0 ]; then
  echo "FAIL (simulation): see above."
  exit 1
fi
if [ "$could_not" -ne 0 ]; then
  echo "COULD NOT JUDGE (simulation): see above."
  exit 2
fi
passed=1
echo "PASS (simulation, S1 and S2a): the four cascade controllers and joint_state_broadcaster are"
echo "  active against rocko_env/SimPlant with all 12 interfaces; pitch stayed within $S1_STILL_DEG deg"
echo "  without a wheel command and moved >= $S1_CHANGE_DEG deg with a $FEEDFORWARD_RAD_S rad/s one; with no"
echo "  motor torque the robot passed $TIPOVER_DEG deg within $FALL_LIMIT_S s."
echo "  This is a MODEL WITH ESTIMATED, UNMEASURED PARAMETERS. It does not show that the real robot"
echo "  balances, or that any gain is good."
exit 0
