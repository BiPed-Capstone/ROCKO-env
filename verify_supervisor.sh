#!/usr/bin/env bash
# Tier 2 check: does the robot supervisor (docs/architecture/robot-supervisor.md) follow its rules against the MOCK?
#
# Run inside the Docker dev container, after build_ros.sh. From the repo root on your laptop:
#   docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_supervisor.sh
# (about 3 to 4 minutes: two mode changes, a recovery from a fault, and the shutdown.)
#
# It starts `rocko_robot.launch.py use_mock_hardware:=true` (the base launch; the supervisor starts the
# tank stack itself), then runs tools/supervisor_flow_check.py, which drives the supervisor through its
# services and a simulated /joy and prints one PASS/FAIL line per check. Then it stops the launch the way
# a Ctrl-C does and checks that nothing is left running. This is a WIRING-and-RULES check with no physics:
# the mock echoes each wheel command back as a velocity and holds pitch at 0.0.
#
# Exit status:  0 = PASS   1 = FAIL (the failing lines are printed)
#               2 = could not run (no build, or a controller_manager is already running)
#
# No `set -e`: waiting and cleanup are EXPECTED to see failures and turn them into messages.
set -uo pipefail

CHECK_TIMEOUT=600   # s for the whole flow check; each step inside has its own, shorter bound
STOP_TIMEOUT=45     # s to let the launch end after SIGINT. The supervisor may wait up to 15 s for its
                    #   child stack, and that stack's own shutdown takes a few seconds more.

WS="${ROS2_WS:-$HOME/ros2_ws}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
set +u
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$WS/install/setup.bash" 2>/dev/null \
  || { echo "ERROR: no build in $WS/install. Run src/ROCKO-env/build_ros.sh first." >&2; exit 2; }
set -u

# Keep this run off other machines' ROS graphs, and refuse to run beside another controller_manager
# (the checks would talk to whichever one answers).
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
command -v pgrep >/dev/null || { echo "ERROR: pgrep (procps) is missing." >&2; exit 2; }
if pgrep -f '/ros2_control_node|Supervisor\.py|ros2 launch rocko_env|DiffDriveController\.py' >/dev/null; then
  echo "ERROR: a controller_manager, supervisor or ROCKO launch is already running. Stop it first." >&2
  exit 2
fi

log="$(mktemp)"
# Own process group (setsid), so the whole launch can be signalled at once.
setsid ros2 launch rocko_env rocko_robot.launch.py use_mock_hardware:=true dashboard:=false foxglove:=false \
  >"$log" 2>&1 &
launch_pid=$!

# Names of processes this launch (or the stack the supervisor starts) must not leave behind.
leftovers() {
  pgrep -af '/ros2_control_node|ros2 launch rocko_env|Supervisor\.py|game_controller_node|TankTeleop\.py|BalanceTeleop\.py|DiffDriveController\.py|robot_state_publisher'
}

forced=""
stop_launch() {
  # SIGINT to the group, like Ctrl-C: launch passes it on, and the supervisor stops its child stack.
  kill -INT -- "-$launch_pid" 2>/dev/null
  for _ in $(seq 1 "$STOP_TIMEOUT"); do kill -0 "$launch_pid" 2>/dev/null || break; sleep 1; done
  if kill -0 "$launch_pid" 2>/dev/null; then
    forced="the launch did not end within ${STOP_TIMEOUT} s of SIGINT"
    kill -TERM -- "-$launch_pid" 2>/dev/null; sleep 3
    kill -KILL -- "-$launch_pid" 2>/dev/null
  fi
  wait "$launch_pid" 2>/dev/null
}
trap 'stop_launch; rm -f "$log"' EXIT

timeout -k 5 "$CHECK_TIMEOUT" python3 "$HERE/tools/supervisor_flow_check.py"
flow_rc=$?
[ "$flow_rc" = 124 ] && echo "FAIL  the flow check did not finish within ${CHECK_TIMEOUT} s"

# ---- 10. cleanup ----------------------------------------------------------------------------
stop_launch
trap 'rm -f "$log"' EXIT
sleep 1
left="$(leftovers)"
if [ -z "$forced" ] && [ -z "$left" ]; then
  echo "PASS  10 launch stopped by SIGINT, no ros2_control_node or child launch left"
  cleanup_rc=0
else
  cleanup_rc=1
  [ -n "$forced" ] && echo "FAIL  10 $forced"
  if [ -n "$left" ]; then
    echo "FAIL  10 processes left behind (killing them):"; echo "$left" | sed 's/^/        /'
    pkill -KILL -f '/ros2_control_node|ros2 launch rocko_env|Supervisor\.py|game_controller_node|TankTeleop\.py|BalanceTeleop\.py|DiffDriveController\.py|robot_state_publisher'
  fi
fi

if [ "$flow_rc" = 0 ] && [ "$cleanup_rc" = 0 ]; then
  echo "PASS (supervisor, mock hardware): every check passed."
  echo "  Reminder: a wiring-and-rules check with no physics. It does not show the robot balances."
  exit 0
fi
echo "FAIL (supervisor, mock hardware)"
cp "$log" /tmp/verify_supervisor.last_fail.log 2>/dev/null && echo "  full launch log kept in /tmp/verify_supervisor.last_fail.log"
echo "--- supervisor log (its state changes, last 40)"
grep -E "\[Supervisor\.py-1\] \[INFO\].*(state:|stack|supervisor up)|\[WARN\].*supervisor|\[tank_teleop\]|\[supervisor\]" "$log" | tail -40
echo "--- launch log (last 40 lines mentioning an error)"
grep -E "ERROR|FATAL|died|[Ff]ailed|Traceback|Error" "$log" | tail -40
exit 1
