#!/usr/bin/env bash
# Tier 2 check: do ROCKO's controllers load, chain and activate against the MOCK hardware?
#
# Run inside the Docker dev container, after build_ros.sh. From the repo root on your laptop:
#   docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_mock.sh        # balancing
#   docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_mock.sh tank   # tank mode
# (-T because no terminal is attached; leave it off for an interactive shell.)
#
# This is a WIRING check, not a simulation. The mock has no physics (each command is echoed
# back as a state, and pitch stays 0.0), so a pass says NOTHING about whether the robot
# balances. What it does prove: every controller that the launch file spawns loads, claims
# interfaces the hardware layer exports, chains, activates and stays active, and the ROCKO
# nodes that need no real sensor have not crashed. See docs/architecture/verification-ladder.md.
#
# Exit status:  0 = PASS   1 = FAIL (what was missing is printed)
#               2 = could not run (no build, bad usage, or a controller_manager is already running)
#
# There is deliberately no `set -e`: grep and timeout are EXPECTED to "fail" while we wait for
# the system to come up, and each failure is turned into a message instead of an abort.
set -uo pipefail

# --- Timing. Generous bounds, not measured: bring-up time in the container was not recorded. ---
STARTUP_TIMEOUT=45   # s to wait for everything to become active. A look that passes just before
                     #   the deadline is followed by the SETTLE wait and a second look, so the
                     #   worst case before a verdict is STARTUP_TIMEOUT + SETTLE + 4 * CALL_TIMEOUT
                     #   (two looks, two ros2 calls each): about 80 s. Cleanup adds up to GRACE.
SETTLE=3             # s the system must STAY up. At 100 Hz that is 300 control cycles (150 in
                     #   tank mode, which runs at 50 Hz): a controller that dies on its first
                     #   update() is dead long before then.
CALL_TIMEOUT=8       # s for one `ros2 control` call. No answer in that long means no controller
                     #   manager is running; without a timeout the call waits forever.
GRACE=15             # s to let `ros2 launch` shut down before forcing it.

mode="${1:-balance}"
WS="${ROS2_WS:-$HOME/ros2_ws}"

# ROS's setup scripts read variables that may be unset, so `set -u` is off while sourcing them.
set +u
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$WS/install/setup.bash" 2>/dev/null \
  || { echo "ERROR: no build in $WS/install. Run src/ROCKO-env/build_ros.sh first." >&2; exit 2; }
set -u

# What "working" means, per mode. These names must match rocko_controllers.yaml /
# rocko_tank_mode_controllers.yaml and the MockRocko block in the ros2_control xacro.
case "$mode" in
  balance)
    launch=rocko.launch.py
    controllers="left_add_feedforward_controller right_add_feedforward_controller pitch_pid_controller velocity_pid_controller"
    # State interfaces the cascade reads: pitch for the inner loop, velocity for the outer.
    states="left_wheel_joint/pitch right_wheel_joint/pitch left_wheel_joint/velocity right_wheel_joint/velocity"
    ;;
  tank)
    launch=rocko_tank_mode.launch.py
    controllers="diffbot_base_controller joint_state_broadcaster"
    states="left_wheel_joint/velocity right_wheel_joint/velocity"
    ;;
  *)
    echo "usage: $0 [balance|tank]" >&2
    exit 2
    ;;
esac
# Both modes: something must have claimed each wheel's velocity command, or nothing drives it.
claimed="left_wheel_joint/velocity right_wheel_joint/velocity"

# Refuse to run if a controller manager is already up. `ros2 control` talks to whichever one
# answers, so a mock left over from an earlier run (or a live robot on the same network) could make
# a broken tree look healthy. LOCALHOST discovery keeps this run off other machines' ROS graphs.
# (Two runs started in the same instant can both pass this check before either launch is up, so
# run one at a time.)
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
if ! command -v pgrep >/dev/null; then
  echo "ERROR: pgrep (procps) is missing, so I cannot tell whether a controller_manager is running." >&2
  exit 2
fi
if pgrep -f ros2_control_node >/dev/null; then
  echo "ERROR: a controller_manager (ros2_control_node) is already running. Stop it first." >&2
  exit 2
fi

log="$(mktemp)"
# `setsid` puts the launch (and every process it starts) in its own process group. Because the
# job is not a group leader to begin with, setsid does not fork, so $! is that group's id.
# Cleanup can then signal the whole group at once with `kill -- -<id>` and leave no orphaned
# controller_manager or foxglove processes behind.
setsid ros2 launch rocko_env "$launch" use_mock_hardware:=true >"$log" 2>&1 &
launch_pid=$!
cleanup() {
  # SIGTERM, not SIGINT: a background job in a non-interactive script ignores SIGINT. Ask the
  # group to stop, give it GRACE seconds, then force it so this script can never hang on exit.
  kill -TERM -- "-$launch_pid" 2>/dev/null
  for _ in $(seq 1 "$GRACE"); do kill -0 "$launch_pid" 2>/dev/null || break; sleep 1; done
  kill -KILL -- "-$launch_pid" 2>/dev/null
  wait "$launch_pid" 2>/dev/null
  rm -f "$log"
}
trap cleanup EXIT

# has_interface <command|state> <name> [text the line must also contain, e.g. "[claimed]"]
# Reads the global $ifaces set by inspect(). `ros2 control list_hardware_interfaces` prints a
# "command interfaces" section and a "state interfaces" section. Plain grep is not enough:
# left_wheel_joint/velocity is BOTH a command and a state interface, so awk tracks which
# section each line is in.
has_interface() {
  awk -v sec="$1" -v name="$2" -v flag="${3:-}" '
    /^command interfaces/ { cur = "command"; next }
    /^state interfaces/   { cur = "state";   next }
    cur == sec && $1 == name && (flag == "" || index($0, flag)) { found = 1 }
    END { exit !found }' <<<"$ifaces"
}

# Look at the system once. Sets $ctl and $ifaces, and fills `problems` with everything wrong
# right now (empty means all is well).
problems=()
ctl=""
ifaces=""
inspect() {
  problems=()
  # -k 2: if the call ignores SIGTERM, kill it 2 s later, so this script cannot hang here.
  ctl="$(timeout -k 2 "$CALL_TIMEOUT" ros2 control list_controllers 2>/dev/null)"
  ifaces="$(timeout -k 2 "$CALL_TIMEOUT" ros2 control list_hardware_interfaces 2>/dev/null)"
  for c in $controllers; do
    grep -Eq "^$c[[:space:]].*[[:space:]]active([[:space:]]|$)" <<<"$ctl" \
      || problems+=("controller '$c' is not active")
  done
  for s in $states; do
    has_interface state "$s" || problems+=("state interface '$s' is not exported")
  done
  for c in $claimed; do
    has_interface command "$c" "[claimed]" || problems+=("command interface '$c' is not claimed by any controller")
  done
  # The ROCKO nodes that run under the mock (the ones that need no real sensor) must not crash:
  # a stack with a dead teleop node is not a working stack, even if the controllers are fine.
  for n in Joystick DiffDriveController; do
    grep -Eq "\[$n\.py-[0-9]+\]: (exception occurred|process has died)" "$log" \
      && problems+=("node $n.py crashed (see the launch log below)")
  done
}

# Poll until everything looks right, or STARTUP_TIMEOUT passes.
deadline=$((SECONDS + STARTUP_TIMEOUT))   # SECONDS is bash's own count of seconds since it started
while [ $SECONDS -lt $deadline ]; do
  inspect
  if [ ${#problems[@]} -eq 0 ]; then
    # Looked good once. Wait SETTLE seconds and look again: THIS second look is the verdict, so
    # a controller that activates and then dies cannot pass.
    sleep "$SETTLE"
    inspect
    break
  fi
  sleep 1
done

if [ ${#problems[@]} -eq 0 ]; then
  echo "PASS ($mode, mock hardware): all controllers active, interfaces present and claimed."
  echo "  controllers: $controllers"
  echo "  Reminder: this is a wiring check with no physics. It does not show the robot balances."
  exit 0
fi

echo "FAIL ($mode, mock hardware):"
printf '  - %s\n' "${problems[@]}"
echo "--- ros2 control list_controllers"
echo "${ctl:-<no answer within ${CALL_TIMEOUT} s: is the controller manager running?>}"
echo "--- ros2 control list_hardware_interfaces"
echo "${ifaces:-<no answer within ${CALL_TIMEOUT} s>}"
echo "--- launch log (last 40 lines mentioning an error)"
grep -E "ERROR|FATAL|died|[Ff]ailed|Traceback|Error" "$log" | tail -40
exit 1
