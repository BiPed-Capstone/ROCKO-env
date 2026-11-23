#!/usr/bin/env bash
# Measure how long diff_drive_controller takes to turn a full-forward command into a full-reverse one, in
# the dev container, on the MOCK (no robot, no physics). The ramp is pure software (the controller's
# acceleration limits, linear.x.* in rocko_tank_mode_controllers.yaml), so the mock measures it exactly;
# the wheels' own response is NOT measured here.
#
#   docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/tools/measure_reversal_ramp.sh [SPEED] [name=value ...]
#
# SPEED [m/s] (default 6.0, what a full stick asks for at max_speed:=0.4, the first stand test's cap; 15.0 is
# full speed): the command steps from +SPEED to -SPEED. Each name=value is set on /diffbot_base_controller with
# `ros2 param set` before the step (then the controller is reconfigured, below), so a PROPOSED limit can be
# measured without editing the guarded YAML (CLAUDE.md section 3), e.g.
#   measure_reversal_ramp.sh 6.0 linear.x.max_deceleration=-30.0 linear.x.max_deceleration_reverse=30.0
# Prints when the limited command (cmd_vel_out) crossed zero and reached 90 % of -SPEED after the step.
source /opt/ros/jazzy/setup.bash      # (before set -u: ROS's setup scripts read unset variables)
source /ros2_ws/install/setup.bash
set -u
speed="${1:-6.0}"; shift || true
startup_wait_s=20       # [s] the most to wait for the controller to become active
launch_stop_wait_s=10   # [s] after SIGINT, how long ros2 launch gets to stop its nodes before they are forced
out="$(mktemp -d)"

# Why setsid and --default-signal=INT: a job started in the background of a script begins with SIGINT
# ignored, so the usual "Ctrl-C" stop would do nothing and the launch would outlive this script. setsid also
# gives the launch its own process group, so the last-resort kill below reaches every node it started.
setsid env --default-signal=INT ros2 launch rocko_env rocko_tank_mode.launch.py use_mock_hardware:=true \
  teleop:=false dashboard:=false foxglove:=false >"$out/launch.log" 2>&1 &
launch_pid=$!
stop_launch() {
  kill -INT "$launch_pid" 2>/dev/null
  for _ in $(seq 1 $((launch_stop_wait_s * 2))); do kill -0 "$launch_pid" 2>/dev/null || break; sleep 0.5; done
  kill -TERM -- "-$launch_pid" 2>/dev/null   # "-PID" = the whole process group: any node SIGINT missed
  rm -rf "$out"
}
trap stop_launch EXIT
active=0
for _ in $(seq 1 $((startup_wait_s * 2))); do
  # -w: the whole word "active", so a controller that is still "inactive" does not count
  ros2 control list_controllers 2>/dev/null | grep "diffbot_base_controller" | grep -qw active && { active=1; break; }
  sleep 0.5
done
[ "$active" -eq 1 ] || { echo "diffbot_base_controller did not become active within ${startup_wait_s} s"; exit 2; }
for setting in "$@"; do
  ros2 param set /diffbot_base_controller "${setting%%=*}" "${setting#*=}" 2>&1 | sed "s/^/param ${setting%%=*}: /"
done
# diff_drive_controller reads its limits only when it is CONFIGURED (seen 2026-11-22: a runtime
# `ros2 param set` changed nothing). Each step below is one lifecycle transition; going back through
# "unconfigured" makes the next "inactive" a fresh configure, which re-reads the parameters.
if [ $# -gt 0 ]; then
  for state in inactive unconfigured inactive active; do
    ros2 control set_controller_state diffbot_base_controller "$state" >/dev/null 2>&1 \
      || echo "could not set the controller $state"
  done
  ros2 control list_controllers 2>/dev/null | grep diffbot_base_controller
fi

# The probe: a small ROS node that holds +SPEED long enough to reach it, then commands -SPEED, and records
# every cmd_vel_out (the command after the controller's limits) with the time it arrived.
python3 - "$speed" <<'EOF'
import sys, time, rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
SETTLE_S = 4.0         # [s] held at +SPEED first: 15 m/s at the YAML's 10 m/s^2 needs 1.5 s
STEP_WINDOW_S = 5.0    # [s] recorded after the step to -SPEED
SEND_PERIOD_S = 0.02   # [s] 50 Hz, as TankTeleop.py sends
speed = float(sys.argv[1])
rclpy.init()
node = Node("reversal_probe")
pub = node.create_publisher(TwistStamped, "/diffbot_base_controller/cmd_vel", 10)
seen = []  # (time [s], limited linear.x [m/s])
node.create_subscription(TwistStamped, "/diffbot_base_controller/cmd_vel_out",
                         lambda m: seen.append((time.monotonic(), m.twist.linear.x)), 50)
def run(target, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        msg = TwistStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.twist.linear.x = target
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=SEND_PERIOD_S)
run(speed, SETTLE_S)
start = time.monotonic()
run(-speed, STEP_WINDOW_S)
after = [(t - start, v) for t, v in seen if t >= start]
before = [v for t, v in seen if t < start]
zero = next((t for t, v in after if v <= 0.0), None)
full = next((t for t, v in after if v <= -0.9 * speed), None)
print(f"speed {speed} m/s: before the step the limited command was {before[-1] if before else float('nan'):+.2f}; "
      f"after it, zero crossing at {zero if zero is None else round(zero, 2)} s, 90 % of reverse at {full if full is None else round(full, 2)} s")
rclpy.shutdown()
EOF
