#!/usr/bin/env python3
"""Judge the running web dashboard against the simulated robot (used by verify_tools.sh).

    check_dashboard.py --port 8080 --log-root DIR --record-s 4 --tolerance-deg 0.5 --min-pitch-deg 1.0

It listens to /dynamic_joint_states itself while it asks the dashboard for its history, then:
  * the page loads and refers to nothing outside the robot (no http:// or https:// address in it);
  * /api/state has fresh values and names the launch and "sim";
  * the controller card: this script publishes a fake /joy (left stick half up, LB held, a NaN right
    stick Y) and /tank_teleop/enabled = true, as tank mode's nodes would; /api/state must show them
    by name, with the NaN as null (valid JSON), and the page must contain the controller drawing;
  * every pitch point in the dashboard's history is within --tolerance-deg of the robot's own
    pitch at that moment, and the history really shows the start-up lean (>= --min-pitch-deg);
  * the dashboard node publishes on no topic except ROS's own /rosout and /parameter_events, and its
    only service clients are the supervisor's two (/rocko/set_enabled, /rocko/set_mode): it cannot
    command the robot itself. This launch has no supervisor, so /api/state must answer fast with
    robot.present false ("no supervisor"), and Enable must be refused fast, not hang;
  * Record over HTTP starts a recording, a second Record is refused, Stop closes it, the run is
    listed as closed and its .tar downloads; a POST from another website's Origin, or whose Host
    header is another site's name, is refused; a path that climbs out of the log folder is refused.
Exit 0 = all held, 1 = something did not (one FAIL line each), 2 = could not judge.
The recording itself is judged afterwards by check_recording.py.
"""
import argparse
import bisect
import io
import json
import sys
import tarfile
import time
import urllib.error
import urllib.request

import rclpy
from control_msgs.msg import DynamicJointState
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

OWN_TOPICS = {"/rosout", "/parameter_events"}  # every ROS node publishes these
SUPERVISOR_SERVICES = {"/rocko/set_enabled", "/rocko/set_mode"}  # the only services the dashboard may call
FAST_S = 2.0  # [s] an answer that needs no ROS round trip (no supervisor, state after Record) comes this fast


def http(port, path, body=None, headers=None):
    """(status, bytes) of a GET (body None) or a JSON POST to the dashboard."""
    request = urllib.request.Request(
        f"http://127.0.0.1:{port}{path}", data=None if body is None else json.dumps(body).encode(),
        headers={"Content-Type": "application/json", **(headers or {})})
    try:
        with urllib.request.urlopen(request, timeout=40) as response:
            return response.status, response.read()
    except urllib.error.HTTPError as error:
        return error.code, error.read()


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--port", type=int, required=True)
    p.add_argument("--record-s", type=float, required=True, help="how long the HTTP recording runs [s]")
    p.add_argument("--tolerance-deg", type=float, required=True)
    p.add_argument("--min-pitch-deg", type=float, required=True)
    p.add_argument("--listen-s", type=float, default=6.0, help="how long to collect the robot's own pitch [s]")
    args = p.parse_args()
    problems = []

    # --- the robot's own pitch, with the time each sample ARRIVED here (the dashboard stamps its
    # history with the time it sampled, on the same machine's clock) ---
    rclpy.init()
    node = Node("verify_tools_dashboard_check")
    own = []  # (unix time, pitch [deg])

    def keep(msg):
        for joint, interfaces in zip(msg.joint_names, msg.interface_values):
            if joint == "left_wheel_joint" and "pitch" in interfaces.interface_names:
                own.append((time.time(), interfaces.values[list(interfaces.interface_names).index("pitch")]))

    node.create_subscription(DynamicJointState, "/dynamic_joint_states", keep, 200)
    # Stand-ins for tank mode's joy node and TankTeleop.py (the simulation starts neither).
    # Index layout: gamepad.py (axis 1 = left Y, axis 3 = right Y, button 9 = LB).
    fake_joy = node.create_publisher(Joy, "/joy", 5)
    fake_enabled = node.create_publisher(Bool, "/tank_teleop/enabled", 5)
    joy = Joy()
    joy.axes = [0.0, 0.5, 0.0, float("nan"), 0.0, 0.0]
    joy.buttons = [0] * 21
    joy.buttons[9] = 1
    end = time.monotonic() + args.listen_s
    while time.monotonic() < end:
        fake_joy.publish(joy)
        fake_enabled.publish(Bool(data=True))
        rclpy.spin_once(node, timeout_sec=0.05)
    # One last pair just before /api/state is read below, so the values are fresh then.
    fake_joy.publish(joy)
    fake_enabled.publish(Bool(data=True))
    rclpy.spin_once(node, timeout_sec=0.2)
    publishers = {name for name, _ in node.get_publisher_names_and_types_by_node("rocko_dashboard", "/")}
    clients = node.get_client_names_and_types_by_node("rocko_dashboard", "/")
    node.destroy_node()
    rclpy.shutdown()
    if len(own) < 100:
        print(f"COULD NOT JUDGE: only {len(own)} /dynamic_joint_states samples in {args.listen_s} s")
        return 2

    # --- the page ---
    status, page = http(args.port, "/")
    text = page.decode(errors="replace")
    if status != 200 or "ROCKO" not in text:
        problems.append(f"GET / gave status {status} and no dashboard page")
    for scheme in ("http://", "https://"):
        if scheme in text:
            problems.append(f"the page refers to an outside address ({scheme}...): it must work with no internet")

    # --- /api/state ---
    status, body = http(args.port, "/api/state")
    state = json.loads(body) if status == 200 else {}
    values, ages = state.get("values", {}), state.get("age_s", {})
    for key in ("pitch_deg", "left_velocity_m_s", "right_velocity_m_s", "target_pitch_deg",
                "wheel_command_rad_s", "velocity_reference_m_s", "command_timed_out"):
        if values.get(key) is None:
            problems.append(f"/api/state has no value for {key}")
    for group in ("joint_states", "pitch_pid", "velocity_pid", "reference"):
        if not (ages.get(group, 1e9) <= state.get("stale_after_s", 0)):
            problems.append(f"/api/state: {group} is {ages.get(group)} s old (stale)")
    controller = state.get("controller") or {}
    pad_axes, pad_buttons = controller.get("axes", {}), controller.get("buttons", {})
    if not (pad_axes.get("left_y") == 0.5 and pad_buttons.get("left_bumper") == 1 and pad_buttons.get("a") == 0):
        problems.append(f"/api/state controller does not show the fake /joy by name: {controller}")
    if "right_y" not in pad_axes or pad_axes["right_y"] is not None:
        problems.append(f"a NaN axis must be sent as null, got {pad_axes.get('right_y', 'nothing')!r}")
    if values.get("teleop_enabled") is not True or "controller" not in ages:
        problems.append(f"/api/state: teleop_enabled {values.get('teleop_enabled')!r}, controller age {ages.get('controller')!r}")
    if 'id="pad"' not in text or "showController" not in text:
        problems.append("the page has no controller drawing")
    if state.get("hardware") != "sim" or state.get("launch") != "rocko_sim.launch.py":
        problems.append(f"/api/state names launch {state.get('launch')!r}, hardware {state.get('hardware')!r}")

    # --- history against the robot's own samples ---
    status, body = http(args.port, "/api/history?since=0")
    history = json.loads(body) if status == 200 else []
    times = [t for t, _ in own]
    compared, worst = 0, 0.0
    for point in history:
        if "pitch_deg" not in point or not (times[0] <= point["t"] <= times[-1]):
            continue  # outside the stretch this script listened to
        i = bisect.bisect_left(times, point["t"])
        nearest = min((own[j] for j in (i - 1, i) if 0 <= j < len(own)), key=lambda s: abs(s[0] - point["t"]))
        worst = max(worst, abs(point["pitch_deg"] - nearest[1]))
        compared += 1
    peak = max((abs(point.get("pitch_deg", 0.0)) for point in history), default=0.0)
    if compared < 40:
        problems.append(f"only {compared} history points overlap the {args.listen_s} s this script listened")
    if worst > args.tolerance_deg:
        problems.append(f"the dashboard's pitch differs from the robot's by up to {worst:.3f} deg "
                        f"(tolerance {args.tolerance_deg} deg)")
    if peak < args.min_pitch_deg:
        problems.append(f"the history's largest |pitch| is {peak:.3f} deg; the start-up lean of "
                        f"{args.min_pitch_deg} deg or more is missing, so the comparison shows nothing")

    # --- it cannot command the robot ---
    if publishers - OWN_TOPICS:
        problems.append(f"the dashboard node publishes on {sorted(publishers - OWN_TOPICS)}; it must publish nothing")
    if {name for name, _ in clients} - SUPERVISOR_SERVICES:
        problems.append(f"the dashboard node has service clients {clients}; only the supervisor's two are allowed")
    # No supervisor in this launch: the state says so at once, and a button press is refused at once.
    start = time.monotonic()
    status, body = http(args.port, "/api/state")
    robot = json.loads(body).get("robot", {}) if status == 200 else {}
    if robot.get("present") is not False or robot.get("message") != "no supervisor" or time.monotonic() - start > FAST_S:
        problems.append(f"/api/state robot is {robot} after {time.monotonic() - start:.1f} s; expected "
                        f"present false, 'no supervisor', within {FAST_S:g} s")
    start = time.monotonic()
    status, body = http(args.port, "/api/enable", {})
    if status != 504 or json.loads(body).get("success") is not False or time.monotonic() - start > FAST_S:
        problems.append(f"Enable with no supervisor gave status {status} {body[:100]!r} after "
                        f"{time.monotonic() - start:.1f} s; expected 504 within {FAST_S:g} s")

    # --- refusals ---
    status, _ = http(args.port, "/api/record/start", {"label": "x"}, {"Origin": "http://elsewhere.example"})
    if status != 403:
        problems.append(f"a POST from another site's Origin got status {status}, expected 403")
    status, _ = http(args.port, "/api/record/start", {"label": "x"}, {"Host": "elsewhere.example:8080"})
    if status != 403:
        problems.append(f"a POST whose Host is another site's name got status {status}, expected 403")
    status, _ = http(args.port, "/api/runs/..%2F..%2Fetc.tar")
    if status != 404:
        problems.append(f"a path climbing out of the log folder got status {status}, expected 404")

    # --- Record, a second Record, Stop, the list, the download ---
    status, body = http(args.port, "/api/record/start", {"label": "dashboard", "battery_v": "12.3", "surface": "simulated"})
    if status != 200:
        problems.append(f"Record gave status {status}: {body[:200]!r}")
    time.sleep(args.record_s)
    # Starting a recording once froze the dashboard (a lock taken twice): the state must still answer.
    start = time.monotonic()
    running = json.loads(http(args.port, "/api/state")[1]).get("recording", {})
    if time.monotonic() - start > FAST_S:
        problems.append(f"/api/state took {time.monotonic() - start:.1f} s while recording")
    if not running.get("running"):
        problems.append("after Record, /api/state does not say a recording is running")
    status, _ = http(args.port, "/api/record/start", {"label": "second"})
    if status != 409:
        problems.append(f"a second Record while one runs got status {status}, expected 409")
    status, body = http(args.port, "/api/record/stop", {})
    if status != 200:
        problems.append(f"Stop gave status {status}: {body[:200]!r}")
    runs = json.loads(http(args.port, "/api/runs")[1])
    mine = [run for run in runs if run["name"].endswith("_dashboard")]
    if len(runs) != 1 or len(mine) != 1 or not mine[0]["closed"]:
        problems.append(f"expected one closed run ending in _dashboard in the list, got {runs}")
    else:
        status, data = http(args.port, f"/api/runs/{mine[0]['name']}.tar")
        try:
            names = tarfile.open(fileobj=io.BytesIO(data)).getnames()
        except tarfile.TarError as error:
            names, status = [], f"not a tar file ({error})"
        if status != 200 or f"{mine[0]['name']}/run.yaml" not in names:
            problems.append(f"the run's download gave status {status} and files {names[:5]}")

    for problem in problems:
        print(f"FAIL  {problem}")
    if problems:
        return 1
    print(f"ok    dashboard: {compared} history points within {worst:.3f} deg of the robot's pitch "
          f"(start-up lean {peak:.2f} deg seen); fake /joy and enabled shown; publishes nothing; Record/Stop made {mine[0]['name']} "
          f"({mine[0]['size_mb']} MB); foreign Origin, a second Record and a climbing path refused")
    return 0


if __name__ == "__main__":
    sys.exit(main())
