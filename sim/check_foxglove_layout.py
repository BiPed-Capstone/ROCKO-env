#!/usr/bin/env python3
"""Check that every signal the Foxglove layout names exists in the running system (used by
verify_tools.sh, against the simulated robot).

    check_foxglove_layout.py foxglove/rocko_balance_layout.json [--bridge-port 8765]
    check_foxglove_layout.py foxglove/rocko_battery_layout.json \
        --optional-topic /rocko/battery_raw=sensor_msgs/msg/BatteryState

A layout is a list of panels, and each plot line is a "message path" such as
    /pitch_pid_controller/controller_state.dof_states[0].feedback
(a topic, then the fields to follow inside its message). A layout that names a topic nobody
publishes, or a field the message does not have, loads without complaint and shows an empty plot.
For every path in the layout this script requires: the topic exists and has a publisher, and each
field in the path exists in the topic's message type. With --bridge-port it also requires that
foxglove_bridge accepts a TCP connection on that port.

--optional-topic TOPIC=TYPE (repeatable) is for a topic that may have no publisher where the check runs: the
battery monitor is an optional extra, absent from the simulation. Such a topic is not required to exist, but each
path on it must still name real fields of TYPE, so a misspelled field still fails.

What it CANNOT check: that Foxglove accepts the layout file or draws it as intended. The layout's
JSON structure was written from memory of Foxglove's exported layouts and has not been opened in
Foxglove (no display or account here). docs/guides/record-a-run.md says how to rebuild the panels by
hand from the same paths if the import fails.

Exit 0 = every path is real, 1 = at least one is not (one FAIL line each), 2 = could not run.
"""
import argparse
import json
import re
import socket
import sys
import time

import rclpy
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message


DISCOVERY_DEADLINE_S = 15.0   # [s] the longest it waits for the layout's topics to appear


def paths_in(layout):
    """Every message path in the layout: (panel id, path)."""
    found = []
    for panel, config in layout["configById"].items():
        for entry in config.get("paths", []):
            found.append((panel, entry["value"]))
        if config.get("topicPath"):
            found.append((panel, config["topicPath"]))
    return found


def field_problem(msg_class, fields):
    """None if the chain of field names exists in the message type, else what is wrong.
    A sequence or array type looks like "sequence<pkg/Type>" or "pkg/Type[4]"; the element type is
    what the next field name is looked up in."""
    current = msg_class
    for name in fields:
        types = current.get_fields_and_field_types()
        if name not in types:
            return f"no field '{name}' in {current.__module__.split('.')[0]}/{current.__name__} (it has {sorted(types)})"
        element = re.sub(r"^sequence<(.*?)(, *\d+)?>$", r"\1", types[name])
        element = re.sub(r"\[\d*\]$", "", element)
        if "/" in element:
            current = get_message(element.replace("/", "/msg/", 1) if "/msg/" not in element else element)
        else:
            current = None  # a number, string or bool: nothing further to look up
            if name != fields[-1]:
                return f"'{name}' is a {types[name]}, so nothing can follow it"
    return None


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("layout")
    p.add_argument("--bridge-port", type=int, default=None)
    p.add_argument("--optional-topic", action="append", default=[], metavar="TOPIC=TYPE",
                   help="a topic that may be unpublished here; its paths are checked against TYPE")
    args = p.parse_args()
    try:
        optional = dict(item.split("=", 1) for item in args.optional_topic)
        optional_types = {topic: get_message(type_name) for topic, type_name in optional.items()}
    except (ValueError, AttributeError, ModuleNotFoundError) as error:
        print(f"ERROR: bad --optional-topic: {error}")
        return 2
    try:
        with open(args.layout) as f:
            paths = paths_in(json.load(f))
    except (OSError, ValueError, KeyError) as error:
        print(f"ERROR: cannot read the layout: {error}")
        return 2
    if not paths:
        print("ERROR: the layout names no message path")
        return 2

    rclpy.init()
    node = Node("verify_tools_foxglove_layout_check")
    # Discovery takes a moment, and longer on a busy machine: wait until every topic the layout needs (the optional
    # ones aside) has a publisher, up to DISCOVERY_DEADLINE_S, instead of judging after a fixed wait (a fixed 3 s
    # once missed topics that were there, while the simulation was overloaded, 2026-11-27).
    needed = {path.partition(".")[0] for _, path in paths} - set(optional_types)
    end = time.monotonic() + DISCOVERY_DEADLINE_S
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
        if all(node.count_publishers(topic) > 0 for topic in needed):
            break
    topic_types = dict(node.get_topic_names_and_types())
    problems = []
    for panel, path in paths:
        topic, _, rest = path.partition(".")
        # "dof_states[0]" and "dof_states[:]" both mean the field dof_states.
        fields = [re.sub(r"\[.*?\]|\{.*?\}", "", part) for part in rest.split(".")] if rest else []
        if topic in optional_types:
            msg_class = optional_types[topic]   # checked against the type given, published or not
        elif topic not in topic_types:
            problems.append(f"{panel}: topic {topic} does not exist")
            continue
        else:
            if node.count_publishers(topic) == 0:
                problems.append(f"{panel}: nobody publishes {topic}")
            msg_class = get_message(topic_types[topic][0])
        problem = field_problem(msg_class, fields)
        if problem:
            problems.append(f"{panel}: {path}: {problem}")
    node.destroy_node()
    rclpy.shutdown()

    if args.bridge_port is not None:
        try:
            socket.create_connection(("127.0.0.1", args.bridge_port), timeout=3).close()
        except OSError as error:
            problems.append(f"foxglove_bridge does not accept a connection on port {args.bridge_port}: {error}")

    for problem in problems:
        print(f"FAIL  {problem}")
    if problems:
        return 1
    print(f"ok    foxglove: all {len(paths)} message paths in {args.layout.split('/')[-1]} name a published topic "
          "and real fields" + (f" (optional, fields only: {', '.join(optional)})" if optional else "") + (f"; the bridge listens on port {args.bridge_port}" if args.bridge_port else "")
          + ". NOT checked: that Foxglove loads or draws the layout")
    return 0


if __name__ == "__main__":
    sys.exit(main())
