#!/usr/bin/env python3
"""Judge one recording made by RecordRun.py (used by verify_tools.sh, in the dev container).

    check_recording.py LOG_ROOT --label L --hardware sim --launch F --battery-v V --surface S \
        --min-state-messages N --require TOPIC [TOPIC ...]

PASS (exit 0) if LOG_ROOT holds exactly one run folder whose name ends in the label, and in it:
  * run.yaml has the label, hardware, launch file, battery and surface that were asked for, a
    commit that is not "unknown", a config path whose SHA-256 matches the file, and a stop reason;
  * bag/metadata.yaml exists (rosbag2 writes it only when the recording was closed properly) and
    says the storage is mcap;
  * every --require topic has at least one message, and /dynamic_joint_states at least N.
Exit 1 with one line per problem otherwise. It reads rosbag2's own metadata; it does not replay the bag.
"""
import argparse
import hashlib
import os
import sys

import yaml


def main():
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("log_root")
    p.add_argument("--label", required=True)
    p.add_argument("--hardware", required=True)
    p.add_argument("--launch", required=True)
    p.add_argument("--battery-v", required=True)
    p.add_argument("--surface", required=True)
    p.add_argument("--min-state-messages", type=int, required=True)
    p.add_argument("--require", nargs="+", required=True)
    args = p.parse_args()

    runs = sorted(os.listdir(args.log_root)) if os.path.isdir(args.log_root) else []
    if len(runs) != 1 or not runs[0].endswith("_" + args.label):
        print(f"FAIL  expected exactly one run folder ending in _{args.label} in {args.log_root}, found {runs}")
        return 1
    run_dir = os.path.join(args.log_root, runs[0])
    problems = []

    try:
        with open(os.path.join(run_dir, "run.yaml")) as f:
            run = yaml.safe_load(f)
    except (OSError, yaml.YAMLError) as error:
        print(f"FAIL  run.yaml cannot be read: {error}")
        return 1
    for key, want in (("label", args.label), ("hardware", args.hardware), ("launch", args.launch),
                      ("battery_v", args.battery_v), ("surface", args.surface)):
        if str(run.get(key)) != want:
            problems.append(f"run.yaml {key} is {run.get(key)!r}, expected {want!r}")
    if run.get("commit") in (None, "", "unknown"):
        problems.append(f"run.yaml commit is {run.get('commit')!r}: the recorder could not find the checkout")
    if run.get("tree") not in ("clean", "dirty"):
        problems.append(f"run.yaml tree is {run.get('tree')!r}, expected clean or dirty")
    config = run.get("config") or ""
    try:
        with open(config, "rb") as f:
            if hashlib.sha256(f.read()).hexdigest() != run.get("config_sha256"):
                problems.append("run.yaml config_sha256 does not match the config file")
    except OSError:
        problems.append(f"run.yaml config {config!r} is not a readable file")
    if not run.get("stop_reason"):
        problems.append("run.yaml has no stop_reason: RecordRun.py did not finish")

    try:
        with open(os.path.join(run_dir, "bag", "metadata.yaml")) as f:
            info = yaml.safe_load(f)["rosbag2_bagfile_information"]
    except (OSError, yaml.YAMLError, KeyError, TypeError) as error:
        print(f"FAIL  bag/metadata.yaml is missing or unreadable ({error}): the bag was not closed properly")
        for problem in problems:
            print(f"FAIL  {problem}")
        return 1
    if info.get("storage_identifier") != "mcap":
        problems.append(f"the bag's storage is {info.get('storage_identifier')!r}, expected mcap")
    counts = {t["topic_metadata"]["name"]: t["message_count"] for t in info["topics_with_message_count"]}
    for topic in args.require:
        if counts.get(topic, 0) < 1:
            problems.append(f"{topic} has no messages in the bag")
    states = counts.get("/dynamic_joint_states", 0)
    if states < args.min_state_messages:
        problems.append(f"/dynamic_joint_states has {states} messages, expected at least {args.min_state_messages}")

    for problem in problems:
        print(f"FAIL  {problem}")
    if problems:
        return 1
    size_mb = sum(os.path.getsize(os.path.join(run_dir, "bag", name))
                  for name in os.listdir(os.path.join(run_dir, "bag"))) / 1e6
    seconds = info["duration"]["nanoseconds"] / 1e9
    print(f"ok    {runs[0]}: commit {run['commit']} ({run['tree']}), {info['message_count']} messages in "
          f"{seconds:.1f} s, {size_mb:.2f} MB ({60 * size_mb / max(seconds, 1e-9):.1f} MB per minute); "
          f"/dynamic_joint_states {states}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
