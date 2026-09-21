#!/usr/bin/env bash
# ROCKO-env PreToolUse hook.
#
# Some files in this repo encode a human decision rather than code: the version
# pins, and the PID gains (which are experimental results measured on a specific
# robot, surface and battery voltage). An agent has no way to verify a change to
# either.
#
# This hook does not block those edits. It escalates them to a permission prompt,
# so a human sees the reason and decides. See CLAUDE.md section 1.
#
# Contract: PreToolUse hooks receive the tool call as JSON on stdin. Emitting
# permissionDecision "ask" turns the call into a prompt for the user.

set -uo pipefail

payload="$(cat)"

python3 - "$payload" <<'PY'
import json, sys

try:
    payload = json.loads(sys.argv[1])
except (json.JSONDecodeError, IndexError):
    sys.exit(0)  # never break the session on a parse failure

path = (payload.get("tool_input") or {}).get("file_path") or ""
if not path:
    sys.exit(0)

REASONS = [
    (
        ("rocko_env/rocko_env/bringup/config/",),
        "This file holds PID gains and the controller update_rate. Gains are "
        "experimental results measured on a specific robot, surface and battery "
        "voltage -- not code -- and an agent cannot verify a change to them. "
        "Expected instead: explain which loop is affected and why, predict the "
        "direction of the change, and let a human run the experiment. See the "
        "tune-gains skill.",
    ),
    (
        ("package.xml", "docker/Dockerfile", "rocko_env/CMakeLists.txt"),
        "This file carries a version pin (ROS 2 Jazzy, ros2_control >= 4.0.0, "
        "Ubuntu 24.04, C++17). Those are locked deliberately -- see "
        "docs/decisions/0001-version-lock.md. Adding a dependency or a source "
        "file here is normal; changing a pinned version is a human decision.",
    ),
]

for needles, reason in REASONS:
    if any(n in path for n in needles):
        print(json.dumps({
            "hookSpecificOutput": {
                "hookEventName": "PreToolUse",
                "permissionDecision": "ask",
                "permissionDecisionReason": f"{path}\n\n{reason}",
            }
        }))
        sys.exit(0)

sys.exit(0)
PY
