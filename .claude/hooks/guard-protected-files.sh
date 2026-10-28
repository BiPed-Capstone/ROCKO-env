#!/usr/bin/env bash
# ROCKO-env PreToolUse hook entry point. All the logic, and the explanation of what this guards
# and why, is in guard_protected_files.py next to this file. It stays a .sh only so that the hook
# command in .claude/settings.json did not have to change. The payload arrives on stdin and goes
# straight to Python, never through a command-line argument (an argument has a size limit, and a
# crash here would let a gated edit through).
exec python3 "$(dirname "${BASH_SOURCE[0]}")/guard_protected_files.py"
