#!/usr/bin/env bash
# COMPILE-ONLY check: does Motor12VoltQuadEncoder.cpp type-check off the Pi?
#
# The motor plugin needs WiringPi, which exists only on a Raspberry Pi, so build_ros.sh leaves it
# out and Tier 1 says nothing about it. This script runs `g++ -fsyntax-only` on it against STUB
# wiringPi.h / softPwm.h headers (tools/compile_check/stubs/). It proves SYNTAX AND TYPES ONLY:
# the file parses, every name resolves, every call matches a declared signature, and the ROS 2
# headers it uses exist in Jazzy. It never links, writes no object file, and cannot run anything.
# It says nothing about whether the plugin works, is fast enough, or builds on a Pi.
#
# Run inside the Docker dev container (needs the ROS 2 headers). From the repo root on your laptop:
#   docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/check_motor_plugin_compiles.sh
# Optional first argument: a different .cpp to check, for proving this check can fail (copy the
# plugin somewhere inside the container, break the copy on purpose, point the script at it).
# Warnings (-Wall -Wextra) are printed but do not fail the check.
#
# Exit status:  0 = compiles   1 = compile error   2 = could not run (no ROS, no g++, no file)
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${ROS2_WS:-$HOME/ros2_ws}"
source_file="${1:-$here/rocko_env/rocko_env/hardware/motors/Motor12VoltQuadEncoder.cpp}"
stubs="$here/tools/compile_check/stubs"
plugin_include="$here/rocko_env/rocko_env/hardware/motors/include"

# ROS's setup scripts read variables that may be unset, so `set -u` (which makes an unset variable
# an error) is turned off while sourcing them. Each setup file is checked to exist BEFORE it is
# sourced: if `source` is given a missing file inside `|| ...`, the shell can quietly end the
# script instead of reporting the problem, so we test with `[ -f ... ]` first and exit 2 ourselves.
[ -f /opt/ros/jazzy/setup.bash ] \
  || { echo "ERROR: /opt/ros/jazzy/setup.bash not found. Run this inside the dev container." >&2; exit 2; }
# The workspace build is sourced too, as the plugin's other ROS headers expect a built workspace.
# (Until 2026-11-24 the plugin also needed rocko_interfaces' QuadEncoderData service header; it counts
# its encoder itself now, finding C-1.)
[ -f "$WS/install/setup.bash" ] \
  || { echo "ERROR: no build in $WS/install. Run src/ROCKO-env/build_ros.sh first." >&2; exit 2; }
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$WS/install/setup.bash"
set -u

command -v g++ >/dev/null || { echo "ERROR: g++ not found." >&2; exit 2; }
[ -f "$source_file" ] || { echo "ERROR: no such file: $source_file" >&2; exit 2; }

# Since ROS Iron, a package's headers are at <prefix>/include/<package>/, so every such folder is
# an include path. The prefixes come from AMENT_PREFIX_PATH (set by the two setup scripts above).
# -isystem keeps the warnings below to ROCKO's own code, not ROS's.
isystem=()
IFS=':' read -r -a prefixes <<< "${AMENT_PREFIX_PATH:-}"
for prefix in "${prefixes[@]}"; do
  for dir in "$prefix"/include/*/; do
    [ -d "$dir" ] && isystem+=(-isystem "${dir%/}")
  done
done
if [ "${#isystem[@]}" -eq 0 ]; then
  echo "ERROR: found no ROS include folders under AMENT_PREFIX_PATH='${AMENT_PREFIX_PATH:-}'." >&2
  exit 2
fi

# -I folders are searched before -isystem ones, so the stubs win even if a real WiringPi existed.
# ROCKO_COMPILE_CHECK_ONLY is what lets the stubs compile at all (they #error without it).
# -fsyntax-only: no object file, no link. No -o on purpose.
cmd=(g++ -std=c++17 -fsyntax-only -Wall -Wextra
     -DROCKO_COMPILE_CHECK_ONLY
     -I "$stubs" -I "$plugin_include"
     "${isystem[@]}"
     "$source_file")

echo "Compile-only check of: $source_file"
echo "Stub headers:          $stubs"
# Each include folder takes two array entries ("-isystem" and the path), so the number of folders
# is the array length divided by 2.
echo "ROS include folders:   $(( ${#isystem[@]} / 2 ))"
echo

if "${cmd[@]}"; then
  echo
  echo "PASS (compile-only): syntax and types only. Not linked, not run, not a Pi build."
  exit 0
fi

echo
echo "FAIL (compile-only): g++ rejected $source_file. The errors are above." >&2
echo "This is a compile error, not a link or runtime result." >&2
exit 1
