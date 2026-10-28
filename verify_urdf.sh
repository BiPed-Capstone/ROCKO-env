#!/usr/bin/env bash
# Tier 0 check: does the robot description expand and parse in every mode, and does each mode
# contain the hardware it should?
#
# Run inside the Docker dev container, after build_ros.sh (the xacro files find each other
# through the installed package). From the repo root on your laptop:
#   docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_urdf.sh
#
# For balancing and tank mode, each with use_mock_hardware false and true, it expands the xacro,
# runs check_urdf, and then ASSERTS WHAT IS IN THE RESULT:
#   mock=false -> the two real motor plugins (plus the IMU, in balancing mode), no mock, nothing else
#   mock=true  -> exactly one mock_components system (MockRocko), no real hardware, nothing else
# and, per model, that the mock exports EXACTLY the same interfaces as the real hardware (joints,
# sensors and gpio alike; a duplicated interface counts as a difference).
# check_urdf alone passes whichever block was selected; without these assertions a swapped
# xacro:if / xacro:unless, or a mock that has drifted from the real names, would pass every
# off-Pi check and first show up on the robot.
#
# Exit status:  0 = all four combinations ok   1 = something wrong   2 = could not run (no build)
set -uo pipefail

WS="${ROS2_WS:-$HOME/ros2_ws}"

# ROS's setup scripts read variables that may be unset, so `set -u` is off while sourcing them.
set +u
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$WS/install/setup.bash" 2>/dev/null \
  || { echo "ERROR: no build in $WS/install. Run src/ROCKO-env/build_ros.sh first." >&2; exit 2; }
set -u

urdf_dir="$WS/install/rocko_env/share/rocko_env/urdf"
work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT
failures=0

fail() { echo "FAIL  $1"; failures=$((failures + 1)); }

# show <file> -> the file's text, indented, under a FAIL line (the cause, not just its last line)
show() { sed 's/^/        /' "$1"; }

# count <file> <fixed text> -> how many lines of the file contain it. Callers count whole
# <plugin>...</plugin> tags, never a bare name: xacro keeps XML comments in its output, and a
# comment may well mention a plugin by name.
count() { grep -cF "$2" "$1"; }

# interfaces <urdf> -> one "<joint|sensor|gpio> <name> <command|state> <interface>" line per
# interface that any <ros2_control> block gives a component. The hardware plugin each belongs to is
# ignored on purpose: the mock is ONE block standing in for several real ones. (XML comments are
# dropped by the parser, so the commented-out <sensor> in the xacro is correctly not counted.)
interfaces() {
  python3 - "$1" <<'PY'
import sys, xml.etree.ElementTree as ET
for block in ET.parse(sys.argv[1]).getroot().iter("ros2_control"):
    for comp in block:
        if comp.tag not in ("joint", "sensor", "gpio"):
            continue
        for kind in ("command_interface", "state_interface"):
            for iface in comp.findall(kind):
                print(comp.tag, comp.get("name"), kind.split("_")[0], iface.get("name"))
PY
}

for model in rocko rocko_tank_mode; do
  broken=0
  for mock in false true; do
    label="$model mock=$mock"
    urdf="$work/$model-$mock.urdf"

    if ! xacro "$urdf_dir/$model.urdf.xacro" use_mock_hardware:="$mock" >"$urdf" 2>"$work/err"; then
      fail "$label: xacro could not expand it:"; show "$work/err"; broken=1; continue
    fi
    if ! check_urdf "$urdf" >"$work/out" 2>&1; then
      fail "$label: check_urdf rejected it:"; show "$work/out"; broken=1; continue
    fi

    real_motors=$(count "$urdf" "<plugin>rocko_env/Motor12VoltQuadEncoder</plugin>")
    real_imus=$(count "$urdf" "<plugin>rocko_env/ICM20948</plugin>")
    mocks=$(count "$urdf" "<plugin>mock_components/GenericSystem</plugin>")
    named=$(count "$urdf" 'name="MockRocko"')
    plugins=$(count "$urdf" "<plugin>")   # every plugin of any kind: catches one we did not think to name

    if [ "$mock" = false ]; then
      expected_imus=0; [ "$model" = rocko ] && expected_imus=1   # only balancing mode has the IMU
      expected_plugins=$((2 + expected_imus))
      if [ "$real_motors" -ne 2 ] || [ "$real_imus" -ne "$expected_imus" ] || [ "$mocks" -ne 0 ] || [ "$plugins" -ne "$expected_plugins" ]; then
        fail "$label: expected 2 real motors, $expected_imus IMU, no mock and $expected_plugins plugins in all (got motors=$real_motors imu=$real_imus mock=$mocks plugins=$plugins)"
        broken=1; continue
      fi
    else
      if [ "$mocks" -ne 1 ] || [ "$named" -ne 1 ] || [ "$real_motors" -ne 0 ] || [ "$real_imus" -ne 0 ] || [ "$plugins" -ne 1 ]; then
        fail "$label: expected exactly one MockRocko and no other plugin (got mock=$mocks named=$named motors=$real_motors imu=$real_imus plugins=$plugins)"
        broken=1; continue
      fi
    fi
    echo "ok    $label"
  done

  # The mock must export exactly what the real hardware exports. Skipped if either expansion
  # already failed above. Plain `sort`, not `sort -u`: an interface exported twice is a real
  # difference (the robot would refuse to load it), and `-u` would hide it.
  if [ "$broken" -eq 0 ]; then
    if interfaces "$work/$model-false.urdf" >"$work/real.raw" 2>"$work/err" \
       && interfaces "$work/$model-true.urdf" >"$work/mock.raw" 2>>"$work/err" \
       && [ -s "$work/real.raw" ]; then
      sort "$work/real.raw" >"$work/real.if"
      sort "$work/mock.raw" >"$work/mock.if"
      if diff "$work/real.if" "$work/mock.if" >"$work/diff"; then
        echo "ok    $model: the mock exports the same $(wc -l <"$work/real.if" | tr -d ' ') interfaces as the real hardware"
      else
        fail "$model: the mock and the real hardware export different interfaces (< real, > mock):"
        show "$work/diff"
      fi
    else
      fail "$model: could not read the interfaces out of the expanded descriptions:"; show "$work/err"
    fi
  fi
done

if [ "$failures" -eq 0 ]; then
  echo "PASS: all four combinations expand, parse, hold the right hardware, and the mock matches the real interfaces."
  exit 0
fi
echo "FAIL: $failures problem(s) above."
exit 1
