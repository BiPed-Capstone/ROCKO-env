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
# A fifth and a sixth expansion, balancing mode with use_sim_plant:=true (and again with
# use_mock_hardware:=true as well, where the sim must win), must each hold exactly one SimPlant system
# (SimRocko) and nothing else, and export exactly the real hardware's interfaces too.
# A seventh expansion, TANK mode with use_sim_plant:=true, must still hold the two real motor plugins
# and no SimPlant: tank mode ignores that argument BY DESIGN (rocko_tank_mode.urdf.xacro does not
# declare it, so the simulator can only ever be reached through rocko.urdf.xacro and rocko_sim.launch.py).
# Finally sim/check_sim_config.py checks that the simulator's configuration has not drifted: the sim
# controller YAML keeps the hardware YAML's wiring, and the simulator settings' defaults agree in
# SimPlantModel.hpp, rocko.urdf.xacro and rocko_sim.launch.py.
# check_urdf alone passes whichever block was selected; without these assertions a swapped
# xacro:if / xacro:unless, or a mock that has drifted from the real names, would pass every
# off-Pi check and first show up on the robot.
#
# Exit status:  0 = all seven expansions and the sim config check ok   1 = something wrong
#               2 = could not run (no build)
set -uo pipefail

WS="${ROS2_WS:-$HOME/ros2_ws}"
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

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

# Fifth and sixth expansions: balancing mode with the simulated robot (use_sim_plant:=true, Tier 3),
# without and with use_mock_hardware:=true. use_sim_plant must win, so both must hold exactly one
# SimPlant system and nothing else (no real plugin, no mock), and export exactly the same interfaces
# as the real hardware, compared with the rocko mock=false expansion above.
for sim_mock in false true; do
  label="rocko sim=true mock=$sim_mock"
  urdf="$work/rocko-sim-$sim_mock.urdf"
  if ! xacro "$urdf_dir/rocko.urdf.xacro" use_sim_plant:=true use_mock_hardware:="$sim_mock" >"$urdf" 2>"$work/err"; then
    fail "$label: xacro could not expand it:"; show "$work/err"
  elif ! check_urdf "$urdf" >"$work/out" 2>&1; then
    fail "$label: check_urdf rejected it:"; show "$work/out"
  else
    sims=$(count "$urdf" "<plugin>rocko_env/SimPlant</plugin>")
    named=$(count "$urdf" 'name="SimRocko"')
    real_motors=$(count "$urdf" "<plugin>rocko_env/Motor12VoltQuadEncoder</plugin>")
    real_imus=$(count "$urdf" "<plugin>rocko_env/ICM20948</plugin>")
    mocks=$(count "$urdf" "<plugin>mock_components/GenericSystem</plugin>")
    plugins=$(count "$urdf" "<plugin>")
    if [ "$sims" -ne 1 ] || [ "$named" -ne 1 ] || [ "$real_motors" -ne 0 ] || [ "$real_imus" -ne 0 ] || [ "$mocks" -ne 0 ] || [ "$plugins" -ne 1 ]; then
      fail "$label: expected exactly one SimPlant (SimRocko) and no other plugin (got sim=$sims named=$named motors=$real_motors imu=$real_imus mock=$mocks plugins=$plugins)"
    else
      echo "ok    $label"
      if [ ! -s "$work/rocko-false.urdf" ]; then
        fail "rocko: no real-hardware expansion to compare the sim with (it failed above)"
      elif interfaces "$work/rocko-false.urdf" >"$work/real.raw" 2>"$work/err" \
           && interfaces "$urdf" >"$work/sim.raw" 2>>"$work/err" \
           && [ -s "$work/real.raw" ]; then
        sort "$work/real.raw" >"$work/real.if"
        sort "$work/sim.raw" >"$work/sim.if"
        if diff "$work/real.if" "$work/sim.if" >"$work/diff"; then
          echo "ok    $label: the sim exports the same $(wc -l <"$work/real.if" | tr -d ' ') interfaces as the real hardware"
        else
          fail "$label: the sim and the real hardware export different interfaces (< real, > sim):"
          show "$work/diff"
        fi
      else
        fail "$label: could not read the interfaces out of the sim description:"; show "$work/err"
      fi
    fi
  fi
done

# Seventh expansion: tank mode asked for the simulator. Tank mode does not declare use_sim_plant (xacro
# silently accepts an argument nothing declares), so it must come out as the plain real tank robot:
# the two motor plugins and nothing else. If someone adds the argument to the tank xacro, this is
# where "tank mode now selects the simulator" shows up.
label="rocko_tank_mode sim=true (ignored by design)"
urdf="$work/tank-sim.urdf"
if ! xacro "$urdf_dir/rocko_tank_mode.urdf.xacro" use_sim_plant:=true >"$urdf" 2>"$work/err"; then
  fail "$label: xacro could not expand it:"; show "$work/err"
else
  sims=$(count "$urdf" "<plugin>rocko_env/SimPlant</plugin>")
  real_motors=$(count "$urdf" "<plugin>rocko_env/Motor12VoltQuadEncoder</plugin>")
  mocks=$(count "$urdf" "<plugin>mock_components/GenericSystem</plugin>")
  plugins=$(count "$urdf" "<plugin>")
  if [ "$sims" -ne 0 ] || [ "$real_motors" -ne 2 ] || [ "$mocks" -ne 0 ] || [ "$plugins" -ne 2 ]; then
    fail "$label: expected the two real motor plugins and no SimPlant (got sim=$sims motors=$real_motors mock=$mocks plugins=$plugins)"
  else
    echo "ok    $label"
  fi
fi

# The simulator's configuration must not drift (what each check guards: the script's docstring).
# Exit 1 means drift (lines starting FAIL); exit 2 means the checker could not run at all (a file
# unreadable, or PyYAML missing), which says nothing about drift.
python3 "$here/sim/check_sim_config.py" >"$work/config" 2>&1
config_status=$?
if [ "$config_status" -eq 0 ]; then
  sed 's/^/      /' "$work/config"
  echo "ok    sim config: no drift"
elif [ "$config_status" -eq 1 ]; then
  fail "sim config has drifted:"; show "$work/config"
else
  fail "sim config check could not run (exit $config_status; this is not drift):"; show "$work/config"
fi

if [ "$failures" -eq 0 ]; then
  echo "PASS: all seven expansions (four real/mock, two sim, one tank-with-sim-request) expand, parse, hold the right hardware, the mock and the sim match the real interfaces, tank mode ignores the sim request, and the sim config has not drifted."
  exit 0
fi
echo "FAIL: $failures problem(s) above."
exit 1
