#!/usr/bin/env bash
# Run one verify_sim.sh check N times in the dev container and print a tally. Run on the LAPTOP:
#   tools/repeat_check.sh C3 20            # check name, count
#   tools/repeat_check.sh scenarios 5      # the scenarios mode
#   SIM_PLANT_OVERRIDES="body_mass_kg:=2.6" tools/repeat_check.sh V 3   # environment is passed through
#
# Why it exists: "run it 20 times and show it is stable" was done by agents one call at a time, with loops they
# wrote on the spot. Two of those loops cost hours: one waited on a `pgrep` pattern that matched its own command
# line (about 7 hours), and one counted runs that exited 2 because a leftover process from the previous run was
# still alive. This script does the cleanup, the loop and the tally the same way every time.
#
# Behaviour:
#   * Before each run it waits (up to 60 s) for the previous run's ros2_control_node to be gone. It finds the
#     process with `pgrep -x ros2_control_node`, an exact name match, so it cannot match its own command line.
#   * A run that exits 2 ("could not run") is counted separately from a failure (1) and is NOT a verdict.
#   * Each run's output is kept in $LOG_DIR/run_<n>.log; the tally lists the last line of every failing run.
#
# Exit status: 0 = every run passed, 1 = at least one run failed, 2 = could not run (mount check, bad usage).
set -u
cd "$(dirname "$0")/.." || exit 2
[ $# -ge 2 ] || { sed -n '2,5p' "$0"; exit 2; }
CHECK="$1"; COUNT="$2"
case "$COUNT" in ''|*[!0-9]*) echo "count must be a whole number" >&2; exit 2 ;; esac
./check_container_mount.sh >/dev/null 2>&1 || { ./check_container_mount.sh; exit 2; }
LOG_DIR="${LOG_DIR:-$(mktemp -d "${TMPDIR:-/tmp}/repeat.XXXXXX")}"
CTR=(docker compose -f docker/compose.yml exec -T)
ENV_ARGS=()
for v in SIM_PLANT_OVERRIDES SCENARIO_METRICS_DIR SCENARIOS_EXPECTED_RED; do
  [ -n "${!v:-}" ] && ENV_ARGS+=(-e "$v=${!v}")
done
pass=0; fail=0; cannot=0; failing=()
for n in $(seq 1 "$COUNT"); do
  for _ in $(seq 1 30); do   # 30 x 2 s = 60 s
    "${CTR[@]}" ros2 pgrep -x ros2_control_node >/dev/null 2>&1 || break
    sleep 2
  done
  log="$LOG_DIR/run_$n.log"
  # ${ENV_ARGS[@]+...}: macOS bash 3.2 treats an empty array as unset under `set -u`.
  "${CTR[@]}" ${ENV_ARGS[@]+"${ENV_ARGS[@]}"} ros2 /ros2_ws/src/ROCKO-env/verify_sim.sh "$CHECK" >"$log" 2>&1
  case $? in
    0) pass=$((pass + 1)) ;;
    2) cannot=$((cannot + 1)); failing+=("run $n: COULD NOT RUN: $(tail -1 "$log")") ;;
    *) fail=$((fail + 1)); failing+=("run $n: FAIL: $(grep -E 'FAIL|failed' "$log" | head -1)") ;;
  esac
done
echo "=== $CHECK x $COUNT ($(git rev-parse --short HEAD)): pass $pass, fail $fail, could not run $cannot ==="
[ ${#failing[@]} -gt 0 ] && printf '%s\n' "${failing[@]}"
echo "logs: $LOG_DIR"
[ "$cannot" -gt 0 ] && exit 2
[ "$fail" -gt 0 ] && exit 1
exit 0
