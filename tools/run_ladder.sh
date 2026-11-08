#!/usr/bin/env bash
# Run the whole verification ladder ONCE and print a one-screen table. Run on the LAPTOP, from anywhere:
#   tools/run_ladder.sh              # everything (about 6 to 8 minutes; the sim step is most of it)
#   tools/run_ladder.sh --no-sim     # skip verify_sim.sh (about 2 minutes)
#   tools/run_ladder.sh --clean      # rm -rf build install log in the container first (do this after a checkout switch)
#   tools/run_ladder.sh --host       # only the host tests and the hook tests (seconds, no container)
#
# Why it exists: in the 2026-10 to 2026-11 runs about 29 % of the Bash calls (18 % of all tool calls) were agents running these
# same commands one at a time and reading each result back into a context that had to be re-read on every
# later call. One call to this script returns the same facts in about 25 lines.
# (Measured with tools/session_usage.py; see docs/claude-analysis/agent-loop-efficiency.md.)
#
# What it does NOT do: it does not decide whether a red result is a regression. A red line prints the last
# 15 lines of that step's output and keeps going, so one flaky check does not hide the rest. Full logs are
# kept in $LOG_DIR (default: a fresh folder under $TMPDIR). CLAUDE.md §4 still applies: report the tiers that
# ran, and "not run" for the rest.
#
# Exit status: 0 = every step that ran passed, 1 = at least one failed, 2 = could not run (container mount).
set -u
cd "$(dirname "$0")/.." || exit 2
ROOT="$(pwd)"
DO_SIM=1; DO_CONTAINER=1; DO_CLEAN=0
for arg in "$@"; do
  case "$arg" in
    --no-sim) DO_SIM=0 ;;
    --host) DO_CONTAINER=0 ;;
    --clean) DO_CLEAN=1 ;;
    -h|--help) sed -n '2,18p' "$0"; exit 0 ;;
    *) echo "unknown argument: $arg" >&2; exit 2 ;;
  esac
done
LOG_DIR="${LOG_DIR:-$(mktemp -d "${TMPDIR:-/tmp}/ladder.XXXXXX")}"
CTR=(docker compose -f docker/compose.yml exec -T ros2)
RESULTS=(); FAILED=0

# step <name> <command...>: run, keep the log, record PASS/FAIL, show the tail of a failure.
step() {
  local name="$1"; shift
  local log="$LOG_DIR/${name//[^A-Za-z0-9_.-]/_}.log" start=$SECONDS
  if "$@" >"$log" 2>&1; then
    RESULTS+=("PASS  $name  ($((SECONDS - start)) s)")
  else
    RESULTS+=("FAIL  $name  ($((SECONDS - start)) s)  log: $log")
    FAILED=1
    echo "---- $name failed; last 15 lines ----"; tail -15 "$log"
  fi
}

if [ "$DO_CONTAINER" = 1 ]; then
  ./check_container_mount.sh >"$LOG_DIR/mount.log" 2>&1
  case $? in
    0) RESULTS+=("PASS  container mount: sees this checkout") ;;
    *) cat "$LOG_DIR/mount.log"; echo "RESULT: could not run the container tiers (mount check)"; exit 2 ;;
  esac
  [ "$DO_CLEAN" = 1 ] && step "clean build dirs" "${CTR[@]}" bash -c 'cd /ros2_ws && rm -rf build install log'
  step "Tier 1 build_ros.sh" "${CTR[@]}" bash -c 'cd /ros2_ws && src/ROCKO-env/build_ros.sh'
  step "Tier 0 verify_urdf.sh" "${CTR[@]}" /ros2_ws/src/ROCKO-env/verify_urdf.sh
  step "Tier 2 verify_mock.sh balance" "${CTR[@]}" /ros2_ws/src/ROCKO-env/verify_mock.sh
  step "Tier 2 verify_mock.sh tank" "${CTR[@]}" /ros2_ws/src/ROCKO-env/verify_mock.sh tank
  step "compile-only motor plugin" "${CTR[@]}" bash -c 'cd /ros2_ws/src/ROCKO-env && ./check_motor_plugin_compiles.sh'
  # Compiler warnings in the Tier 1 build are a finding (CLAUDE.md §4): count them separately.
  warn=$(grep -c "warning:" "$LOG_DIR/Tier_1_build_ros.sh.log" 2>/dev/null || true)
  RESULTS+=("INFO  compiler warnings in the Tier 1 build: ${warn:-0}  (CMake notices are not counted)")
  if [ "$DO_SIM" = 1 ]; then
    step "Tier 3 verify_sim.sh (S1 S2a H4 C3 C2 H6 FF)" "${CTR[@]}" /ros2_ws/src/ROCKO-env/verify_sim.sh
    step "Tier 3 verify_sim.sh scenarios (U I V S2b AB)" "${CTR[@]}" /ros2_ws/src/ROCKO-env/verify_sim.sh scenarios
  fi
fi

# Host tests: no ROS, no container. Every C++ test gets every include folder (a test uses the ones it needs).
INC=(-I rocko_env/rocko_env/hardware/sensors/icm20948/include -I rocko_env/rocko_env/hardware/sim/include
     -I rocko_env/rocko_env/controllers/add_feedforward/include)
for t in tools/*_test.cpp; do
  n=$(basename "$t" .cpp)
  step "host test $n" bash -c "g++ -std=c++17 -Wall -Wextra ${INC[*]} '$t' -o '$LOG_DIR/$n.bin' && '$LOG_DIR/$n.bin'"
done
for t in tools/*_test.py; do
  step "host test $(basename "$t")" env PYTHONDONTWRITEBYTECODE=1 python3 -I "$t"
done
step "hook tests" .claude/hooks/test-guard-protected-files.sh
step "sim config drift check" env PYTHONDONTWRITEBYTECODE=1 python3 -I sim/check_sim_config.py
step "doc link scan" env PYTHONDONTWRITEBYTECODE=1 python3 -I tools/check_doc_links.py

echo; echo "=== ladder results ($(git rev-parse --short HEAD), dirty files: $(git status --short | wc -l | tr -d ' ')) ==="
printf '%s\n' "${RESULTS[@]}"
[ "$DO_CONTAINER" = 0 ] && echo "NOT RUN: every container tier (Tier 0 urdf, Tier 1, Tier 2, Tier 3), by request (--host)"
[ "$DO_CONTAINER" = 1 ] && [ "$DO_SIM" = 0 ] && echo "NOT RUN: Tier 3 (--no-sim)"
echo "logs: $LOG_DIR"
if [ "$FAILED" = 0 ]; then echo "RESULT: PASS"; exit 0; else echo "RESULT: FAIL (read the failed steps above; a timing check that failed once may be a known flake: re-run it with tools/repeat_check.sh)"; exit 1; fi
