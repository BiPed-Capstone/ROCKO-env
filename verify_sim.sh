#!/usr/bin/env bash
# Tier 3 check: do the real controllers run against the SIMULATED robot (rocko_env/SimPlant), does
# the simulated pitch respond to the wheels, does the simulated robot fall when its motors give no
# torque, and do the stage 4 safeguards (findings H-4, C-3, C-2, H-6, H-11) do what they say?
#
# Run inside the Docker dev container, after build_ros.sh. From the repo root on your laptop:
#   docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_sim.sh
# Or only some checks, by name (any of S1 S2a H4 C3 C2 H6 FF, in any order):
#   docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_sim.sh H4 C3
#
# THE SIMULATOR IS A MODEL WITH ESTIMATED, UNMEASURED PARAMETERS (docs/architecture/sim-plant.md).
# A pass says the plumbing and the model behave as specified. It says NOTHING about whether the real
# robot balances, and nothing about whether any gain is good.
#
# Each check is one launch of rocko_sim.launch.py, one after another (about 15-25 s each). In every
# run the wiring is checked (the four cascade controllers and joint_state_broadcaster active, the
# SimPlant component active, all 14 interfaces present, both wheel commands claimed) unless the
# check says otherwise. teleop:=false (a script publishes a velocity reference of 0, so the whole
# cascade is running) unless the check says otherwise.
#   S1   sim/sim_probe.py records pitch from /dynamic_joint_states.
#        baseline: upright, no wheel command beyond the cascade's own: pitch must stay within
#                  S1_STILL_DEG of its start (the model's upright equilibrium is exact). A robot that
#                  was never released also stays still, so every probe run must show the robot was
#                  released at least RELEASED_MIN_S before the probe's last sample (below).
#        command:  the same, plus a feedforward of FEEDFORWARD_RAD_S on both wheels: pitch must move
#                  at least S1_CHANGE_DEG. Pitch changes BECAUSE the wheels moved.
#   S2a  disable_torque:=true, initial_pitch_deg:=2.0: |pitch| must pass TIPOVER_DEG within
#        FALL_LIMIT_S of the moment it starts to move.
#   (In S1 command and S2a the robot falls past TIPOVER_DEG, so the tipover cutoff trips; the run
#   prints a note. Neither check depends on it: the fall is already under way when it trips.)
#   H4   tipover cutoff. initial_pitch_deg:=TIPOVER_START_DEG, past the limit from the start. Both
#        left_ and right_add_feedforward_controller must log 'TIPOVER CUTOFF TRIPPED' before the
#        robot is released (so in the first cycle they write); every wheel command written while
#        |pitch| > TIPOVER_DEG must be exactly 0.0 (from controller_manager's introspection topic,
#        the value the plant receives); ONE re-arm published on the shared /tipover_rearm topic
#        while still past the limit must be refused by BOTH controllers and leave the command at
#        0.0; the controllers must stay active (the cutoff does not deactivate them).
#        (The re-arm hysteresis, refusing a re-arm inside the trip limit but outside
#        rearm_max_pitch_deg, is checked by the host test tools/tipover_latch_test.cpp, not here.)
#   C3   teleop watchdog. teleop:=true (the real DiffDriveController.py; Joystick.py is started too
#        but runs idle, because no gamepad connects to its TCP port: that path is covered by the host
#        test tools/teleop_watchdog_test.py). A fake gamepad publishes TwistStamped messages
#        straight to diffbot_base_controller/cmd_vel at 20 Hz for 2 s, then stops. command_timed_out must go False while it
#        drives and True TELEOP_TIMEOUT_S to TELEOP_TIMEOUT_S + TELEOP_MARGIN_S after the last
#        command; the velocity reference must be exactly [0, 0] after that; the controllers must
#        stay active and the plant keep publishing. DiffDriveController's log must show 'teleop
#        commands resumed' BEFORE its first 'teleop command timed out' (a time-out logged before
#        any command arrived would not be evidence of the watchdog firing).
#   C2   IMU fault, two runs. initial_pitch_deg:=2.0 (so a true pitch of exactly 0.0 cannot happen)
#        and an outage IMU_FAULT_START_S after release.
#        short: IMU_FAULT_SHORT_S, fewer failed reads than the C-2 limit (5 in a row): no ERROR, the
#               controllers stay active, and the exported pitch is never 0.0 or NaN.
#        long:  IMU_FAULT_LONG_S, more than the limit: SimPlant's read() returns ERROR, which
#               controller_manager must answer by deactivating the hardware, and the exported pitch
#               is never 0.0 or NaN up to then. The wiring is NOT required to stay up here.
#   H6   initial_pitch_deg:=2.0. pitch_pid_controller's feedback_dot (its D term's derivative) must
#        be finite, reach H6_MIN_RATE_DEG_S, equal the exported pitch_rate of the same cycle within
#        H6_TOLERANCE_DEG_S, and error_dot must be -feedback_dot (rate reference 0).
#   FF   feedforward age limit (finding H-11). A script publishes FEEDFORWARD_RAD_S on both
#        /left_feedforward and /right_feedforward at FF_RATE_HZ for FF_PUBLISH_S, then is killed (the
#        stand-in for DiffDriveController.py dying). Each of left_ and right_add_feedforward_controller
#        must log 'Feedforward on ... ms old' exactly once after the kill, between FF_EARLIEST_S and
#        FF_AGE_S + FF_MARGIN_S after it, never before the publisher started, and the controllers
#        must stay active (only the feedforward term goes to 0.0). A time-out WHILE the publisher
#        runs is printed as a note, not judged: it is a real delivery gap (if it came less than
#        FF_LOG_GAP_S before the kill and no line follows the kill, FF exits 2, could not judge,
#        because the controller rate-limits these lines). What the wheel command
#        then is, is not judged here: the host test tools/feedforward_timeout_test.cpp checks the value.
#
# Exit status:  0 = PASS   1 = FAIL (what was wrong is printed)
#               2 = could not run (no build, a controller_manager already running, an unknown
#                   check name) or could not judge
# On anything but PASS the run directory (launch logs, probe output for each run) is KEPT, and its
# path inside the container is printed: it is the evidence of the failure.
#
# THE BALANCE SCENARIOS (bootstrap-sim step 7) are a SEPARATE MODE, never part of the default run:
#   docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_sim.sh scenarios
#   ... verify_sim.sh scenarios U S2b      (only some, any of U I V S2b AB)
# The default run above checks plumbing and safeguards and must stay green; the scenarios ask whether
# the controllers HOLD the simulated robot, which with the current gains they do not. Kept apart so a
# red scenario never hides a red plumbing check, and a green plumbing run never claims balance.
#   U    hold upright: initial_pitch_deg:=U_START_DEG, zero velocity reference, U_RECORD_S. PASS if
#        |pitch| never reaches TIPOVER_DEG and, from U_SETTLE_FROM_S on, |pitch| <= U_SETTLED_DEG and
#        the mean measured wheel speed <= U_MAX_SPEED_M_S.
#   I    recover from a push: upright start, SimPlant's impulse (I_IMPULSE_DEG_S added to the lean rate
#        at I_IMPULSE_AT_S after release). PASS if the push shows in pitch_rate, |pitch| never reaches
#        TIPOVER_DEG, and |pitch| is back under I_SETTLED_DEG within I_SETTLE_S and stays there.
#   V    follow a velocity step: teleop:=true, a fake gamepad (the C3 path) sends linear.x = 0, then
#        V_STEP_M_S. PASS if the mean measured wheel velocity reaches V_RISE_FRACTION of the step
#        within V_RISE_S, overshoots by at most V_OVERSHOOT_PCT, and |pitch| stays under V_PITCH_DEG.
#   S2b  the verifier's own check: U again with pitch_sign:=+1 (the pitch loop's sign flipped). U must
#        FAIL, and for the right reason: the wiring is up and |pitch| reaches TIPOVER_DEG within
#        S2B_FALL_S of release. S2b is "ok" only then.
#   AB   characterisation, "as-built signs": U with encoder_sign:=+1 (the sim's default, the
#        authors' intent) for AB_RECORD_S. EXPECTED TO FALL by the slow run-away of finding H-12 (the
#        outer loop is positive feedback at DC for any positive velocity gains). "ok" if it falls no
#        sooner than AB_MIN_FALL_S; the time to fall is printed. A run that does NOT fall is a FAIL:
#        the signs or the gains changed, and finding H-12's docs must be revisited.
# U, I, V and S2b run with encoder_sign:=SCENARIO_ENCODER_SIGN, an ASSUMPTION (see its comment).
# Scenarios exit status: 0 only if U, I and V pass AND S2b and AB are ok; 1 if any is not; 2 if a run
# could not be judged and nothing failed. While SCENARIOS_EXPECTED_RED=1 a failing U, I or V is
# printed "FAIL (expected until the sim gains are tuned)", but it still counts: the exit status is 1.
#
# No `set -e`: grep and timeout are EXPECTED to "fail" while we wait for things to come up.
set -uo pipefail

ALL_CHECKS="S1 S2a H4 C3 C2 H6 FF"
ALL_SCENARIOS="U I V S2b AB"
mode=checks
known="$ALL_CHECKS"
if [ "${1:-}" = scenarios ]; then
  mode=scenarios; known="$ALL_SCENARIOS"; shift
fi
selected="${*:-$known}"
for c in $selected; do
  [[ " $known " == *" $c "* ]] || { echo "ERROR: unknown $mode name '$c' (known: $known)" >&2; exit 2; }
done
wants() { [[ " $selected " == *" $1 "* ]]; }

# --- Timing. Generous bounds, as in verify_mock.sh; bring-up time in the container is not measured.
STARTUP_TIMEOUT=45   # s to wait for every controller to become active
CALL_TIMEOUT=8       # s for one `ros2 control` / `ros2 topic` call; without a timeout it can wait forever
GRACE=15             # s to let a launch shut down before forcing it
PROBE_SECONDS=8      # s of /dynamic_joint_states the probe records in each run (from its first
                     #   message; the robot is released a second or two later, when the cascade
                     #   activates, so at least ~5 s of each run is after release)
RELEASED_MIN_S=3     # s: the release must be logged at least this long before the probe's last sample, so
                     #   a pass is never a robot that stood still because it was never released

# --- Thresholds. Why each number (measured values are in the stage 3 and stage 4 journals):
FEEDFORWARD_RAD_S=5.0  # [rad/s] wheel command added by S1's command run: 6 % duty through the
                       #   write() copy, well clear of its 0.05 rad/s deadband, far from saturation
S1_STILL_DEG=0.01      # [deg] the baseline must stay closer than this to its start. The model's
                       #   upright equilibrium is exact (unit test #10), so the measured baseline is 0
S1_CHANGE_DEG=0.5      # [deg] the command run must move at least this much: 50x S1_STILL_DEG, and
                       #   far below what the command produces (several degrees, measured). A plant
                       #   whose pitch ignored the wheels would give exactly 0
TIPOVER_DEG=35         # [deg] the tipover angle of finding H-4; also tipover_pitch_deg's default
FALL_LIMIT_S=0.48      # [s] the model falls 2 -> 35 deg in 0.4273 s with no torque (unit test #6);
                       #   the probe sees it to within one 10 ms cycle. 0.48 adds ~40 ms for timing
                       #   jitter and stays below 0.50 s, the fall time with the motors merely
                       #   braked at 0 % duty, so "torque not disabled" cannot pass
TIPOVER_START_DEG=40   # [deg] H4's start: 5 deg past the limit, so the cutoff must trip in the first
                       #   cycle. Released there, the model falls on to 60 deg ("lying on the floor")
                       #   and stays past the limit, so the re-arm must be refused
TIPOVER_RECORD_S=16    # [s] H4 records the commands this long after the controllers start writing:
                       #   time for the wiring check and the re-arm, plus REARM_OBSERVED_S after it.
                       #   Measured 2026-11-02: the refusal came 11.4 s after the first write (the
                       #   wiring check, then `ros2 topic pub -w 2` waiting ~3 s for discovery to find
                       #   BOTH subscribers), so 8 s was too short; 16 s leaves ~4 s of margin
REARM_OBSERVED_S=1     # [s] the commands must be seen at 0.0 at least this long after the refusal
REARM_REPLY_WAIT_S=5   # [s] wait this long for each controller's 'Tipover re-arm refused' line: the
                       #   next 10 ms cycle answers, so this is all DDS delivery and log flushing
TELEOP_TIMEOUT_S=0.2   # [s] DiffDriveController's command_timeout_s default (and the launch arg's)
TELEOP_MARGIN_S=0.1    # [s] allowed lateness of the timeout: the node checks every 10 ms, plus
                       #   Python executor and DDS delivery in the container; 0.1 s is 10 cycles
TELEOP_SPEED_M_S=0.05  # [m/s] the fake gamepad's linear.x: small, but exactly representable as a
                       #   reference that is visibly not 0
IMU_FAULT_START_S=1.0  # [s after release] when the injected IMU outage starts
IMU_FAULT_SHORT_S=0.03 # [s] 3 failed reads at 100 Hz: below the C-2 limit of 5 in a row
IMU_FAULT_LONG_S=0.2   # [s] 20 failed reads: past the limit at the 5th, so read() returns ERROR
IMU_RECORD_S=3         # [s after release] covers the outage and 1.8 s after it
H6_RECORD_S=3          # [s] H6 records controller_state and the plant this long: ~300 cycles
MIN_SAMPLES=50         # [samples] the fewest a scenario check judges from: 0.5 s at 100 Hz
H6_MIN_RATE_DEG_S=1.0  # [deg/s] |feedback_dot| must reach this: from a 2 deg start the model's pitch
                       #   rate is tens of deg/s, while a D term that reads no rate gives exactly 0
H6_TOLERANCE_DEG_S=1e-9  # [deg/s] pid_controller copies the state value; only float noise allowed
FF_PUBLISH_S=4         # [s] FF publishes the feedforward this long before killing the publisher;
                       #   `ros2 topic pub` needs a second or two to start, so >= 2 s of messages
FF_AGE_S=0.2           # [s] kMaxFeedforwardAgeNs in AddFeedforwardController.hpp (200 ms). Keep equal
FF_RATE_HZ=100         # [Hz] the rate DiffDriveController.py publishes at (timer_period 0.01 s). At
                       #   20 Hz one run (2026-11-03) saw a 202 ms gap in delivery to one controller
                       #   while the publisher was running, a false time-out; at the real node's rate
                       #   a 200 ms gap needs 20 lost messages, not 4
FF_EARLIEST_S=0.1      # [s] the time-out must not be logged sooner than this after the kill. The last
                       #   message left at most one 10 ms publish period before it, so a correct
                       #   200 ms limit logs about 0.19-0.21 s after; a limit of about 100 ms or less,
                       #   or a time-out while messages were still arriving, logs before 0.1 s
FF_MARGIN_S=0.3        # [s] allowed lateness after FF_AGE_S: one 10 ms cycle, plus the kill and DDS
                       #   in the container (generous, as TELEOP_MARGIN_S)
FF_LOG_GAP_S=1.0       # [s] kTimeoutLogMinCycles (100) at update_rate 100: after a logged time-out,
                       #   the next one is counted but not logged for this long. A delivery gap
                       #   this close before the kill can hide the kill's line: FF cannot judge then

# --- Scenario thresholds (`verify_sim.sh scenarios`): the ONE place they are written. COMMITTED
# BEFORE ANY TUNING (stage 5 journal, docs/journal/2026-11-03-stage5-balance-in-sim.md): change one
# only with a reason written here and in the journal, never to make a run pass. Frames and signs:
# pitch [deg] is the EXPORTED pitch the controllers read (0 = upright; the sign convention of
# sim-plant.md §2.4; limits are on |pitch|, so either sign). "Measured wheel velocity" [m/s] is the
# exported `velocity` state the velocity loop reads, averaged over both wheels: about HALF the true
# speed (finding U-2) and signed by encoder_sign. Times [s] count from SimPlant's release (its
# "releasing the simulated robot" log line), the zero its own impulse time uses. Sizes were checked
# against sim/cascade_linearisation.py's linear model at one stable point of its C3 map (pitch d 0.3,
# velocity p 8; NOT a gain recommendation): they are reachable by a working loop, not tailored to one.
SCENARIOS_EXPECTED_RED=1  # 1 until the commit that first makes U, I and V pass; set it to 0 there.
                          #   Only changes the wording of a FAIL; the exit status is 1 either way
SCENARIO_ENCODER_SIGN=-1  # ASSUMPTION, NOT CHECKED ON THE ROBOT: the encoder reads NEGATIVE when a
                          #   positive wheel command drives the robot. Chosen because with every gain
                          #   positive it is the only encoder sign under which the outer loop can be
                          #   negative feedback (finding H-12; sim-plant.md §2.4); the sim default (+1,
                          #   the authors' intent) is kept in AB. Every scenario result carries this
                          #   label. Proposal 0006 asks a human to settle the real sign
U_START_DEG=2.0        # [deg] U's initial pitch: small, the size used by C2 and H6; from exactly 0 the
                       #   model's equilibrium is exact (unit test #10) and nothing would happen
U_RECORD_S=10          # [s] "hold upright for 10 s" (bootstrap-sim step 7)
U_SETTLE_FROM_S=5      # [s] U judges its last 5 s: a working loop settles a 2 deg offset well inside
                       #   5 s (the linear model's stable point: < 0.02 deg after 5 s), while the
                       #   slow run-aways seen so far (time constants 1-10 s) are large by then
U_SETTLED_DEG=2.0      # [deg] after settling the robot may lean no more than it started: a loop that
                       #   ends up leaning further than its 2 deg start is not holding it
U_MAX_SPEED_M_S=0.05   # [m/s, measured, so ~0.1 m/s true] told to stand still, it must not roll off.
                       #   Half V's step; a stage 3 run with the as-built signs read -2.0 m/s at 10 s
TIPOVER_SCENARIO_DEG=$TIPOVER_DEG  # [deg] a fall: H-4's cutoff angle (35), the point of no return
I_IMPULSE_AT_S=2.0     # [s] the push comes 2 s after release, with the robot standing still at
                       #   exactly upright (no other disturbance in I)
I_IMPULSE_DEG_S=100    # [deg/s] added once to the lean rate (sim-plant.md §5: exported pitch_rate
                       #   jumps by +100). The linear model's stable point peaks at 3.4 deg, so it
                       #   is large enough that "back under 2 deg" means something, and far from 35
I_SETTLED_DEG=2.0      # [deg] the same "settled" lean as U
I_SETTLE_S=3.0         # [s after the push] back under I_SETTLED_DEG this soon (and stays): the linear
                       #   model's stable point needs 0.6 s, so 3 s is a generous bound
I_RECORD_S=8           # [s after release] 6 s after the push: twice I_SETTLE_S, to see it stay
V_STEP_M_S=0.1         # [m/s] linear.x of the step, a slow walk. DiffDriveController makes it the
                       #   velocity reference of both wheels (and a feedforward of 0.1/r rad/s); the
                       #   loop drives the MEASURED velocity to it, so the true speed is ~0.2 m/s (U-2)
V_BEFORE_S=1.0         # [s after release] linear.x = 0 this long, then the step
V_STEP_S=8.0           # [s] the step is held this long (the gamepad keeps sending, 20 Hz, so the
                       #   C-3 watchdog never fires)
V_RISE_FRACTION=0.9    # 90 % of the step
V_RISE_S=3.0           # [s after the step] to reach 90 %. A balancing robot must first lean the
                       #   other way, then accelerate; 3 s to reach a slow walk is the slowest that
                       #   still reads as "following" (a 10 s slow mode would take ~20 s and fail)
V_OVERSHOOT_PCT=30     # [% of the step] peak measured velocity at most 1.3 x the step
V_PITCH_DEG=10.0       # [deg] a 0.1 m/s step needs a lean of well under a degree in steady state; 10
                       #   deg is far below the 35 deg trip and equals the re-arm band (H-4)
S2B_FALL_S=1.0         # [s] with the pitch loop's sign flipped the linear model diverges at ~20 /s
                       #   (1.218 per cycle, sim-plant.md §7.2): 2 -> 35 deg in ~0.2 s. 1.0 s is
                       #   generous and far below AB_MIN_FALL_S, so a slow drift cannot pass for it
AB_RECORD_S=20         # [s] long enough to see the as-built fall (9.3-10.0 s observed so far)
AB_MIN_FALL_S=3.0      # [s] AB must fall by a slow run-away (linear model: a real pole at +0.82 /s),
                       #   not at once: a fall before 3 s would be a different fault

WS="${ROS2_WS:-$HOME/ros2_ws}"
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ROS's setup scripts read variables that may be unset, so `set -u` is off while sourcing them.
set +u
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source "$WS/install/setup.bash" 2>/dev/null \
  || { echo "ERROR: no build in $WS/install. Run src/ROCKO-env/build_ros.sh first." >&2; exit 2; }
set -u

# Keep this run off other machines' ROS graphs, and refuse to run next to another controller
# manager: `ros2 control` talks to whichever one answers.
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
command -v pgrep >/dev/null || { echo "ERROR: pgrep (procps) is missing." >&2; exit 2; }
controller_manager_running() { pgrep -f ros2_control_node >/dev/null; }
if controller_manager_running; then
  echo "ERROR: a controller_manager (ros2_control_node) is already running. Stop it first." >&2
  exit 2
fi

controllers="joint_state_broadcaster left_add_feedforward_controller right_add_feedforward_controller pitch_pid_controller velocity_pid_controller"
states=""
for j in left_wheel_joint right_wheel_joint; do
  for s in position velocity yaw pitch roll pitch_rate; do states="$states $j/$s"; done
done
commands="left_wheel_joint/velocity right_wheel_joint/velocity"

out="$(mktemp -d)"
groups=()   # process-group ids started by this script; cleanup signals each whole group
# start_group <log> <command...>: run it in its own process group (setsid; see verify_mock.sh).
start_group() { local log="$1"; shift; setsid "$@" >"$log" 2>&1 & groups+=("$!"); }
stop_groups() {
  # SIGTERM first (a background job ignores SIGINT), then force, so this script never hangs.
  for g in "${groups[@]}"; do kill -TERM -- "-$g" 2>/dev/null; done
  for _ in $(seq 1 "$GRACE"); do
    alive=0; for g in "${groups[@]}"; do kill -0 "$g" 2>/dev/null && alive=1; done
    [ "$alive" -eq 0 ] && break; sleep 1
  done
  for g in "${groups[@]}"; do kill -KILL -- "-$g" 2>/dev/null; wait "$g" 2>/dev/null; done
  groups=()
  # The next run refuses to start next to a leftover controller manager, so wait for it to go.
  for _ in $(seq 1 "$GRACE"); do controller_manager_running || return 0; sleep 1; done
  return 1
}
passed=0
cleanup() {
  stop_groups
  if [ "$passed" -eq 1 ]; then
    rm -rf "$out"
  else
    echo "Run directory KEPT as evidence (inside the dev container): $out" >&2
  fi
}
trap cleanup EXIT

# has_interface <command|state> <name> [text the line must also contain]. As in verify_mock.sh:
# velocity is both a command and a state interface, so awk tracks which section a line is in.
has_interface() {
  awk -v sec="$1" -v name="$2" -v flag="${3:-}" '
    /^command interfaces/ { cur = "command"; next }
    /^state interfaces/   { cur = "state";   next }
    cur == sec && $1 == name && (flag == "" || index($0, flag)) { found = 1 }
    END { exit !found }' <<<"$ifaces"
}

# wiring_problems: look once; fills `problems` (empty = all is well).
wiring_problems() {
  problems=()
  ctl="$(timeout -k 2 "$CALL_TIMEOUT" ros2 control list_controllers 2>/dev/null)"
  ifaces="$(timeout -k 2 "$CALL_TIMEOUT" ros2 control list_hardware_interfaces 2>/dev/null)"
  hw="$(timeout -k 2 "$CALL_TIMEOUT" ros2 control list_hardware_components 2>/dev/null)"
  for c in $controllers; do
    grep -Eq "^$c[[:space:]].*[[:space:]]active([[:space:]]|$)" <<<"$ctl" \
      || problems+=("controller '$c' is not active")
  done
  grep -q "plugin name: rocko_env/SimPlant" <<<"$hw" \
    || problems+=("no hardware component uses the rocko_env/SimPlant plugin")
  grep -q "label=active" <<<"$hw" || problems+=("the SimPlant hardware component is not active")
  for s in $states; do
    has_interface state "$s" || problems+=("state interface '$s' is not exported")
  done
  for c in $commands; do
    has_interface command "$c" "[claimed]" || problems+=("command interface '$c' is not claimed")
  done
}

# wait_for_wiring: look until the wiring is right or STARTUP_TIMEOUT passes; `problems` holds the
# result of the last look.
wait_for_wiring() {
  local deadline=$((SECONDS + STARTUP_TIMEOUT))
  wiring_problems
  while [ ${#problems[@]} -ne 0 ] && [ $SECONDS -lt $deadline ]; do sleep 1; wiring_problems; done
}

# log_stamp <log> <extended regex>: the [<epoch seconds>] stamp of the first matching log line
# (ROS log lines look like "[node-1] [ERROR] [1793667068.42] [logger]: text"), or nothing.
# Example: "[ros2_control_node-1] [WARN] [1793667068.42] [x]: hi"  ->  1793667068.42
log_stamp() {
  grep -m1 -E "$2" "$1" | sed -n 's/.*\[\(INFO\|WARN\|ERROR\)\] \[\([0-9]*\.[0-9]*\)\].*/\2/p'
}
# wait_for_log <log> <extended regex> <seconds>: true as soon as the log has a matching line.
wait_for_log() {
  local deadline=$((SECONDS + $3))
  until grep -Eq "$2" "$1"; do [ $SECONDS -ge $deadline ] && return 1; sleep 0.5; done
}
# metric <metrics.json> <key>: one value from a probe's metrics.json, or nothing.
metric() { python3 -c 'import json, sys; print(json.load(open(sys.argv[1]))[sys.argv[2]])' "$1" "$2" 2>/dev/null; }
# seconds_between <earlier stamp> <later stamp> <at least>: true if later - earlier >= at least.
seconds_between() { awk -v a="$1" -v b="$2" -v m="$3" 'BEGIN {exit !(b - a >= m)}'; }

# released_problem <launch log> <metrics.json>: prints why there is no evidence that the robot was
# released at least RELEASED_MIN_S before the probe's LAST SAMPLE, or nothing if there is. The
# release time is the [INFO] [<epoch seconds>] stamp of SimPlant's log line; the last sample's time
# is last_stamp_s in metrics.json, from the same system clock (not a time taken by this script after
# the run, which would count the seconds spent in the wiring checks as released time).
released_problem() {
  local release last
  release="$(log_stamp "$1" "releasing the simulated robot")"
  last="$(metric "$2" last_stamp_s)"
  if [ -z "$release" ]; then
    echo "SimPlant never logged 'releasing the simulated robot': it was held still the whole run"
  elif [ -z "$last" ]; then
    echo "the probe's metrics.json has no last_stamp_s, so the release time cannot be compared with the last sample"
  elif ! seconds_between "$release" "$last" "$RELEASED_MIN_S"; then
    echo "SimPlant released the robot less than $RELEASED_MIN_S s before the probe's last sample (release $release, last sample $last)"
  fi
}

# --- Shared globals. Bash functions cannot return lists, so these are set by one function and read
# by others. Each is written only where named here:
#   out        the run directory for this whole script (mktemp, above); kept on failure
#   groups     process groups to stop at the end of a run (start_group, stop_groups)
#   label, dir the current run's name and its directory under $out (new_run)
#   problems   what is wrong with the wiring, empty if nothing (wiring_problems, wait_for_wiring)
#   ctl, ifaces, hw  the last `ros2 control` answers, for printing (wiring_problems)
#   check_pid  pid of the scenario_checks.py started by start_check
#   failed, could_not  set to 1 by any run that failed / could not be judged; decide the exit status
#   passed     set to 1 only at the very end; cleanup deletes $out only then
# Each check keeps its own findings in a local array called `failures`, passed to judge.
failed=0
could_not=0
# new_run <label>: the run's name ($label) and its directory ($dir), for everything below.
new_run() { label="$1"; dir="$out/$label"; mkdir -p "$dir"; }
# launch <launch args...>: announce the run and start rocko_sim.launch.py.
launch() {
  echo "=== $label: rocko_sim.launch.py $*"
  start_group "$dir/launch.log" ros2 launch rocko_env rocko_sim.launch.py "$@"
}
# zero_reference: a velocity reference of 0, so velocity_pid_controller (and so the whole cascade)
# computes. Published at 20 Hz, not once: a single message sent before the controller subscribes
# is lost.
zero_reference() {
  start_group "$dir/reference.log" ros2 topic pub -r 20 /velocity_pid_controller/reference \
    control_msgs/msg/MultiDOFCommand "{dof_names: [left_wheel_joint, right_wheel_joint], values: [0.0, 0.0]}"
}
# start_check <subcommand> <args...>: sim/scenario_checks.py in the background; its pid in $check_pid.
start_check() {
  start_group "$dir/check.log" python3 "$here/sim/scenario_checks.py" "$@" --out "$dir" \
    --wait "$STARTUP_TIMEOUT" --min-samples "$MIN_SAMPLES"
  check_pid="$!"
}
# judge <check status> [problem ...]: print the result of a run from the check's exit status and
# the problems found by this script, then shut the run down.
judge() {
  local status="$1"; shift
  [ -s "$dir/check.log" ] && sed 's/^/    /' "$dir/check.log"
  if [ $# -ne 0 ]; then
    echo "  FAIL ($label):"; printf '    - %s\n' "$@"
    echo "  --- launch log, lines mentioning an error"
    grep -E "ERROR|FATAL|died|[Ff]ailed|Traceback" "$dir/launch.log" | tail -20 | sed 's/^/    /'
    failed=1
  elif [ "$status" -eq 0 ]; then
    echo "  ok   ($label)"
  elif [ "$status" -eq 2 ]; then
    echo "  COULD NOT JUDGE ($label): see the reason above"; could_not=1
  else
    echo "  FAIL ($label): see the reason above"; failed=1
  fi
  end_run
}
end_run() {
  if ! stop_groups; then
    echo "ERROR: the controller_manager from '$label' did not exit; cannot start the next run." >&2
    exit 2
  fi
}
# note_tipover: say so when the tipover cutoff tripped in a run that does not test it.
note_tipover() {
  grep -q "TIPOVER CUTOFF TRIPPED" "$dir/launch.log" \
    && echo "    note: the tipover cutoff tripped in this run (the robot passed $TIPOVER_DEG deg)"
}

# run <label> <feedforward rad/s, or "-" for none> <probe mode> <probe threshold> <launch args...>
# S1 and S2a: sim/sim_probe.py watches the pitch.
run() {
  new_run "$1"
  local ff="$2" mode="$3" threshold="$4"; shift 4
  # The probe first, so it is listening before the cascade starts and the robot is released.
  start_group "$dir/probe.log" python3 "$here/sim/sim_probe.py" --duration "$PROBE_SECONDS" \
    --out "$dir" --mode "$mode" --threshold-deg "$threshold" --within "$FALL_LIMIT_S" \
    --wait "$STARTUP_TIMEOUT"
  local probe_pid="$!"
  launch teleop:=false "$@"
  zero_reference
  if [ "$ff" != "-" ]; then
    for side in left right; do
      start_group "$dir/ff_$side.log" ros2 topic pub -r 20 "/${side}_feedforward" \
        std_msgs/msg/Float64 "{data: $ff}"
    done
  fi

  wait_for_wiring
  wait "$probe_pid"; local probe_status=$?
  # Look at the wiring once more after the probe: it must still be up at the end of the run.
  [ ${#problems[@]} -eq 0 ] && wiring_problems
  sed 's/^/    /' "$dir/probe.log"
  note_tipover

  # No metrics.json means the probe saw no message at all; it then reports "could not judge" itself.
  local not_released=""
  [ -s "$dir/metrics.json" ] && not_released="$(released_problem "$dir/launch.log" "$dir/metrics.json")"
  if [ ${#problems[@]} -ne 0 ]; then
    echo "  FAIL ($label): the wiring is not right:"
    printf '    - %s\n' "${problems[@]}"
    echo "  --- ros2 control list_controllers"; echo "${ctl:-<no answer>}" | sed 's/^/    /'
    echo "  --- launch log, lines mentioning an error"
    grep -E "ERROR|FATAL|died|[Ff]ailed|Traceback" "$dir/launch.log" | tail -20 | sed 's/^/    /'
    failed=1
  elif [ -n "$not_released" ]; then
    echo "  FAIL ($label): no evidence the robot was released during the probe window: $not_released"
    failed=1
  elif [ "$probe_status" -eq 0 ]; then
    echo "  ok   ($label)"
  elif [ "$probe_status" -eq 2 ]; then
    echo "  COULD NOT JUDGE ($label): see the probe's reason above"; could_not=1
  else
    echo "  FAIL ($label): see the probe's reason above"; failed=1
  fi
  end_run
}

# H4: the tipover cutoff trips at once, zeroes both wheel commands, refuses a re-arm while the robot
# is still past the limit, and leaves the controllers active.
check_tipover() {
  new_run "H4-tipover"
  start_check tipover --limit-deg "$TIPOVER_DEG" --record "$TIPOVER_RECORD_S"
  local pid="$check_pid"
  launch teleop:=false initial_pitch_deg:="$TIPOVER_START_DEG"
  zero_reference
  local failures=() side log="$dir/launch.log"
  wait_for_wiring
  for side in left right; do
    wait_for_log "$log" "\[${side}_add_feedforward_controller\]: TIPOVER CUTOFF TRIPPED" "$STARTUP_TIMEOUT" \
      || failures+=("${side}_add_feedforward_controller never logged 'TIPOVER CUTOFF TRIPPED'")
  done
  # Tripped in the first cycle the controllers wrote: before the cycle's write() released the robot
  # (the same process writes both lines, so their order in the log is the order they happened).
  local trip_line release_line
  wait_for_log "$log" "releasing the simulated robot" "$REARM_REPLY_WAIT_S"
  trip_line="$(grep -n -m1 "TIPOVER CUTOFF TRIPPED" "$log" | cut -d: -f1)"
  release_line="$(grep -n -m1 "releasing the simulated robot" "$log" | cut -d: -f1)"
  if [ -z "$release_line" ]; then
    # Without a release there is no order to compare, and a robot that was never released proves nothing.
    failures+=("SimPlant never released the robot ('releasing the simulated robot' is not in the launch log)")
  elif [ -n "$trip_line" ] && [ "$trip_line" -gt "$release_line" ]; then
    failures+=("the cutoff tripped only after the robot was released, not in the first cycle")
  fi
  # ONE deliberate re-arm on the shared topic, while the robot is still past the limit: BOTH wheel
  # controllers must receive it and refuse it. -w 2: publish only once both have subscribed.
  timeout -k 2 "$CALL_TIMEOUT" ros2 topic pub --once -w 2 /tipover_rearm \
    std_msgs/msg/Empty "{}" >"$dir/rearm.log" 2>&1
  local publish_status=$?
  if [ "$publish_status" -ne 0 ]; then
    # Not the controllers' fault: the message may never have been sent, so do not wait for replies
    # to it (that would blame the cutoff for a slow DDS discovery).
    failures+=("publishing /tipover_rearm failed or timed out (exit $publish_status; discovery slow? see $dir/rearm.log): the re-arm was not tested")
  else
    for side in left right; do
      wait_for_log "$log" "\[${side}_add_feedforward_controller\]: Tipover re-arm refused" "$REARM_REPLY_WAIT_S" \
        || failures+=("${side}_add_feedforward_controller did not log 'Tipover re-arm refused' after the one /tipover_rearm message")
    done
  fi
  wait "$pid"; local status=$?
  # The evidence, shortened: who tripped, and the refused re-arm. The sed keeps the logger name, e.g.
  # "[ros2_control_node-1] [ERROR] [1793.1] [left_add_feedforward_controller]: TIPOVER ..."
  #   ->  "    log: left_add_feedforward_controller: TIPOVER ..."
  grep -E "TIPOVER CUTOFF TRIPPED|Tipover re-arm refused|RE-ARMED" "$log" \
    | sed 's/^.*\] \[\([a-z_]*\)\]: /    log: \1: /' | cut -c1-140
  grep -q "RE-ARMED" "$log" && failures+=("a controller logged 'RE-ARMED' although the robot was past the limit")
  # After the refusal the commands must have been watched (at 0.0) for REARM_OBSERVED_S more.
  local refused last watched_after_refusal=0
  refused="$(log_stamp "$log" "Tipover re-arm refused")"
  last="$(metric "$dir/metrics.json" last_stamp_s)"
  if [ -n "$refused" ] && [ -n "$last" ] && seconds_between "$refused" "$last" "$REARM_OBSERVED_S"; then
    watched_after_refusal=1
  fi
  if [ -n "$refused" ] && [ "$watched_after_refusal" -eq 0 ]; then
    failures+=("the commands were not watched for $REARM_OBSERVED_S s after the refused re-arm (refused $refused, last sample ${last:-none})")
  fi
  # The cutoff zeroes the command; it must not deactivate anything.
  [ ${#problems[@]} -eq 0 ] && wiring_problems
  [ ${#problems[@]} -ne 0 ] && failures+=("after the trip: ${problems[*]}")
  judge "$status" "${failures[@]}"
}

# C3: the real teleop nodes; a fake gamepad drives, then goes silent.
check_teleop() {
  new_run "C3-watchdog"
  launch teleop:=true
  # No zero_reference here: DiffDriveController publishes the velocity reference.
  wait_for_wiring
  local failures=() log="$dir/launch.log"
  [ ${#problems[@]} -ne 0 ] && failures+=("before the gamepad: ${problems[*]}")
  start_check teleop --timeout-s "$TELEOP_TIMEOUT_S" --margin-s "$TELEOP_MARGIN_S" \
    --speed-m-s "$TELEOP_SPEED_M_S"
  wait "$check_pid"; local status=$?
  # The time-out line counts only if it comes AFTER a command had arrived ('resumed'): a time-out
  # logged at start-up, before anyone drove, would make this pass whatever the watchdog does.
  local resumed_line timed_out_line
  resumed_line="$(grep -n -m1 "teleop commands resumed" "$log" | cut -d: -f1)"
  timed_out_line="$(grep -n -m1 "teleop command timed out: zeroing the velocity reference" "$log" | cut -d: -f1)"
  if [ -z "$resumed_line" ]; then
    failures+=("DiffDriveController never logged 'teleop commands resumed'")
  elif [ -z "$timed_out_line" ]; then
    failures+=("DiffDriveController never logged 'teleop command timed out'")
  elif [ "$timed_out_line" -lt "$resumed_line" ]; then
    failures+=("DiffDriveController logged 'teleop command timed out' before any command had arrived (log line $timed_out_line, 'resumed' at line $resumed_line)")
  fi
  # The watchdog zeroes the reference; the balance loop must keep running.
  wiring_problems
  [ ${#problems[@]} -ne 0 ] && failures+=("after the time-out: ${problems[*]}")
  note_tipover
  judge "$status" "${failures[@]}"
}

# C2: an injected IMU outage, shorter or longer than the C-2 limit.
check_imu_fault() {
  local kind="$1" duration="$2"
  new_run "C2-imu-fault-$kind"
  start_check imu-hold --record "$IMU_RECORD_S"
  local pid="$check_pid"
  launch teleop:=false initial_pitch_deg:=2.0 \
    imu_fault_start_s:="$IMU_FAULT_START_S" imu_fault_duration_s:="$duration"
  zero_reference
  local failures=() log="$dir/launch.log"
  wait "$pid"; local status=$?
  grep -q "injected IMU read failure" "$log" \
    || failures+=("SimPlant never logged an injected IMU read failure: the outage did not happen")
  # The probe must have been listening when the outage began (and, for the short one, for a second
  # after it ended), or "never 0.0" would be about samples from before it. Outage times count from
  # SimPlant's release.
  local release last need
  release="$(log_stamp "$log" "releasing the simulated robot")"
  last="$(metric "$dir/metrics.json" last_stamp_s)"
  need="$IMU_FAULT_START_S"
  [ "$kind" = short ] && need="$(awk -v a="$IMU_FAULT_START_S" -v b="$duration" 'BEGIN {print a + b + 1}')"
  if [ -z "$release" ] || [ -z "$last" ] || ! seconds_between "$release" "$last" "$need"; then
    failures+=("the probe's last sample (${last:-none}) is not $need s after the release (${release:-none})")
  fi
  local deactivated="Deactivating following hardware components as their read cycle resulted in an error"
  if [ "$kind" = short ]; then
    grep -q "$deactivated" "$log" && failures+=("controller_manager deactivated the hardware: read() returned ERROR")
    wiring_problems
    [ ${#problems[@]} -ne 0 ] && failures+=("after the outage: ${problems[*]}")
  else
    grep -q "$deactivated.*SimRocko" "$log" \
      || failures+=("controller_manager did not log '$deactivated ... SimRocko': no ERROR from read()")
    # What controller_manager did about it, for the record (not judged).
    echo "    after the ERROR, controller_manager logged:"
    grep -E "Deactivating following|mode switch|Not acceptable" "$log" | sed 's/^.*\]: /      /'
    echo "    controller states:"
    timeout -k 2 "$CALL_TIMEOUT" ros2 control list_controllers 2>/dev/null | sed 's/^/      /'
    echo "    hardware component state:"
    timeout -k 2 "$CALL_TIMEOUT" ros2 control list_hardware_components 2>/dev/null \
      | grep -E "name:|state:" | sed 's/^[[:space:]]*/      /'
  fi
  judge "$status" "${failures[@]}"
}

# H6: the pitch PID's D term reads the exported pitch_rate.
check_pitch_rate() {
  new_run "H6-pitch-rate"
  start_check pitch-rate --record "$H6_RECORD_S" --min-rate-deg-s "$H6_MIN_RATE_DEG_S" \
    --tolerance-deg-s "$H6_TOLERANCE_DEG_S"
  local pid="$check_pid"
  launch teleop:=false initial_pitch_deg:=2.0
  zero_reference
  wait_for_wiring
  local failures=()
  [ ${#problems[@]} -ne 0 ] && failures+=("${problems[*]}")
  wait "$pid"; local status=$?
  judge "$status" "${failures[@]}"
}

# FF: the feedforward age limit (finding H-11). A publisher feeds both wheels, then dies.
check_feedforward_age() {
  new_run "FF-feedforward-age"
  launch teleop:=false
  zero_reference
  wait_for_wiring
  local failures=() side log="$dir/launch.log" ff_pids=() pid
  [ ${#problems[@]} -ne 0 ] && failures+=("before the feedforward: ${problems[*]}")
  # Stamps on the same system clock as the ROS log stamps.
  local started killed
  started="$(date +%s.%N)"
  for side in left right; do
    # Unbuffered, so its 'publishing #' lines reach the log before the SIGKILL below.
    start_group "$dir/ff_$side.log" env PYTHONUNBUFFERED=1 ros2 topic pub -r "$FF_RATE_HZ" \
      "/${side}_feedforward" std_msgs/msg/Float64 "{data: $FEEDFORWARD_RAD_S}"
    ff_pids+=("$!")
  done
  sleep "$FF_PUBLISH_S"
  # The kill is the stand-in for DiffDriveController.py dying: SIGKILL, so no message goes out after
  # it. The time is taken just before.
  killed="$(date +%s.%N)"
  for pid in "${ff_pids[@]}"; do kill -KILL -- "-$pid" 2>/dev/null; wait "$pid" 2>/dev/null; done
  local status=0 deadline
  for side in left right; do
    local pattern="\[${side}_add_feedforward_controller\]: Feedforward on .* ms old"
    local stamps=() after=() during=0 before=0 stamp last_during=""
    # Wait (for log flushing) until a time-out line stamped after the kill appears, or give up.
    deadline=$((SECONDS + REARM_REPLY_WAIT_S))
    while :; do
      mapfile -t stamps < <(grep -E "$pattern" "$log" \
        | sed -n 's/.*\[\(INFO\|WARN\|ERROR\)\] \[\([0-9]*\.[0-9]*\)\].*/\2/p')
      after=()
      for stamp in "${stamps[@]}"; do seconds_between "$killed" "$stamp" 0 && after+=("$stamp"); done
      [ ${#after[@]} -gt 0 ] || [ $SECONDS -ge $deadline ] && break
      sleep 0.5
    done
    for stamp in "${stamps[@]}"; do
      if ! seconds_between "$started" "$stamp" 0; then
        before=$((before + 1))
      elif ! seconds_between "$killed" "$stamp" 0; then
        during=$((during + 1))
        last_during="$stamp"
      fi
    done
    # Before any feedforward was sent there is nothing to age out (the "never arrived" rule).
    [ "$before" -eq 0 ] \
      || failures+=("${side}_add_feedforward_controller logged a time-out $before time(s) before any feedforward was sent")
    # While the publisher ran, a time-out means a real delivery gap of more than FF_AGE_S (the
    # publisher is a Python process sharing the container's CPUs; seen once at 20 Hz and once at
    # 100 Hz, always on one wheel only, so not a stall of the controller's executor, which serves
    # both). Reported, not judged: it is what the limit is for.
    [ "$during" -gt 0 ] \
      && echo "    note: ${side}: $during time-out(s) while the publisher was running (a delivery gap > $FF_AGE_S s)"
    if [ ${#after[@]} -ge 1 ]; then
      stamp="${after[0]}"
      echo "    ${side}: time-out logged $(awk -v a="$killed" -v b="$stamp" 'BEGIN {printf "%.3f", b - a}') s after the kill (${#after[@]} line(s) after it)"
      if ! seconds_between "$killed" "$stamp" "$FF_EARLIEST_S"; then
        failures+=("${side}_add_feedforward_controller logged the time-out less than $FF_EARLIEST_S s after the kill (killed $killed, logged $stamp): the limit is shorter than $FF_AGE_S s")
      elif seconds_between "$killed" "$stamp" "$(awk -v a="$FF_AGE_S" -v b="$FF_MARGIN_S" 'BEGIN {print a + b}')"; then
        failures+=("${side}_add_feedforward_controller logged the time-out more than $FF_AGE_S + $FF_MARGIN_S s after the kill (killed $killed, logged $stamp)")
      fi
      [ ${#after[@]} -eq 1 ] \
        || failures+=("${side}_add_feedforward_controller logged the time-out ${#after[@]} times after the kill, expected once")
    elif [ -n "$last_during" ] && ! seconds_between "$last_during" "$killed" "$FF_LOG_GAP_S"; then
      # The controller logs at most one time-out line per FF_LOG_GAP_S, so the kill's time-out may
      # have been counted but not logged. Not the controller's fault, and not a pass either.
      echo "    ${side}: a time-out was logged less than $FF_LOG_GAP_S s before the kill, so the kill's line may be suppressed: cannot judge"
      status=2
    elif grep -q "publishing #" "$dir/ff_$side.log"; then
      failures+=("${side}_add_feedforward_controller never logged 'Feedforward on ... ms old' after its publisher was killed")
    else
      # Nothing was ever sent, so there was nothing to age out: not the controller's fault.
      echo "    ${side}: the publisher never logged 'publishing #' (see $dir/ff_$side.log)"
      status=2
    fi
  done
  # Only the feedforward term goes to 0.0: nothing may be deactivated.
  wiring_problems
  [ ${#problems[@]} -ne 0 ] && failures+=("after the time-out: ${problems[*]}")
  note_tipover
  judge "$status" "${failures[@]}"
}

# ---- The balance scenarios (mode `scenarios`) ----------------------------------------------------
scenario_failed=0
scenario_could_not=0
declare -A scenario_line  # name -> one-line result, printed again at the end
# scenario_report <name> <ok|FAIL|COULD NOT JUDGE> <text>: print and remember one scenario result.
scenario_report() {
  local name="$1" verdict="$2" text="$3" suffix=""
  case "$verdict" in
    FAIL) scenario_failed=1
          if [ "$SCENARIOS_EXPECTED_RED" -eq 1 ] && [[ " U I V " == *" $name "* ]]; then
            suffix=" (expected until the sim gains are tuned)"
          fi ;;
    "COULD NOT JUDGE") scenario_could_not=1 ;;
  esac
  scenario_line[$name]="SCENARIO $name: $verdict$suffix: $text"
  echo "  ${scenario_line[$name]}"
}
# scenario_reason: the check's one-line reason from metrics.json.
scenario_reason() { metric "$dir/metrics.json" reason; }

# scenario_launch_problems: what is wrong with the run itself (not the robot's behaviour): the wiring
# before the check ended, and the release. Fills `failures`.
scenario_launch_problems() {
  [ ${#problems[@]} -ne 0 ] && failures+=("the wiring was not right: ${problems[*]}")
  grep -q "releasing the simulated robot" "$dir/launch.log" \
    || failures+=("SimPlant never released the robot")
}

# run_upright <label> <record s> <launch args...>: one launch with the U check. Sets $check_status.
run_upright() {
  new_run "$1"; local record="$2"; shift 2
  start_check upright --launch-log "$dir/launch.log" --record "$record" \
    --tipover-deg "$TIPOVER_SCENARIO_DEG" --settle-from "$U_SETTLE_FROM_S" \
    --settled-pitch-deg "$U_SETTLED_DEG" --max-speed-m-s "$U_MAX_SPEED_M_S"
  local pid="$check_pid"
  launch teleop:=false initial_pitch_deg:="$U_START_DEG" "$@"
  zero_reference
  wait_for_wiring
  wait "$pid"; check_status=$?
  sed 's/^/    /' "$dir/check.log"
  note_tipover
}

scenario_upright() {
  run_upright "U-upright" "$U_RECORD_S" encoder_sign:="$SCENARIO_ENCODER_SIGN"
  local failures=(); scenario_launch_problems
  if [ ${#failures[@]} -ne 0 ]; then scenario_report U "COULD NOT JUDGE" "${failures[*]}"
  else scenario_report U "$(verdict_of "$check_status")" "$(scenario_reason)"; fi
  end_run
}

scenario_impulse() {
  new_run "I-impulse"
  start_check impulse --launch-log "$dir/launch.log" --record "$I_RECORD_S" \
    --tipover-deg "$TIPOVER_SCENARIO_DEG" --impulse-time-s "$I_IMPULSE_AT_S" \
    --impulse-deg-s "$I_IMPULSE_DEG_S" --settle-within-s "$I_SETTLE_S" \
    --settled-pitch-deg "$I_SETTLED_DEG"
  local pid="$check_pid"
  launch teleop:=false encoder_sign:="$SCENARIO_ENCODER_SIGN" \
    impulse_time_s:="$I_IMPULSE_AT_S" impulse_angular_velocity_deg_per_s:="$I_IMPULSE_DEG_S"
  zero_reference
  wait_for_wiring
  wait "$pid"; local status=$?
  sed 's/^/    /' "$dir/check.log"
  note_tipover
  local failures=(); scenario_launch_problems
  if [ ${#failures[@]} -ne 0 ]; then scenario_report I "COULD NOT JUDGE" "${failures[*]}"
  else scenario_report I "$(verdict_of "$status")" "$(scenario_reason)"; fi
  end_run
}

scenario_velocity_step() {
  new_run "V-velocity-step"
  start_check velocity-step --launch-log "$dir/launch.log" --tipover-deg "$TIPOVER_SCENARIO_DEG" \
    --speed-m-s "$V_STEP_M_S" --before-s "$V_BEFORE_S" --step-s "$V_STEP_S" \
    --rise-fraction "$V_RISE_FRACTION" --rise-within-s "$V_RISE_S" \
    --max-overshoot-pct "$V_OVERSHOOT_PCT" --max-pitch-deg "$V_PITCH_DEG"
  local pid="$check_pid"
  # teleop:=true: the real DiffDriveController.py turns the fake gamepad into the velocity reference
  # and the feedforwards, as on the robot (no zero_reference here).
  launch teleop:=true encoder_sign:="$SCENARIO_ENCODER_SIGN"
  wait_for_wiring
  wait "$pid"; local status=$?
  sed 's/^/    /' "$dir/check.log"
  note_tipover
  local failures=(); scenario_launch_problems
  if [ ${#failures[@]} -ne 0 ]; then scenario_report V "COULD NOT JUDGE" "${failures[*]}"
  else scenario_report V "$(verdict_of "$status")" "$(scenario_reason)"; fi
  end_run
}

# S2b: U with the pitch loop's sign flipped (pitch_sign:=+1 flips the sign of the pitch the
# controllers read, which is the same as negating both pitch gains, without a second gain file).
scenario_s2b() {
  run_upright "S2b-sign-flipped" "$U_RECORD_S" encoder_sign:="$SCENARIO_ENCODER_SIGN" pitch_sign:=1
  local failures=(); scenario_launch_problems
  local fell; fell="$(metric "$dir/metrics.json" fell_at_s)"
  if [ ${#failures[@]} -ne 0 ]; then
    scenario_report S2b "COULD NOT JUDGE" "${failures[*]}"
  elif [ "$check_status" -eq 0 ]; then
    scenario_report S2b FAIL "U PASSED with the pitch loop's sign flipped: the scenario cannot see a sign error"
  elif [ "$check_status" -ne 1 ]; then
    scenario_report S2b "COULD NOT JUDGE" "U could not judge: $(scenario_reason)"
  elif [ -z "$fell" ] || [ "$fell" = None ] || ! awk -v f="$fell" -v m="$S2B_FALL_S" 'BEGIN {exit !(f <= m)}'; then
    scenario_report S2b FAIL "U failed, but not by a fast fall within $S2B_FALL_S s (fell_at_s: ${fell:-none}): $(scenario_reason)"
  else
    scenario_report S2b ok "U failed for the right reason: the pitch diverged, |pitch| reached $TIPOVER_SCENARIO_DEG deg $fell s after release (limit $S2B_FALL_S s)"
  fi
  end_run
}

# AB: the as-built signs (encoder_sign:=+1, the sim's default), expected to fall slowly.
scenario_as_built() {
  run_upright "AB-as-built-signs" "$AB_RECORD_S" encoder_sign:=1
  local failures=(); scenario_launch_problems
  local fell; fell="$(metric "$dir/metrics.json" fell_at_s)"
  if [ ${#failures[@]} -ne 0 ]; then
    scenario_report AB "COULD NOT JUDGE" "${failures[*]}"
  elif [ -z "$fell" ] || [ "$fell" = None ]; then
    if [ "$check_status" -eq 2 ]; then
      scenario_report AB "COULD NOT JUDGE" "$(scenario_reason)"
    else
      scenario_report AB FAIL "did NOT fall in $AB_RECORD_S s, but finding H-12 predicts it must: the signs or gains changed; revisit H-12 and this expectation"
    fi
  elif ! awk -v f="$fell" -v m="$AB_MIN_FALL_S" 'BEGIN {exit !(f >= m)}'; then
    scenario_report AB FAIL "fell $fell s after release, sooner than $AB_MIN_FALL_S s: not the slow run-away of H-12"
  else
    scenario_report AB ok "fell as expected (finding H-12): |pitch| reached $TIPOVER_SCENARIO_DEG deg $fell s after release"
  fi
  end_run
}

# verdict_of <check exit status>: the word scenario_report takes.
verdict_of() { case "$1" in 0) echo PASS ;; 1) echo FAIL ;; *) echo "COULD NOT JUDGE" ;; esac; }

if [ "$mode" = scenarios ]; then
  echo "SCENARIOS ($selected). ASSUMPTION for U, I, V, S2b: encoder_sign:=$SCENARIO_ENCODER_SIGN (not checked on"
  echo "the robot; proposal 0006). Gains: rocko_sim_controllers.yaml as it is. A model with ESTIMATED parameters."
  wants U && scenario_upright
  wants I && scenario_impulse
  wants V && scenario_velocity_step
  wants S2b && scenario_s2b
  wants AB && scenario_as_built
  echo "--- scenario results (encoder_sign:=$SCENARIO_ENCODER_SIGN assumed for U, I, V, S2b; AB uses +1):"
  for s in $selected; do echo "  ${scenario_line[$s]}"; done
  echo "  A pass would mean: the model, with its estimated parameters and the assumed signs, is held up by"
  echo "  these gains. It would say nothing about the real robot."
  if [ "$scenario_failed" -ne 0 ]; then echo "SCENARIOS: FAIL (exit 1)"; exit 1; fi
  if [ "$scenario_could_not" -ne 0 ]; then echo "SCENARIOS: COULD NOT JUDGE (exit 2)"; exit 2; fi
  passed=1
  echo "SCENARIOS: PASS (exit 0)"
  exit 0
fi

wants S1 && run "S1-baseline" - still "$S1_STILL_DEG"
wants S1 && run "S1-command" "$FEEDFORWARD_RAD_S" change "$S1_CHANGE_DEG"
wants S2a && run "S2a-no-torque" - exceed "$TIPOVER_DEG" disable_torque:=true initial_pitch_deg:=2.0
wants H4 && check_tipover
wants C3 && check_teleop
wants C2 && check_imu_fault short "$IMU_FAULT_SHORT_S"
wants C2 && check_imu_fault long "$IMU_FAULT_LONG_S"
wants H6 && check_pitch_rate
wants FF && check_feedforward_age

if [ "$failed" -ne 0 ]; then
  echo "FAIL (simulation: $selected): see above."
  exit 1
fi
if [ "$could_not" -ne 0 ]; then
  echo "COULD NOT JUDGE (simulation: $selected): see above."
  exit 2
fi
passed=1
echo "PASS (simulation: $selected). Each check that ran held, as printed above. In brief:"
wants S1 && echo "  S1: the cascade and joint_state_broadcaster are active against SimPlant with all 14 interfaces;
    pitch stayed within $S1_STILL_DEG deg with no wheel command and moved >= $S1_CHANGE_DEG deg with $FEEDFORWARD_RAD_S rad/s."
wants S2a && echo "  S2a: with no motor torque the robot passed $TIPOVER_DEG deg within $FALL_LIMIT_S s."
wants H4 && echo "  H4: past $TIPOVER_DEG deg both controllers tripped in the first cycle and wrote exactly 0.0, one
    /tipover_rearm message past the limit was refused by both, and the controllers stayed active."
wants C3 && echo "  C3: when the fake gamepad went silent the velocity reference went to exactly [0, 0] within
    $TELEOP_TIMEOUT_S + $TELEOP_MARGIN_S s, and the balance loop kept running."
wants C2 && echo "  C2: a $IMU_FAULT_SHORT_S s IMU outage gave no ERROR; a $IMU_FAULT_LONG_S s one made read() return ERROR and
    controller_manager deactivate the hardware; the exported pitch was never 0.0 or NaN."
wants H6 && echo "  H6: pitch_pid_controller's D term reads the exported pitch_rate (gyro), not a differentiated angle."
wants FF && echo "  FF: when the feedforward publisher died, both controllers timed it out once, $FF_EARLIEST_S to
    $FF_AGE_S + $FF_MARGIN_S s later, and stayed active."
echo "  This is a MODEL WITH ESTIMATED, UNMEASURED PARAMETERS. It does not show that the real robot"
echo "  balances, or that any gain is good."
exit 0
