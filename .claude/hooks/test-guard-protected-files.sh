#!/usr/bin/env bash
# Tests for the guard hook (guard-protected-files.sh -> guard_protected_files.py).
# Needs only bash and python3 (no ROS, no robot).
#
#   .claude/hooks/test-guard-protected-files.sh
#
# Each case feeds the hook a fake tool call and checks BOTH the decision and the exit status.
# The exit status matters: the harness treats a hook that exits non-zero as "no opinion" and lets
# the edit through, so a crashing hook would otherwise score as a harmless "allow".
#
# The hook judges whole files, so cases run against a throwaway copy of the files it guards (the
# real gains, CMake and Dockerfile) plus small fixtures. Nothing in the repo is touched. Prints
# one line per case and exits non-zero if any case fails. Run it after any change to the hook.
set -uo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd "$here/../.." && pwd)"
hook="$here/guard-protected-files.sh"

fixture="$(mktemp -d)"
trap 'chmod -R u+rwx "$fixture" 2>/dev/null; rm -rf "$fixture"' EXIT
export CLAUDE_PROJECT_DIR="$fixture"

cfg=rocko_env/rocko_env/bringup/config
yaml=$cfg/rocko_controllers.yaml
tank=$cfg/rocko_tank_mode_controllers.yaml
cmake=rocko_env/CMakeLists.txt
docker=docker/Dockerfile
motor=rocko_env/rocko_env/hardware/motors/Motor.cpp
worktree_motor=.claude/worktrees/wt1/$motor
for f in $yaml $tank $cmake $docker; do
  mkdir -p "$fixture/$(dirname "$f")" && cp "$repo/$f" "$fixture/$f"
done
mkdir -p "$fixture/$(dirname $motor)" "$fixture/$(dirname $worktree_motor)" "$fixture/.claude" "$fixture/docs"
cat > "$fixture/$motor" <<'EOF'
// Fake driver: what matters is the safety words, plus a harmless "default".
Wheel() = default;
if (!imu_ok) { return hardware_interface::return_type::ERROR; }
pwm = std::clamp(pwm, 0, 100);   // clamp before writing the pin
v = rate_limiter.limit(v);
bool imuFault = false;   // set when the IMU read fails
EOF
cp "$fixture/$motor" "$fixture/$worktree_motor"
printf '{}\n' > "$fixture/.claude/settings.json"
ln -s "$fixture/.claude/settings.json" "$fixture/docs/innocent.json"   # a symlink INTO the guardrails
printf '# 20\xb0C\ngains: {p: 1.0}\n' > "$fixture/$cfg/rocko_bad_bytes.yaml"   # a latin-1 degree sign
printf 'a:\r\n  update_rate: 100\r\n  b: 1\r\n' > "$fixture/$cfg/rocko_crlf.yaml"   # Windows line endings
printf 'gains: {p: 1.0}\n' > "$fixture/$cfg/rocko_unreadable.yaml"; chmod 000 "$fixture/$cfg/rocko_unreadable.yaml"
printf 'x:\n  gains: {p: 1.0}\n' > "$fixture/$cfg/rocko_controllers_sim.yaml"
printf 'x:\n  gains: {p: 1.0}\n' > "$fixture/$cfg/simple_gains.yaml"
mkdir -p "$fixture/$cfg/sub" && printf 'x:\n  gains: {p: 1.0}\n' > "$fixture/$cfg/sub/deeper.yaml"
mkdir -p "$fixture/rocko_sim2" && printf '# note: TODO\nfind_package(controller_manager 4.0.0)\n' > "$fixture/rocko_sim2/CMakeLists.txt"
printf 'x:\n  other: {k: x#y}\n' > "$fixture/$cfg/rocko_hash.yaml"   # a '#' with no space before it is NOT a YAML comment

# Is this filesystem case-insensitive (macOS default, Windows)? On Linux and WSL2 a differently
# cased path is simply a different file, so the mixed-case gain case below cannot apply.
touch "$fixture/CaseProbe"; if [ -e "$fixture/caseprobe" ]; then case_insensitive=1; else case_insensitive=0; fi

fail=0
cases=0

# check <allow|deny|ask> <description> <json payload>
# "allow" means the hook printed nothing: it has no opinion, and normal permission rules apply.
check() {
  local want="$1" desc="$2" payload="$3" out rc got
  cases=$((cases + 1))
  # An empty payload means a helper below refused to build it (its message is on stderr above).
  # Without this, the hook would answer "ask" to the empty input and an `ask` case would pass.
  if [ -z "$payload" ]; then echo "FAIL  $desc (empty payload: test setup error above)"; fail=1; return; fi
  out="$(printf '%s' "$payload" | "$hook")"; rc=$?
  if [ "$rc" -ne 0 ]; then
    echo "FAIL  $desc (hook exited $rc: a non-zero exit lets the edit through)"; fail=1; return
  fi
  if [ -z "$out" ]; then
    got=allow
  else
    got="$(printf '%s' "$out" | python3 -c \
      'import json,sys; print(json.load(sys.stdin)["hookSpecificOutput"]["permissionDecision"])')"
  fi
  if [ "$got" = "$want" ]; then echo "ok    $desc"; else echo "FAIL  $desc (wanted $want, got $got)"; fail=1; fi
}

# edit <path> <old> <new>   -> an Edit tool call. A path starting with / is used as given; any
# other is relative to the fixture. If the file exists but does not contain old_string, the case
# would silently test nothing (for example after the gains are re-tuned), so refuse to build the
# payload and say why.
edit() {
  python3 - "$fixture" "$1" "$2" "$3" <<'PY'
import json, os, sys
fixture, path, old, new = sys.argv[1:]
if not path.startswith("/"):
    path = os.path.join(fixture, path)
if os.path.isfile(path) and old:
    with open(path, newline="", errors="surrogateescape") as f:
        text = f.read().replace("\r\n", "\n")
    if old not in text:
        sys.exit("TEST SETUP ERROR: %r is not in %s (changed? update the test)" % (old, path))
print(json.dumps({"tool_name": "Edit", "tool_input": {"file_path": path, "old_string": old, "new_string": new}}))
PY
}

# raw_edit <path> <old> <new>   -> an Edit payload built WITHOUT the check above (for edits whose
# old_string is deliberately not in the file)
raw_edit() {
  python3 - "$fixture" "$1" "$2" "$3" <<'PY'
import json, os, sys
fixture, path, old, new = sys.argv[1:]
print(json.dumps({"tool_name": "Edit", "tool_input": {"file_path": os.path.join(fixture, path), "old_string": old, "new_string": new}}))
PY
}

# write_with <path> <old> <new>   -> a Write of the file's current text with old->new (first match)
write_with() {
  python3 - "$fixture/$1" "$2" "$3" <<'PY'
import json, sys
path, old, new = sys.argv[1:]
with open(path, newline="") as f:
    text = f.read().replace(old, new, 1)
print(json.dumps({"tool_name": "Write", "tool_input": {"file_path": path, "content": text}}))
PY
}
unchanged() { write_with "$1" "" ""; }   # a Write whose content is the file as it is

# write_new <path> <content>   -> a Write that creates a file
write_new() {
  python3 - "$fixture/$1" "$2" <<'PY'
import json, sys
print(json.dumps({"tool_name": "Write", "tool_input": {"file_path": sys.argv[1], "content": sys.argv[2]}}))
PY
}

# swap_gains <path>   -> a Write that exchanges the pitch-loop and velocity-loop gain sets between
# their controllers. The same values still exist in the file; they now belong to other controllers.
swap_gains() {
  python3 - "$fixture/$1" <<'PY'
import json, sys
path = sys.argv[1]
text = open(path).read()
a = "{p: 10.0, i: 0.0, d: 0.1, i_clamp_max: 150.0, i_clamp_min: -150.0}"   # pitch loop, left wheel
b = "{p: 2.0, i: 5.0, d: 0.2, i_clamp_max: 150.0, i_clamp_min: -150.0}"     # velocity loop, left wheel
if a not in text or b not in text:
    sys.exit("TEST SETUP ERROR: the gain sets this test swaps are not in " + path + " (re-tuned? update swap_gains)")
text = text.replace(a, "@@", 1).replace(b, a, 1).replace("@@", b, 1)   # first occurrence only: a appears twice
print(json.dumps({"tool_name": "Write", "tool_input": {"file_path": path, "content": text}}))
PY
}

# crlf_edit   -> an Edit of the CRLF fixture whose strings contain real CR characters
crlf_edit() {
  python3 - "$fixture/$cfg/rocko_crlf.yaml" <<'PY'
import json, sys
print(json.dumps({"tool_name": "Edit", "tool_input": {"file_path": sys.argv[1],
      "old_string": "update_rate: 100\r\n  b: 1", "new_string": "update_rate: 1000\r\n  b: 1"}}))
PY
}

# huge_write_to_settings   -> a 1.3 MB Write to settings.json (an argv-sized payload would break a
# hook that took the payload on its command line)
huge_write_to_settings() {
  python3 - "$fixture/.claude/settings.json" <<'PY'
import json, sys
print(json.dumps({"tool_name": "Write", "tool_input": {"file_path": sys.argv[1],
      "content": '{"pad": "' + "x" * 1300000 + '"}'}}))
PY
}

# check_in_time <allow|deny|ask> <description>   -> the hook given a Write of 100,000 unclosed
# '#[[' comment openers to CMakeLists.txt, with a 20 s limit. A hook that is killed for being slow
# lets the edit through, so slowness is a failure, not just an inconvenience.
check_in_time() {
  local want="$1" desc="$2" got
  cases=$((cases + 1))
  got="$(python3 - "$hook" "$fixture/$cmake" <<'PY'
import json, subprocess, sys
hook, path = sys.argv[1], sys.argv[2]
payload = json.dumps({"tool_name": "Write", "tool_input": {"file_path": path, "content": "#[[\n" * 100000}})
try:
    r = subprocess.run([hook], input=payload, capture_output=True, text=True, timeout=20)
    print(json.loads(r.stdout)["hookSpecificOutput"]["permissionDecision"] if r.stdout.strip() else "allow", end="")
except subprocess.TimeoutExpired:
    print("TIMEOUT (over 20 s)", end="")
PY
)"
  if [ "$got" = "$want" ]; then echo "ok    $desc"; else echo "FAIL  $desc (wanted $want, got $got)"; fail=1; fi
}

echo "--- hardware gains and update_rate"
check deny  "change a P gain"                  "$(edit $yaml 'p: 10.0, i: 0.0' 'p: 12.0, i: 0.0')"
check deny  "change only the number"           "$(edit $yaml '10.0, i: 0.0' '12.0, i: 0.0')"
check deny  "change update_rate"               "$(edit $yaml 'update_rate: 100' 'update_rate: 200')"
check deny  "change a command limit"           "$(edit $tank 'max_velocity: 20.0' 'max_velocity: 200.0')"
check deny  "switch a limit off"               "$(edit $tank 'linear.x.has_velocity_limits: true' 'linear.x.has_velocity_limits: false')"
check deny  "change the cmd_vel watchdog"      "$(edit $tank 'cmd_vel_timeout: 1.0' 'cmd_vel_timeout: 1000.0')"
check deny  "Write that changes a gain"        "$(write_with $yaml 'p: 10.0' 'p: 11.0')"
check deny  "swap gains between controllers"   "$(swap_gains $yaml)"
check deny  "attach a gain set to another joint" "$(edit $yaml 'left_add_feedforward_controller/left_wheel_joint:   {p' 'left_add_feedforward_controller/right_wheel_joint:   {p')"
check deny  "hide a second p after a '#'"      "$(edit $yaml 'i_clamp_min: -150.0} # 10, 1.0' 'i_clamp_min: -150.0, k: x#y, p: 99.0} # 10, 1.0')"
check deny  "add a p after a '#y' that is not a comment" "$(edit $cfg/rocko_hash.yaml 'k: x#y}' 'k: x#y, p: 99.0}')"
check deny  "'p:10.0' is a different key from 'p: 10.0'" "$(edit $yaml '{p: 10.0, i: 0.0, d: 0.1, i_clamp_max: 150.0, i_clamp_min: -150.0} # 10' '{p:10.0, i: 0.0, d: 0.1, i_clamp_max: 150.0, i_clamp_min: -150.0} # 10')"
check deny  "capitalise a gain key (P is not p)" "$(edit $yaml '{p: 10.0, i: 0.0' '{P : 10.0, i: 0.0')"
check deny  "capitalise update_rate"           "$(edit $yaml 'update_rate: 100' 'Update_Rate: 100')"
check deny  "a gain after a lone CR (ends a YAML comment)" "$(edit $yaml 'i_clamp_min: -150.0} # 10, 1.0' "i_clamp_min: -150.0} # 10, 1.0"$'\r'"      left_add_feedforward_controller/left_wheel_joint: {p: 99.0}")"
check deny  "delete text so the Edit tool eats a newline" "$(edit $yaml 'right_wheel_joint:   {p: 0.0, i: 0.0, d: 0.0, i_clamp_max: 100.0, i_clamp_min: -100.0}' '')"
check deny  "de-indent a gain line (leaves its map)" "$(edit $yaml '      pitch_pid_controller/left_wheel_joint:   {p: 2.0' '    pitch_pid_controller/left_wheel_joint:   {p: 2.0')"
check deny  "gain in a subfolder of config/"   "$(edit $cfg/sub/deeper.yaml 'p: 1.0' 'p: 2.0')"
check deny  "gain edit in a CRLF file"         "$(crlf_edit)"
check deny  "gain edit, LF old_string, CRLF file" "$(edit $cfg/rocko_crlf.yaml 'update_rate: 100
  b: 1' 'update_rate: 1000
  b: 1')"
check deny  "gain edit in a non-UTF-8 file"    "$(edit $cfg/rocko_bad_bytes.yaml 'p: 1.0' 'p: 2.0')"
check deny  "new gains via an empty old_string" "$(raw_edit $cfg/new_gains.yaml '' 'gains: {p: 99.0}')"
check deny  "Write a new non-sim file with gains" "$(write_new $cfg/new_gains.yaml 'gains: {p: 1.0}')"
if [ "$case_insensitive" = 1 ]; then
  check deny "mixed-case path to the gains"    "$(edit Rocko_Env/ROCKO_ENV/Bringup/Config/Rocko_Controllers.YAML 'p: 10.0' 'p: 99.0')"
else
  echo "skip  mixed-case path to the gains (this filesystem is case-sensitive: that is a different file)"
fi
check deny  "gain in simple_gains.yaml (not sim)" "$(edit $cfg/simple_gains.yaml 'p: 1.0' 'p: 2.0')"
check ask   "an edit that does not match the file" "$(raw_edit $yaml 'i_clamp_max: 150.0”' 'i_clamp_max: 999.0')"
check allow "edit only a comment"              "$(edit $yaml '# 10, 1.0, 0.1' '# tried 10, 1.0, 0.2')"
check allow "rename a topic (wiring)"          "$(edit $yaml 'left_feedforward' 'left_ff')"
check allow "reformat a gain, spacing only"    "$(edit $yaml '{p: 10.0, i: 0.0, d: 0.1, i_clamp_max: 150.0, i_clamp_min: -150.0} # 10' '{p: 10.0,   i: 0.0 , d: 0.1,i_clamp_max: 150.0, i_clamp_min: -150.0 } # 10')"
check allow "reformat update_rate"             "$(edit $yaml 'update_rate: 100' 'update_rate :   100')"
check allow "Write that changes nothing"       "$(unchanged $yaml)"
check allow "gain in a *_sim.yaml file"        "$(edit $cfg/rocko_controllers_sim.yaml 'p: 1.0' 'p: 2.0')"
check ask   "unreadable file at a gated path"  "$(write_new $cfg/rocko_unreadable.yaml 'gains: {p: 9.0}')"

echo "--- version pins"
check deny  "bump the ros2_control pin"        "$(edit $cmake 'controller_manager 4.0.0' 'controller_manager 5.0.0')"
check deny  "change only the number"           "$(edit $cmake '4.0.0)' '5.0.0)')"
check deny  "change the C++ standard"          "$(edit $cmake 'cxx_std_17' 'cxx_std_20')"
check deny  "bump the standard, keep the word elsewhere" "$(edit $cmake 'target_compile_features(rocko_env PUBLIC cxx_std_17)' 'target_compile_features(rocko_env PUBLIC cxx_std_23)
message(STATUS "cxx_std_17")')"
check deny  "comment the pin out"              "$(edit $cmake 'find_package(controller_manager 4.0.0)' '# find_package(controller_manager 4.0.0)
find_package(controller_manager)')"
check deny  "hide the pin in a #[[ ]] comment" "$(edit $cmake 'find_package(controller_manager 4.0.0)' '#[[
find_package(controller_manager 4.0.0)
#]]
find_package(controller_manager)')"
check deny  "hide the pin after a form feed"   "$(edit $cmake 'find_package(controller_manager 4.0.0)' "# x"$'\f'"find_package(controller_manager 4.0.0)")"
check deny  "neuter the version check"         "$(edit $cmake 'if(NOT controller_manager_FOUND)' 'if(FALSE)')"
check deny  "neuter it by adding AND FALSE"    "$(edit $cmake 'if(NOT controller_manager_FOUND)' 'if(NOT controller_manager_FOUND AND FALSE)')"
check deny  "lower cmake_minimum_required"     "$(edit $cmake 'cmake_minimum_required(VERSION 3.16)' 'cmake_minimum_required (VERSION 3.5)')"
check deny  "change the Docker base image"     "$(edit $docker 'FROM ros:jazzy' 'FROM ros:rolling')"
check deny  "delete text so the Edit tool eats a newline into a pin" "$(edit rocko_sim2/CMakeLists.txt 'TODO' '')"
check_in_time deny "100,000 unclosed #[[ openers are handled in time"
check allow "reformat a pin line"              "$(edit $cmake 'find_package(controller_manager 4.0.0)' 'find_package( controller_manager   4.0.0 )')"
check allow "add a trailing comment to a pin line" "$(edit $cmake 'find_package(controller_manager 4.0.0)' 'find_package(controller_manager 4.0.0)  # Jazzy ships 4.x')"
check allow "add a source file to CMake"       "$(edit $cmake 'QuadEncoder.cpp' 'QuadEncoder.cpp
  rocko_env/hardware/mock/Foo.cpp')"
check allow "add a second cxx_std_17 target"   "$(edit $cmake 'target_compile_features(rocko_env PUBLIC cxx_std_17)' 'target_compile_features(rocko_env PUBLIC cxx_std_17)
target_compile_features(rocko_sim PUBLIC cxx_std_17)')"
check allow "create a NEW CMakeLists.txt"      "$(write_new rocko_sim/CMakeLists.txt 'cmake_minimum_required(VERSION 3.16)
find_package(controller_manager 4.0.0)
target_compile_features(rocko_sim PUBLIC cxx_std_17)')"
check allow "add an apt package to Dockerfile" "$(edit $docker 'vim \' 'vim \
    gdb \')"

echo "--- safety behavior"
check deny  "remove a clamp"                   "$(edit $motor 'pwm = std::clamp(pwm, 0, 100);   // clamp before writing the pin' 'pwm = pwm;')"
check deny  "remove an error return"           "$(edit $motor 'return hardware_interface::return_type::ERROR;' 'return hardware_interface::return_type::OK;')"
check deny  "rename clamp to limit"            "$(edit $motor 'std::clamp(pwm, 0, 100);   // clamp before writing the pin' 'std::min(pwm, 100);   // limit')"
check deny  "remove a rate limiter"            "$(edit $motor 'v = rate_limiter.limit(v);' 'v = v;')"
check deny  "remove a clamp inside a worktree" "$(edit $worktree_motor 'pwm = std::clamp(pwm, 0, 100);   // clamp before writing the pin' 'pwm = pwm;')"
check deny  "remove a camelCase fault flag"    "$(edit $motor 'bool imuFault = false;   // set when the IMU read fails' '')"
check allow "remove a harmless 'default'"      "$(edit $motor 'Wheel() = default;' '')"
check allow "keep the clamp while editing"     "$(edit $motor 'std::clamp(pwm, 0, 100)' 'std::clamp(pwm, 0, 99)')"
check allow "add a clamp"                      "$(edit $motor 'pwm = std::clamp' 'pwm = std::clamp(pwm, 0, 1) + std::clamp')"

echo "--- the guardrails themselves"
check ask   "edit settings.json"               "$(edit .claude/settings.json '{}' '{"a":1}')"
check ask   "edit the hook"                    "$(edit .claude/hooks/guard_protected_files.py 'deny' 'allow')"
check ask   "mixed-case path to settings"      "$(edit .Claude/SETTINGS.json '{}' '{"a":1}')"
check ask   "symlink into settings.json"       "$(edit docs/innocent.json '{}' '{"a":1}')"
check ask   "another checkout's hook"          "$(edit /tmp/some-other-checkout/.claude/hooks/x.sh 'a' 'b')"
check ask   "git config (core.fsmonitor runs code)" "$(edit .git/config 'a' 'b')"
check ask   "git hooks"                        "$(edit .git/hooks/pre-commit 'a' 'b')"
check ask   "a submodule's git hooks"          "$(edit .git/modules/x/hooks/pre-commit 'a' 'b')"
check ask   ".mcp.json (adds tools)"           "$(edit .mcp.json 'a' 'b')"
check ask   "a custom slash command"           "$(edit .claude/commands/x.md 'a' 'b')"
check ask   "a huge Write to settings.json"    "$(huge_write_to_settings)"
check allow "Claude memory files are not guarded" "$(edit /Users/someone/.claude/projects/x/memory/m.md 'a' 'b')"

echo "--- the hook fails closed"
check ask   "malformed JSON"                   "not json"
check ask   "a payload that is not an object"  "[1, 2, 3]"
check allow "no file_path"                     '{"tool_name":"Edit","tool_input":{}}'
check allow "null old_string / new_string"     "{\"tool_name\":\"Edit\",\"tool_input\":{\"file_path\":\"$fixture/$yaml\",\"old_string\":null,\"new_string\":null}}"
check allow "edits that is not a list"         "{\"tool_name\":\"MultiEdit\",\"tool_input\":{\"file_path\":\"$fixture/$yaml\",\"edits\":\"oops\"}}"

echo "--- everything else is none of the hook's business"
mkdir -p "$fixture/rocko_env/rocko_env/controllers" && printf 'int foo = 1;\n' > "$fixture/rocko_env/rocko_env/controllers/Foo.cpp"
check allow "ordinary C++ edit"                "$(edit rocko_env/rocko_env/controllers/Foo.cpp 'foo' 'bar')"
check allow "a file outside the project"       "$(edit /tmp/x.yaml 'p: 1.0' 'p: 2.0')"

echo "--- settings.json actually wires the hook up"
cases=$((cases + 1))
if python3 - "$repo/.claude/settings.json" "$here" <<'PY'
import json, os, sys
settings, hooks_dir = sys.argv[1], sys.argv[2]
cfg = json.load(open(settings))
entries = cfg["hooks"]["PreToolUse"]
matcher = "|".join(e["matcher"] for e in entries)
missing = [t for t in ("Edit", "Write", "MultiEdit") if t not in matcher.split("|")]
cmds = [h["command"] for e in entries for h in e["hooks"]]
target = os.path.join(hooks_dir, "guard-protected-files.sh")
ok_cmd = any("guard-protected-files.sh" in c for c in cmds) and os.access(target, os.X_OK)
if missing or not ok_cmd:
    print(f"matcher missing {missing}; command points at an executable hook: {ok_cmd}")
    sys.exit(1)
PY
then echo "ok    matcher covers Edit, Write, MultiEdit; the hook file exists and is executable"
else echo "FAIL  settings.json does not wire the hook up correctly"; fail=1; fi

echo "--- $cases cases ($((cases - 1)) hook cases plus the wiring check); exit status asserted on every hook case"
exit "$fail"
