#!/usr/bin/env bash
# Host test of the Pi helper scripts (pi/rocko_env.sh, rocko_run.sh, rocko_shell.sh, launch.sh and its
# tank-service guard, install_boot_service.sh's rendered units).
# No ROS, no robot, no real sudo: a fake ROS setup file, a fake `ros2` that prints what it was given
# and what it can see, and a fake `sudo` that logs the call and runs the rest as the same user.
#
#   bash tools/pi_scripts_test.sh        # from the repo root; ran on macOS bash 3.2 and the container's bash 5
#
# WHAT THIS PROVES: the scripts build the environment they say they build (ROS, venv, workspace
# overlay, ROS_DOMAIN_ID, cd), go through sudo, hand sudo's fresh environment the settings that depend
# on the caller's home, pass the command and its exit status through, and launch.sh maps tank/balance to
# the right launch file and keeps recordings in the caller's ~/rocko_logs.
# WHAT IT CANNOT PROVE: that real `sudo` accepts these calls, that the Pi's root can read the venv, or
# that WiringPi then works. Those need the Pi (docs/guides/bench-test-tank-mode.md, step 1).
#
# Exit status: 0 = all checks passed, 1 = a check failed.
set -u
repo="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
work="$(mktemp -d)"
trap 'rm -rf "$work"' EXIT
pass=0; fail=0

check() {  # check <name> <condition exit status> [detail]
  if [ "$2" -eq 0 ]; then pass=$((pass + 1)); echo "ok    $1"
  else fail=$((fail + 1)); echo "FAIL  $1"; [ -n "${3:-}" ] && printf '%s\n' "$3" | sed 's/^/        /'; fi
}
has() { grep -qF -- "$2" <<<"$1"; }   # has <text> <fixed string>

# --- a fake machine ---
ws="$work/ws"; home="$work/home"; fakebin="$work/bin"; rosbin="$work/ros/bin"
mkdir -p "$ws/install" "$home/.venv/bin" "$fakebin" "$rosbin"
cat >"$work/ros/setup.bash" <<EOF
export FAKE_ROS_SOURCED=1
export PATH="$rosbin:\$PATH"
EOF
echo 'export FAKE_WS_SOURCED=1' >"$ws/install/setup.bash"
echo 'export VIRTUAL_ENV="'"$home"'/.venv"' >"$home/.venv/bin/activate"
cat >"$rosbin/ros2" <<'EOF'
#!/bin/sh
echo "ros2 $*"
echo "seen: domain=$ROS_DOMAIN_ID pwd=$PWD venv=$VIRTUAL_ENV ws=$FAKE_WS_SOURCED nobyte=$PYTHONDONTWRITEBYTECODE"
exit "${FAKE_ROS2_EXIT:-0}"
EOF
# Like the real sudo with env_reset: the caller's environment is DROPPED (HOME becomes another folder,
# PATH is kept as sudo's secure_path would). Without this a script that forgot to hand a setting over
# would still pass, because the fake would let it through. FAKE_ROS2_EXIT is the test's own knob.
cat >"$fakebin/sudo" <<'EOF'
#!/bin/sh
echo "sudo $*" >>"$SUDO_LOG"
exec env -i HOME=/nonexistent-root-home PATH="$PATH" FAKE_ROS2_EXIT="${FAKE_ROS2_EXIT:-}" "$@"
EOF
chmod +x "$rosbin/ros2" "$fakebin/sudo"
export SUDO_LOG="$work/sudo.log"; : >"$SUDO_LOG"
# What the scripts get from the caller. HOME is the fake home, as the Pi user's would be.
export HOME="$home" PATH="$fakebin:$PATH" ROCKO_WS="$ws" ROCKO_ROS_SETUP="$work/ros/setup.bash"
unset ROCKO_VENV ROCKO_DOMAIN_ID ROS_DOMAIN_ID

# 1. rocko_env.sh, sourced, sets up the terminal
out="$(bash -c 'source "$1" 2>&1; echo "now: domain=$ROS_DOMAIN_ID pwd=$PWD ros=$FAKE_ROS_SOURCED ws=$FAKE_WS_SOURCED venv=$VIRTUAL_ENV"' _ "$repo/pi/rocko_env.sh")"
check "rocko_env.sh: ROS, workspace overlay, venv, domain 17, cd to the workspace" \
  "$(has "$out" "now: domain=17 pwd=$ws ros=1 ws=1 venv=$home/.venv" && echo 0 || echo 1)" "$out"

# 2. a fresh checkout (no venv, no build) is reported, not fatal
out="$(ROCKO_VENV="$work/nope" ROCKO_WS="$work/empty" bash -c 'mkdir -p "$ROCKO_WS"; source "$1" 2>&1; echo "pwd=$PWD"' _ "$repo/pi/rocko_env.sh")"
r=0; has "$out" "no Python venv" || r=1; has "$out" "not found: build first" || r=1; has "$out" "pwd=$work/empty" || r=1
check "rocko_env.sh: missing venv and missing build are reported and the shell still gets set up" "$r" "$out"

# 3. rocko_run.sh: root (via sudo), environment, command, exit status
: >"$SUDO_LOG"
out="$("$repo/pi/rocko_run.sh" ros2 control list_controllers 2>&1)"
r=0
if [ "$(id -u)" -ne 0 ]; then has "$(cat "$SUDO_LOG")" "sudo env ROCKO_VENV=$home/.venv" || r=1; fi
has "$out" "ros2 control list_controllers" || r=1
has "$out" "seen: domain=17 pwd=$ws venv=$home/.venv ws=1 nobyte=1" || r=1
check "rocko_run.sh: runs the command through sudo, in the environment, in the workspace, with no .pyc" "$r" "$out
$(cat "$SUDO_LOG")"

FAKE_ROS2_EXIT=7 "$repo/pi/rocko_run.sh" ros2 anything >/dev/null 2>&1; code=$?
check "rocko_run.sh: the command's exit status (7) is the script's" "$([ "$code" -eq 7 ] && echo 0 || echo 1)" "got $code"

"$repo/pi/rocko_run.sh" >/dev/null 2>&1; code=$?
check "rocko_run.sh: no command is a usage error (2), not a root shell" "$([ "$code" -eq 2 ] && echo 0 || echo 1)" "got $code"

out="$(ROCKO_DOMAIN_ID=23 "$repo/pi/rocko_run.sh" ros2 x 2>&1)"
check "rocko_run.sh: ROCKO_DOMAIN_ID overrides the default 17 inside the root environment" \
  "$(has "$out" "domain=23" && echo 0 || echo 1)" "$out"

# 4. rocko_shell.sh: starts an interactive root bash that has sourced rocko_env.sh
: >"$SUDO_LOG"
out="$(printf 'echo "shell: domain=$ROS_DOMAIN_ID pwd=$PWD ros=$FAKE_ROS_SOURCED"\nexit\n' | "$repo/pi/rocko_shell.sh" 2>&1)"
r=0; has "$out" "shell: domain=17 pwd=$ws ros=1" || r=1
if [ "$(id -u)" -ne 0 ]; then has "$(cat "$SUDO_LOG")" "bash -c source" || r=1; has "$(cat "$SUDO_LOG")" "bash --norc -i" || r=1; fi
check "rocko_shell.sh: an interactive root shell (through rocko_run.sh and sudo), already in the environment" "$r" "$out
$(cat "$SUDO_LOG")"

# 5. launch.sh
out="$("$repo/pi/launch.sh" tank 2>&1)"
check "launch.sh tank: ros2 launch rocko_env rocko_tank_mode.launch.py, recordings to ~/rocko_logs" \
  "$(has "$out" "ros2 launch rocko_env rocko_tank_mode.launch.py log_root:=$home/rocko_logs" && echo 0 || echo 1)" "$out"

out="$("$repo/pi/launch.sh" balance record:=true battery_v:=12.1 2>&1)"
check "launch.sh balance: rocko.launch.py, the caller's arguments kept, in order, before log_root" \
  "$(has "$out" "ros2 launch rocko_env rocko.launch.py record:=true battery_v:=12.1 log_root:=$home/rocko_logs" && echo 0 || echo 1)" "$out"

out="$("$repo/pi/launch.sh" tank log_root:=/data/runs gui:=true 2>&1)"
r=0; has "$out" "rocko_tank_mode.launch.py log_root:=/data/runs gui:=true" || r=1
[ "$(grep -o 'log_root:=' <<<"$out" | wc -l)" -eq 1 ] || r=1
check "launch.sh: an explicit log_root:= is kept and not added twice" "$r" "$out"

: >"$SUDO_LOG"
out="$("$repo/pi/launch.sh" flying 2>&1)"; code=$?
r=0; [ "$code" -eq 2 ] || r=1; has "$out" "unknown mode 'flying'" || r=1; [ ! -s "$SUDO_LOG" ] || r=1
check "launch.sh: an unknown mode is refused (2) before sudo or ros2 is touched" "$r" "code $code: $out"

"$repo/pi/launch.sh" >/dev/null 2>&1; code=$?
check "launch.sh: no mode is a usage error (2)" "$([ "$code" -eq 2 ] && echo 0 || echo 1)" "got $code"

# 6. build_ros.sh finds its workspace: the checkout itself on the Pi, ROS2_WS (with a src/) in the container
for tool in apt-get rosdep colcon; do
  printf '#!/bin/sh\necho "%s $* [in $PWD]"\n' "$tool" >"$fakebin/$tool"; chmod +x "$fakebin/$tool"
done
mkdir -p "$work/checkout/rocko_env" "$work/checkout/rocko_interfaces"
cp "$repo/build_ros.sh" "$work/checkout/build_ros.sh"
out="$(env -u ROS2_WS "$work/checkout/build_ros.sh" 2>&1)"; code=$?
r=0; [ "$code" -eq 0 ] || r=1
has "$out" "rosdep install --from-paths rocko_env rocko_interfaces --ignore-src -r -y [in $work/checkout]" || r=1
has "$out" "colcon build --symlink-install" || r=1
has "$out" "[in $work/checkout]" || r=1
check "build_ros.sh on the Pi: the checkout is the workspace (no src/, no ros2_ws); rosdep looks at the two packages" "$r" "$out"

mkdir -p "$work/cws/src"
out="$(ROS2_WS="$work/cws" "$work/checkout/build_ros.sh" 2>&1)"; code=$?
r=0; [ "$code" -eq 0 ] || r=1
has "$out" "rosdep install --from-paths src --ignore-src -r -y [in $work/cws]" || r=1
check "build_ros.sh in the container: ROS2_WS is the workspace and src/ is searched" "$r" "$out"

# 7. `sudo pi/launch.sh ...` (root with SUDO_USER set, so HOME is /root) is refused, not half-working
mkdir -p "$work/asroot"
printf '#!/bin/sh\nif [ "$1" = "-u" ]; then echo 0; else echo root; fi\n' >"$work/asroot/id"; chmod +x "$work/asroot/id"
for script in "launch.sh tank" "rocko_run.sh ros2 x"; do
  out="$(PATH="$work/asroot:$PATH" SUDO_USER=rocko "$repo/pi/${script%% *}" ${script#* } 2>&1)"; code=$?
  r=0; [ "$code" -eq 2 ] || r=1; has "$out" "without sudo" || r=1; has "$out" "ros2 launch" && r=1
  check "${script%% *}: run under sudo is refused (2) with the reason" "$r" "code $code: $out"
done

# 8. the boot services: each unit is rendered with the caller's paths; installing under sudo is refused
out="$(HOME="$home" ROCKO_VENV="$home/myvenv" ROCKO_DOMAIN_ID=23 "$repo/pi/install_boot_service.sh" dashboard print 2>&1)"; code=$?
r=0; [ "$code" -eq 0 ] || r=1
has "$out" "Environment=ROCKO_VENV=$home/myvenv" || r=1
has "$out" "Environment=ROCKO_DOMAIN_ID=23" || r=1
has "$out" "Environment=HOME=/root" || r=1
has "$out" "ExecStart=$repo/pi/rocko_run.sh ros2 run rocko_env Dashboard.py" || r=1
has "$out" "log_root:=$home/rocko_logs" || r=1
grep -qE '@[A-Z_]+@' <<<"$out" && r=1   # no @PLACEHOLDER@ left behind
check "install_boot_service.sh dashboard print: this checkout, the caller's venv, logs, domain, HOME=/root" "$r" "$out"
out="$(HOME="$home" ROCKO_VENV="$home/myvenv" "$repo/pi/install_boot_service.sh" tank print 2>&1)"; code=$?
r=0; [ "$code" -eq 0 ] || r=1
has "$out" "ExecStart=$repo/pi/launch.sh tank log_root:=$home/rocko_logs" || r=1
has "$out" "Environment=ROCKO_BOOT_SERVICE=1" || r=1
has "$out" "KillSignal=SIGINT" || r=1
has "$out" "Environment=HOME=/root" || r=1
grep -qE '@[A-Z_]+@' <<<"$out" && r=1
check "install_boot_service.sh tank print: pi/launch.sh tank, the service marker, a SIGINT stop, HOME=/root" "$r" "$out"
out="$(PATH="$work/asroot:$PATH" "$repo/pi/install_boot_service.sh" tank install 2>&1)"; code=$?   # the asroot fake id says uid 0
r=0; [ "$code" -eq 2 ] || r=1; has "$out" "without sudo" || r=1
check "install_boot_service.sh: install under sudo is refused (2) with the reason" "$r" "code $code: $out"
out="$("$repo/pi/install_boot_service.sh" balance 2>&1)"; code=$?
r=0; [ "$code" -eq 2 ] || r=1; has "$out" "usage" || r=1
check "install_boot_service.sh: an unknown service name is refused with the usage" "$r" "code $code: $out"

# 9. while the tank boot service runs, launch.sh refuses other launches; the service itself is let through
mkdir -p "$work/tankup"
# fake systemctl: reports "active" (exit 0) only for 'is-active --quiet rocko-tank.service'; anything else exits 3 (not active)
printf '#!/bin/sh\n[ "$1 $2 $3" = "is-active --quiet rocko-tank.service" ] && exit 0\nexit 3\n' >"$work/tankup/systemctl"
chmod +x "$work/tankup/systemctl"
out="$(PATH="$work/tankup:$PATH" "$repo/pi/launch.sh" balance 2>&1)"; code=$?
r=0; [ "$code" -eq 2 ] || r=1; has "$out" "sudo systemctl stop rocko-tank.service" || r=1; has "$out" "ros2 launch" && r=1
check "launch.sh balance while rocko-tank.service runs: refused (2), says how to stop it" "$r" "code $code: $out"
out="$(PATH="$work/tankup:$PATH" ROCKO_BOOT_SERVICE=1 "$repo/pi/launch.sh" tank 2>&1)"; code=$?
r=0; [ "$code" -eq 0 ] || r=1; has "$out" "ros2 launch rocko_env rocko_tank_mode.launch.py" || r=1
check "launch.sh tank from the service itself (ROCKO_BOOT_SERVICE=1): let through" "$r" "code $code: $out"

echo
echo "$pass passed, $fail failed"
[ "$fail" -eq 0 ]
