#!/usr/bin/env bash
# Launch ROCKO as root, in ROCKO's environment.
#
#     pi/launch.sh tank                          rocko_tank_mode.launch.py (no balancing; drives)
#     pi/launch.sh balance                       rocko.launch.py (balancing)
#     pi/launch.sh tank gui:=true                anything after the mode goes to `ros2 launch`
#     pi/launch.sh balance record:=true record_label:=first_test battery_v:=12.1
#
# Refuses to start while the tank-mode boot service (rocko-tank.service) is running: stop that first.
#
# Hardware is real unless you pass use_mock_hardware:=true (the launch files' own default).
# Run it as YOUR user, without `sudo` in front (see rocko_run.sh).
set -eu
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

usage() {
  cat >&2 <<'USAGE'
usage: pi/launch.sh tank|balance [launch arguments...]
  pi/launch.sh tank
  pi/launch.sh balance record:=true record_label:=first_test battery_v:=12.1
USAGE
  exit 2
}
[ $# -ge 1 ] || usage
mode="$1"; shift
case "$mode" in
  tank)    launch_file=rocko_tank_mode.launch.py ;;
  balance) launch_file=rocko.launch.py ;;
  *)       echo "launch.sh: unknown mode '$mode' (use tank or balance)" >&2; usage ;;
esac
if [ "$(id -u)" -eq 0 ] && [ -n "${SUDO_USER:-}" ]; then
  echo "launch.sh: run this as your own user, without sudo (it calls sudo itself)." >&2; exit 2
fi
# Two launches would fight over the motors and the GPIO pins. If tank mode was started at boot by its
# service (pi/rocko-tank.service), refuse every other launch until it is stopped. The service itself sets
# ROCKO_BOOT_SERVICE=1 and is let through. No systemctl (a laptop, the container): nothing to check.
if [ -z "${ROCKO_BOOT_SERVICE:-}" ] && command -v systemctl >/dev/null 2>&1 \
   && systemctl is-active --quiet rocko-tank.service 2>/dev/null; then
  echo "launch.sh: tank mode is already running as the boot service (rocko-tank.service)." >&2
  echo "  Stop it first:  sudo systemctl stop rocko-tank.service   (it starts again at the next boot)" >&2
  echo "  Never at boot:  pi/install_boot_service.sh tank remove" >&2
  exit 2
fi

# Recordings go where tools/fetch_logs.sh looks: YOUR ~/rocko_logs. Left alone, the launch (running as
# root) would write to /root/rocko_logs, which fetch_logs.sh cannot see. An explicit log_root:= wins.
# The folder is made here, as you, so it is yours; the run folders inside it are made by root and stay
# root-owned (delete one with sudo).
have_log_root=0
for argument in "$@"; do
  case "$argument" in log_root:=*) have_log_root=1 ;; esac
done
args=("$@")
if [ "$have_log_root" -eq 0 ]; then
  mkdir -p "$HOME/rocko_logs"
  args+=("log_root:=$HOME/rocko_logs")
fi

exec "$here/rocko_run.sh" ros2 launch rocko_env "$launch_file" "${args[@]}"
