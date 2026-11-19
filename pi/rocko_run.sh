#!/usr/bin/env bash
# Run ONE command as root, in ROCKO's environment (see rocko_env.sh).
#
#     pi/rocko_run.sh ros2 control list_controllers
#     pi/rocko_run.sh ros2 topic echo /joint_states --field velocity
#
# Run it as YOUR user, not with `sudo` in front: it calls sudo itself, and a command already running
# under sudo has root's home (/root), so your venv and your ~/rocko_logs would not be found.
#
# Why root: the motor plugin drives the GPIO pins through WiringPi, which only works as root, so the
# launch runs as root. ROS 2 programs run by different users may fail to talk to each other (Fast DDS
# shares memory between processes, and that memory belongs to one user), so every ros2 command that must
# see the launch runs as root too. Symptom of a terminal that is not root: "waiting for service
# /controller_manager/list_controllers to become available". (Seen with the old helper; the cause is
# the usual one, not re-measured.)
#
# Run it from any directory: the command starts in the workspace (rocko_env.sh does the cd), so a
# relative path in your command means a path inside the workspace.
set -eu
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
[ $# -ge 1 ] || { echo "usage: pi/rocko_run.sh <command> [args...]" >&2; exit 2; }
if [ "$(id -u)" -eq 0 ] && [ -n "${SUDO_USER:-}" ]; then
  echo "rocko_run.sh: run this as your own user, without sudo (it calls sudo itself)." >&2; exit 2
fi

# sudo resets the environment and makes HOME /root, so hand over, by name, the settings that depend on
# YOUR environment. `sudo env VAR=...` works where `sudo VAR=... cmd` can be refused. Already root
# (a real root login): no sudo needed. $sudo_cmd is deliberately unquoted: empty means "nothing".
# PYTHONDONTWRITEBYTECODE: a root-owned __pycache__ must not appear inside your checkout.
sudo_cmd=sudo
[ "$(id -u)" -ne 0 ] || sudo_cmd=""
# shellcheck disable=SC2086
exec $sudo_cmd env \
  ROCKO_VENV="${ROCKO_VENV:-$HOME/.venv}" ROCKO_DOMAIN_ID="${ROCKO_DOMAIN_ID:-17}" \
  ROCKO_WS="${ROCKO_WS:-}" ROCKO_ROS_SETUP="${ROCKO_ROS_SETUP:-}" PYTHONDONTWRITEBYTECODE=1 \
  bash -c 'source "$1"; shift; exec "$@"' rocko_run "$here/rocko_env.sh" "$@"
