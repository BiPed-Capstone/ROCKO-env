#!/usr/bin/env bash
# Install, check or remove a systemd service that starts part of ROCKO when the Pi boots.
# Run it ON THE PI, as YOUR user (it calls sudo itself), from anywhere:
#
#     pi/install_boot_service.sh robot               # the supervisor: tank at boot, controller/dashboard switch mode
#                                                    # (pi/rocko.service). Replaces tank and dashboard (removes them)
#     pi/install_boot_service.sh dashboard           # the web dashboard (pi/rocko-dashboard.service)
#     pi/install_boot_service.sh tank                # tank mode, driven by the USB controller (pi/rocko-tank.service)
#     pi/install_boot_service.sh tank status         # is it enabled and running?
#     pi/install_boot_service.sh tank remove         # stop, disable, delete
#     pi/install_boot_service.sh tank print          # print the unit it would install; changes nothing
#
# Each unit is pi/rocko-<name>.service (robot: pi/rocko.service) with YOUR paths filled in: this checkout, your venv (ROCKO_VENV,
# default ~/.venv), your ~/rocko_logs, and ROCKO_DOMAIN_ID (default 17, as pi/rocko_env.sh).
# Re-run it after moving the checkout. Guides: docs/guides/use-the-dashboard.md ("Start it at boot") and
# docs/guides/drive-with-usb-controller.md ("Start tank mode at boot"), docs/guides/run-on-the-pi.md.
set -eu
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd "$here/.." && pwd)"
usage() { echo "usage: pi/install_boot_service.sh robot|dashboard|tank [install|status|remove|print]" >&2; exit 2; }
case "${1:-}" in robot|dashboard|tank) ;; *) usage ;; esac
if [ "$1" = robot ]; then name="rocko.service"; else name="rocko-$1.service"; fi
template="$here/$name"
unit_dir="${ROCKO_UNIT_DIR:-/etc/systemd/system}"   # only the host test changes this
target="$unit_dir/$name"
action="${2:-install}"

# Why sed: a systemd unit has no variables (it cannot read $HOME or find this checkout), and the service runs as
# root, whose HOME is /root. So the paths are pasted in. "|" is sed's separator here: a path containing "|"
# would break it (not expected on the Pi).
render() {
  sed -e "s|@REPO@|$repo|g" \
      -e "s|@VENV@|${ROCKO_VENV:-$HOME/.venv}|g" \
      -e "s|@LOG_ROOT@|$HOME/rocko_logs|g" \
      -e "s|@DOMAIN_ID@|${ROCKO_DOMAIN_ID:-17}|g" \
      "$template"
}

# print changes nothing, so it is allowed under sudo; everything else must run as the user.
if [ "$(id -u)" -eq 0 ] && [ "$action" != print ]; then
  echo "install_boot_service.sh: run this as your own user, without sudo (it calls sudo itself)." >&2
  exit 2
fi

case "$action" in
  print)
    render ;;
  install)
    if [ "$1" = robot ]; then
      # Seen 2026-11-23: installed after a `git pull` without a rebuild, the service restarted three times on
      # "file 'rocko_robot.launch.py' was not found" and gave up. Catch that here, where the fix can be printed.
      built="$repo/install/rocko_env/share/rocko_env/launch/rocko_robot.launch.py"
      if [ -z "${ROCKO_SKIP_BUILD_CHECK:-}" ] && [ ! -e "$built" ]; then
        echo "install_boot_service.sh: this checkout is not built with the supervisor yet ($built is missing)." >&2
        echo "  Build first:  ./build_ros.sh   then run this again." >&2
        exit 2
      fi
      # rocko.service runs everything the other two start, so they conflict with it (two launches on the motors,
      # two dashboards on port 8080). Replace them, and say so.
      for old in rocko-tank.service rocko-dashboard.service; do
        if [ -e "$unit_dir/$old" ]; then
          echo "Removing $old: rocko.service replaces it (they would fight over the motors / port 8080)."
          sudo systemctl disable --now "$old" 2>/dev/null || true
          sudo rm -f "$unit_dir/$old"
        fi
      done
    elif [ -e "$unit_dir/rocko.service" ]; then
      echo "install_boot_service.sh: rocko.service is installed, and it already starts the $1 part (and more)." >&2
      echo "  Installing $name too would fight it. Remove it first: pi/install_boot_service.sh robot remove" >&2
      exit 2
    fi
    mkdir -p "$HOME/rocko_logs"   # made as you, so recordings land in a folder you own
    render | sudo tee "$target" >/dev/null
    sudo systemctl daemon-reload
    sudo systemctl enable --now "$name"
    echo "Installed $target; enabled at boot; started."
    echo "Check: pi/install_boot_service.sh $1 status" ;;
  status)
    # These commands print their answer ("disabled", "failed") AND exit non-zero for it, so "|| echo no" would print
    # both lines; "no" is only for an empty answer (systemctl missing).
    at_boot="$(systemctl is-enabled "$name" 2>/dev/null)" || true
    running="$(systemctl is-active "$name" 2>/dev/null)" || true
    echo "enabled at boot: ${at_boot:-no}"
    echo "running now:     ${running:-no}"
    # A climbing count means it keeps exiting (the dashboard does that, harmlessly, while a launch's own dashboard
    # holds port 8080; for tank mode it means the launch keeps failing: read the log).
    echo "restarts:        $(systemctl show -p NRestarts --value "$name" 2>/dev/null)"
    echo "last log lines (sudo journalctl -u $name for more):"
    sudo journalctl -u "$name" -n 5 --no-pager 2>/dev/null | sed 's/^/  /' ;;
  remove)
    sudo systemctl disable --now "$name" 2>/dev/null || true
    sudo rm -f "$target"
    sudo systemctl daemon-reload
    echo "Removed $name." ;;
  *)
    usage ;;
esac
