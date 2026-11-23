#!/usr/bin/env bash
# Install, check or remove the systemd service that starts the web dashboard when the Pi boots.
# Run it ON THE PI, as YOUR user (it calls sudo itself), from anywhere:
#
#     pi/install_dashboard_service.sh            # install, enable at boot, start now
#     pi/install_dashboard_service.sh status     # is it enabled and running?
#     pi/install_dashboard_service.sh remove     # stop, disable, delete
#     pi/install_dashboard_service.sh print      # print the unit it would install; changes nothing
#
# The unit is pi/rocko-dashboard.service with YOUR paths filled in: this checkout, your venv
# (ROCKO_VENV, default ~/.venv), your ~/rocko_logs, and ROCKO_DOMAIN_ID (default 17, as pi/rocko_env.sh).
# Re-run it after moving the checkout. Guide: docs/guides/use-the-dashboard.md, "Start it at boot".
set -eu
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd "$here/.." && pwd)"
name=rocko-dashboard.service
target="/etc/systemd/system/$name"
action="${1:-install}"

# Why sed: a systemd unit has no variables (it cannot read $HOME or find this checkout), and the service runs as
# root, whose HOME is /root. So the paths are pasted in. "|" is sed's separator here: a path containing "|"
# would break it (not expected on the Pi).
render() {
  sed -e "s|@REPO@|$repo|g" \
      -e "s|@VENV@|${ROCKO_VENV:-$HOME/.venv}|g" \
      -e "s|@LOG_ROOT@|$HOME/rocko_logs|g" \
      -e "s|@DOMAIN_ID@|${ROCKO_DOMAIN_ID:-17}|g" \
      "$here/rocko-dashboard.service"
}

if [ "$(id -u)" -eq 0 ] && [ "$action" != print ]; then
  echo "install_dashboard_service.sh: run this as your own user, without sudo (it calls sudo itself)." >&2
  exit 2
fi

case "$action" in
  print)
    render ;;
  install)
    mkdir -p "$HOME/rocko_logs"   # made as you, so recordings land in a folder you own
    render | sudo tee "$target" >/dev/null
    sudo systemctl daemon-reload
    sudo systemctl enable --now "$name"
    echo "Installed $target; enabled at boot; started."
    echo "Check: pi/install_dashboard_service.sh status   (the page: http://$(hostname -I | cut -d' ' -f1):8080)" ;;
  status)
    echo "enabled at boot: $(systemctl is-enabled "$name" 2>/dev/null || echo no)"
    echo "running now:     $(systemctl is-active "$name" 2>/dev/null || echo no)"
    # A climbing count means it keeps exiting: normal while a launch's dashboard holds port 8080, a fault otherwise.
    echo "restarts:        $(systemctl show -p NRestarts --value "$name" 2>/dev/null)"
    echo "last log lines (sudo journalctl -u $name for more):"
    sudo journalctl -u "$name" -n 5 --no-pager 2>/dev/null | sed 's/^/  /' ;;
  remove)
    sudo systemctl disable --now "$name" 2>/dev/null || true
    sudo rm -f "$target"
    sudo systemctl daemon-reload
    echo "Removed $name." ;;
  *)
    echo "usage: pi/install_dashboard_service.sh [install|status|remove|print]" >&2
    exit 2 ;;
esac
