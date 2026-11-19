#!/usr/bin/env bash
# Open an interactive ROOT shell in ROCKO's environment (see rocko_env.sh). Use one per terminal on
# the Pi, then type ros2 commands as usual:
#
#     pi/rocko_shell.sh
#     ros2 control list_controllers
#
# Leave it with `exit`. The prompt ends in `#` (root). Why root, and why not `sudo` in front: see
# rocko_run.sh. --norc keeps root's own ~/.bashrc out, so the environment is exactly rocko_env.sh's.
set -eu
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "$here/rocko_run.sh" bash --norc -i
