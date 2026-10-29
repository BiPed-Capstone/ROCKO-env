#!/usr/bin/env bash
# Precondition for every tier of the verification ladder: does the dev container see the files of
# THIS checkout (the folder this script lives in)? Run it on the LAPTOP, not in the container:
#   ./check_container_mount.sh
#
# Why it matters: docker/compose.yml mounts `..` (the checkout that compose file lives in) at
# /ros2_ws/src/ROCKO-env and gives the container a fixed name, rocko-env-dev. Every checkout's
# compose file sits in a folder called `docker`, so compose treats them all as one project. A laptop
# therefore has ONE container, and it shows whichever checkout last ran
# `docker compose -f <checkout>/docker/compose.yml up -d`. Edit a different checkout (a git worktree,
# a second clone) and build_ros.sh, verify_urdf.sh and verify_mock.sh still run and still pass, but
# they test the OTHER checkout's files.
#
# How: create an empty file with a random name here, then ask the container whether a file of that
# name exists in /ros2_ws/src/ROCKO-env. It can only be there if that folder IS this checkout.
# The obvious alternative, comparing this folder's path with the mount source `docker inspect`
# reports, needs a rule per OS: Docker Desktop for macOS reports /Users/me/x as /host_mnt/Users/me/x,
# and Windows and WSL2 spell paths differently again. The marker asks the real question directly.
#
# What `ok` does not say: that the build output in the container came from this checkout. The build
# folder survives a switch, and make only recompiles files newer than their output. After a switch,
# rebuild from clean (the FAIL message prints the command).
#
# Exit status:  0 = the container sees this checkout   1 = it mounts another one (the fix is printed)
#               2 = could not tell (no docker here, Docker or the container is not running, or this
#                   folder is read-only)
#
# No `set -e`: each command that can fail is checked, and its failure becomes a message.
set -u

repo="$(cd "$(dirname "$0")" && pwd)"       # the checkout this script lives in
container_checkout=/ros2_ws/src/ROCKO-env    # where docker/compose.yml mounts a checkout (`volumes:`)

# compose <args> -> docker compose, with this checkout's compose file.
# </dev/null: `docker compose exec` reads stdin even with -T, so without this the script would eat
# input meant for whatever runs it (for example the rest of a `while read` loop).
compose() { docker compose -f "$repo/docker/compose.yml" "$@" </dev/null; }

if ! command -v docker >/dev/null; then
  echo "ERROR: no docker command here. Run this on the laptop, not in the container or on the Pi." >&2
  exit 2
fi
if [ ! -f "$repo/docker/compose.yml" ]; then
  echo "ERROR: no docker/compose.yml next to this script ($repo). Run it from its checkout, not via a symlink or a copy." >&2
  exit 2
fi

# Only stdout is thrown away: if this fails, Docker's own reason stays on screen, above ours.
if ! compose exec -T ros2 true >/dev/null; then
  echo "ERROR: cannot reach the 'ros2' container (Docker's reason is above). Start Docker, then:" >&2
  echo "  docker compose -f \"$repo/docker/compose.yml\" up -d" >&2
  exit 2
fi

marker="$(mktemp "$repo/.mount-check.XXXXXX")" \
  || { echo "ERROR: cannot create a marker file in $repo (is the folder writable?)" >&2; exit 2; }
trap 'rm -f "$marker"' EXIT   # runs on every exit, Ctrl+C included, so no marker is left behind

# The same file, at the path the container would see it if it mounts this checkout.
marker_in_container="$container_checkout/$(basename "$marker")"
if compose exec -T ros2 test -e "$marker_in_container"; then
  echo "ok    the container sees this checkout: $repo"
  exit 0
fi

echo "FAIL  the container does NOT see this checkout: $repo"
echo "      It mounts a different checkout, so build_ros.sh and the verify scripts would test THAT one's files."
echo "      Which one:  docker inspect -f '{{range .Mounts}}{{println .Source}}{{end}}' rocko-env-dev"
echo "      Either work in that checkout, or point the container here. Pointing it here recreates it,"
echo "      which stops anything running in it now and leaves the other checkout failing this check."
echo "      Add the same extra -f file you started it with (compose.mac.yml, compose.windows.yml), if any:"
echo "        docker compose -f \"$repo/docker/compose.yml\" up -d --force-recreate"
echo "      A new container has none of the ROS packages the old one installed, and the build folder it"
echo "      reuses was compiled from the other checkout, so rebuild from clean (needs the network):"
echo "        docker compose -f \"$repo/docker/compose.yml\" exec -T ros2 bash -c 'cd /ros2_ws && rm -rf build install log && src/ROCKO-env/build_ros.sh'"
exit 1
