#!/usr/bin/env bash
# Copy recorded runs from the robot to this laptop. Run on the LAPTOP:
#   tools/fetch_logs.sh pi@192.168.1.42                 # every run in ~/rocko_logs on the Pi -> ./rocko_logs/
#   tools/fetch_logs.sh pi@192.168.1.42 ~/Desktop/logs  # somewhere else on the laptop
#
# It uses rsync over ssh, so it asks for the Pi's password unless you have set up an ssh key, copies
# only what is new, and can be run again after an interrupted copy. It NEVER deletes anything on the
# Pi: once you have checked the copy, free the Pi's disk yourself (the recorder refuses to start
# when the disk is nearly full):   ssh -t pi@<ip> 'sudo rm -r ~/rocko_logs/<run folder>'
#
# Each run is a folder: run.yaml (what was run) and bag/ (open the .mcap file in Foxglove).
set -eu
if [ $# -lt 1 ] || [ $# -gt 2 ]; then sed -n '2,10p' "$0"; exit 2; fi
robot="$1"
dest="${2:-./rocko_logs}"
mkdir -p "$dest"
# "rocko_logs" must be the folder RecordRun.py records into (its DEFAULT_LOG_ROOT, ~/rocko_logs). A
# run made with another log_root:= is not copied by this script.
# The trailing slash on the source copies the CONTENTS of ~/rocko_logs, not the folder itself.
# --progress, not --info=progress2: the rsync that ships with macOS does not know the newer option.
rsync --archive --partial --progress "$robot:rocko_logs/" "$dest/"
echo "Copied to $dest:"
ls -1 "$dest"
