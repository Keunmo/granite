#!/bin/bash
# Live mono VIO on the Raspberry Pi 5 rig (Kurokesu AR0234 on CAM/DISP 1 and the PX4 FC IMU through
# mavlink-routerd). Runs granite_vio --live with the Pangolin GUI and records the run as EuRoC.
#
# Usage: pi5_live_kurokesu_vio.sh [--no-record] [--no-gui] [--threads N] [extra granite_vio options]
# Env:   GRANITE_DEPLOY (default ~/deploy/granite), GRANITE_THREADS (default 3: the TBB worker limit)
#
# Outputs in $GRANITE_DEPLOY/runs/<date_time>/: run_traj_tum.txt, run_states.csv, run.log, euroc/ (recording).
# Ctrl-C or closing the window ends the run and writes the results. A second Ctrl-C ends the program at once.
set -u

E=${GRANITE_DEPLOY:-$HOME/deploy/granite}
B=$E/bin
C=$E/config
CALIB=$C/kurokesu_448x400_calib.json
CONFIG=$C/kurokesu_448x400_handheld.json

record=1
gui=1
threads=${GRANITE_THREADS:-3}
extra=()
while [ $# -gt 0 ]; do
  case "$1" in
    --no-record) record=0 ;;
    --no-gui) gui=0 ;;
    --threads) shift; threads=${1:?--threads needs a number} ;;
    *) extra+=("$1") ;;
  esac
  shift
done

# Keep the terminal open after the run when the desktop icon starts the script.
finish() {
  local status=$1
  if [ "${GRANITE_PAUSE:-0}" = 1 ]; then
    echo
    read -r -p "Exit status $status. Press Enter to close this window." _
  fi
  exit "$status"
}

for f in "$B/granite_vio" "$CALIB" "$CONFIG"; do
  if [ ! -e "$f" ]; then
    echo "Missing $f"
    finish 1
  fi
done

# The FC IMU comes through mavlink-routerd (UDP 127.0.0.1:14601).
if ! systemctl is-active --quiet mavlink-router; then
  echo "mavlink-router is not active. Start it: sudo systemctl start mavlink-router"
  finish 1
fi

# The camera accepts only one process.
users=$(pgrep -af "granite_vio|granite_pi_cam_imu_vio|bin/vio |xeye-deploy/bin/capture|ar0234_view" | grep -v pgrep)
if [ -n "$users" ]; then
  echo "Another program uses the camera:"
  echo "$users"
  finish 1
fi

OUT=$E/runs/$(date +%Y%m%d_%H%M%S)
mkdir -p "$OUT"
args=(--live --use-imu 1 --cam-calib "$CALIB" --config-path "$CONFIG"
      --num-threads "$threads" --out "$OUT/run")
[ "$record" = 1 ] && args+=(--record "$OUT/euroc")
[ "$gui" = 0 ] && args+=(--show-gui 0)

echo "Deployed: $(cat "$E/DEPLOYED" 2>/dev/null)"
echo "TBB threads: $threads  Output: $OUT"
echo "Keep the rig still for 2 s, then move it clearly within 1 s. Ctrl-C or close the window to stop."
cd "$E" || finish 1
# tee ignores Ctrl-C: granite_vio writes its results and the LIVE line after Ctrl-C.
"$B/granite_vio" "${args[@]}" "${extra[@]}" 2>&1 | { trap '' INT; tee "$OUT/run.log"; }
status=${PIPESTATUS[0]}
echo "Results: $OUT"
finish "$status"
