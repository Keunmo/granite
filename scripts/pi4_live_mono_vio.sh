#!/bin/bash
# Live mono VIO on the Raspberry Pi 4 rig (Arducam B0429 + MPU-9250 on a CP2112).
# Runs granite_vio --live with the Pangolin GUI and records the run as EuRoC.
#
# Usage: pi4_live_mono_vio.sh [--no-record] [--no-gui] [--threads N] [extra granite_vio options]
# Env:   GRANITE_EVAL (default ~/granite_eval), GRANITE_I2C (default: the CP2112 adapter),
#        GRANITE_THREADS (default 3: the TBB worker limit, --num-threads of granite_vio)
#
# Outputs in $GRANITE_EVAL/live/<date_time>/: run_traj_tum.txt, run_states.csv,
# run.log, euroc/ (recording). Ctrl-C or closing the window ends the run and
# writes the results. A second Ctrl-C ends the program at once.
set -u

E=${GRANITE_EVAL:-$HOME/granite_eval}
B=$E/bin/granite
C=$E/config/live
CALIB=$C/b0429_pi_640x400_calib.json
CONFIG=$C/b0429_pi_640x400_handheld.json

record=1
gui=1
threads=${GRANITE_THREADS:-3}
extra=()
while [ $# -gt 0 ]; do
  case "$1" in
    --no-record) record=0 ;;
    --no-gui) gui=0 ;;
    --threads) shift; threads=${1:?--threads needs a number} ;;
    --num-threads) shift; threads=${1:?--num-threads needs a number} ;;
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

# The CP2112 adapter number changes after a replug or a reboot.
I2C=${GRANITE_I2C:-}
if [ -z "$I2C" ]; then
  for d in /sys/bus/i2c/devices/i2c-*; do
    if grep -q CP2112 "$d/name" 2>/dev/null; then
      I2C=/dev/$(basename "$d")
      break
    fi
  done
fi
if [ -z "$I2C" ]; then
  echo "No CP2112 I2C adapter. Check the USB cable of the IMU."
  finish 1
fi

# The camera and the IMU accept only one process.
if pgrep -f "$B/granite_" > /dev/null; then
  echo "Another granite program uses the sensors:"
  pgrep -af "$B/granite_"
  finish 1
fi

OUT=$E/live/$(date +%Y%m%d_%H%M%S)
mkdir -p "$OUT"
args=(--live --use-imu 1 --cam-calib "$CALIB" --config-path "$CONFIG" --i2c "$I2C"
      --num-threads "$threads" --out "$OUT/run")
[ "$record" = 1 ] && args+=(--record "$OUT/euroc")
[ "$gui" = 0 ] && args+=(--show-gui 0)

echo "Deployed: $(cat "$B/DEPLOYED" 2>/dev/null)"
echo "IMU: $I2C  TBB threads: $threads  Output: $OUT"
echo "Keep the rig still for 2 s, then move it clearly within 1 s. Ctrl-C or close the window to stop."
cd "$E" || finish 1
# tee ignores Ctrl-C: granite_vio writes its results and the LIVE line after Ctrl-C.
"$B/granite_vio" "${args[@]}" "${extra[@]}" 2>&1 | { trap '' INT; tee "$OUT/run.log"; }
status=${PIPESTATUS[0]}
echo "Results: $OUT"
finish "$status"
