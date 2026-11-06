#!/bin/bash
# Copies the Pi 4 cross build to ~/granite_eval/bin/granite/ on the Pi and writes the commit to
# bin/granite/DEPLOYED. The folder keeps only the newest build. Each file goes to a temporary name and
# then gets its final name: a running granite_vio keeps its old files.
#
# Usage: cross/pi4/deploy.sh [host]   (default 10.20.10.50)
#        --scripts also copies the live script, its icon and the desktop file.
# Env:   BUILD_ROOT (default ~/cross/pi4-build)
set -eu
HERE=$(cd "$(dirname "$0")" && pwd)
SRC=$(cd "$HERE/../.." && pwd)
HOST=10.20.10.50
SCRIPTS=0
for a in "$@"; do
  case "$a" in
    --scripts) SCRIPTS=1 ;;
    *) HOST=$a ;;
  esac
done
G=${BUILD_ROOT:-$HOME/cross/pi4-build}/granite
FILES="granite_vio granite_pi_cam_imu_vio granite_mono_vio_dataset libgranite.so"
SHA=$(git -C "$SRC" rev-parse --short HEAD)
[ -z "$(git -C "$SRC" status --porcelain -uno)" ] || SHA="$SHA-dirty"

ssh "$HOST" "mkdir -p ~/granite_eval/bin/granite"
for f in $FILES; do
  scp -q "$G/$f" "$HOST:granite_eval/bin/granite/.$f.new"
done
ssh "$HOST" "cd ~/granite_eval/bin/granite && for f in $FILES; do mv -f .\$f.new \$f; done && echo $SHA \$(date -Is) > DEPLOYED && cat DEPLOYED"

if [ "$SCRIPTS" = 1 ]; then
  S=$SRC/scripts
  scp -q "$S/pi4_live_mono_vio.sh" "$HOST:granite_eval/.pi4_live_mono_vio.sh.new"
  scp -q "$S/pi4_live_mono_vio.svg" "$HOST:granite_eval/pi4_live_mono_vio.svg"
  scp -q "$S/pi4_live_mono_vio.desktop" "$HOST:Desktop/pi4_live_mono_vio.desktop"
  ssh "$HOST" 'cd ~/granite_eval && chmod +x .pi4_live_mono_vio.sh.new && mv -f .pi4_live_mono_vio.sh.new pi4_live_mono_vio.sh &&
    chmod +x ~/Desktop/pi4_live_mono_vio.desktop && mkdir -p ~/.local/share/applications &&
    cp ~/Desktop/pi4_live_mono_vio.desktop ~/.local/share/applications/ &&
    DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/$(id -u)/bus gio set ~/Desktop/pi4_live_mono_vio.desktop metadata::trusted true'
  echo "Scripts and the desktop icon copied"
fi
