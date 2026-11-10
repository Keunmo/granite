#!/bin/bash
# Copies the Pi 5 cross build, the live config, the live script and the desktop icon to the Pi 5:
#   ~/deploy/granite/bin/      granite_vio, granite_pi_cam_imu_vio, granite_mono_vio_dataset
#   ~/deploy/granite/lib/      libgranite.so (RPATH $ORIGIN/../lib)
#   ~/deploy/granite/config/   cross/pi5/live/*.json
#   ~/deploy/granite/scripts/  pi5_live_kurokesu_vio.sh and its icon
#   ~/deploy/granite/runs/     one folder for each live run (the live script writes it)
#   ~/deploy/granite/DEPLOYED  the commit and the date
#   ~/Desktop/granite-live-kurokesu.desktop
# Each binary goes to a temporary name and then gets its final name: a running granite_vio keeps its old files.
#
# Usage: cross/pi5/deploy.sh [host]   (default 10.20.10.60)
# Env:   BUILD_ROOT (default ~/cross/pi5-build), DEPLOY_DIR (default deploy/granite, in the home folder of the Pi)
set -eu
HERE=$(cd "$(dirname "$0")" && pwd)
SRC=$(cd "$HERE/../.." && pwd)
HOST=${1:-10.20.10.60}
D=${DEPLOY_DIR:-deploy/granite}
G=${BUILD_ROOT:-$HOME/cross/pi5-build}/granite
BINS="granite_vio granite_pi_cam_imu_vio granite_mono_vio_dataset"
SHA=$(git -C "$SRC" rev-parse --short HEAD)
[ -z "$(git -C "$SRC" status --porcelain -uno)" ] || SHA="$SHA-dirty"

ssh "$HOST" "mkdir -p ~/$D/bin ~/$D/lib ~/$D/config ~/$D/scripts ~/$D/runs ~/Desktop"
for f in $BINS; do
  scp -q "$G/$f" "$HOST:$D/bin/.$f.new"
done
scp -q "$G/libgranite.so" "$HOST:$D/lib/.libgranite.so.new"
ssh "$HOST" "cd ~/$D && for f in $BINS; do mv -f bin/.\$f.new bin/\$f; done && mv -f lib/.libgranite.so.new lib/libgranite.so &&
  echo $SHA \$(date -Is) > DEPLOYED && cat DEPLOYED"
scp -q "$HERE"/live/*.json "$HOST:$D/config/"
scp -q "$HERE/pi5_live_kurokesu_vio.sh" "$HOST:$D/scripts/pi5_live_kurokesu_vio.sh"
scp -q "$SRC/scripts/pi4_live_mono_vio.svg" "$HOST:$D/scripts/pi5_live_kurokesu_vio.svg"
scp -q "$HERE/granite-live-kurokesu.desktop" "$HOST:Desktop/granite-live-kurokesu.desktop"
ssh "$HOST" "chmod +x ~/$D/scripts/pi5_live_kurokesu_vio.sh ~/Desktop/granite-live-kurokesu.desktop &&
  DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/\$(id -u)/bus gio set ~/Desktop/granite-live-kurokesu.desktop metadata::trusted true"
echo "Deployed to $HOST:~/$D"
