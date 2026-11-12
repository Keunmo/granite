#!/usr/bin/env bash
# Install the Kurokesu AR0234 camera driver on a Raspberry Pi 5 with Ubuntu 24.04.
#
#   sudo ./install_ar0234_driver.sh [--port cam1|cam0] [--with-viewer] [DRIVER_DIR]
#   ./install_ar0234_driver.sh --check
#
# DRIVER_DIR is a copy of the ar0234-rpi-driver repository on the branch
# rpi5-ubuntu2404. The default is resources/ar0234-rpi-driver of this
# directory. The script does these steps:
#
#   1. Install the kernel headers of the running kernel, dkms, dtc and v4l-utils.
#   2. Build and install the module and the overlay with DRIVER_DIR/setup.sh.
#   3. Edit /boot/firmware/config.txt (copy: config.txt.bak-ar0234):
#      camera_auto_detect=0, dtoverlay=ar0234 (CAM/DISP 1) or
#      dtoverlay=ar0234,cam0 (CAM/DISP 0). Other camera overlays on the same
#      port get a '#'.
#
# Step 1 needs the Ubuntu apt archive (internet). The script does not run
# 'apt full-upgrade' and does not reboot the Pi.
# Reboot after the script. Then run the script with --check.
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=lib/bootcfg.sh
source "$SCRIPT_DIR/lib/bootcfg.sh"

# Camera overlays that use a CSI port. Extend the list for other sensors.
CAMERA_OVERLAY_RE='^dtoverlay=(ar0234|arducam[-_a-z0-9]*|imx[0-9]+[a-z0-9]*|ov[0-9]+[a-z0-9]*)([,].*)?$'

usage() { awk 'NR > 1 && !/^#/ { exit } NR > 1 { sub(/^# ?/, ""); print }' "$0"; exit "${1:-0}"; }

check() {
  local ok=1 m media=""
  echo "Kernel: $(uname -r)"
  if dkms status 2>/dev/null | grep ar0234 >/dev/null; then
    dkms status | grep ar0234
  else
    echo "FAIL  dkms has no ar0234 module"; ok=0
  fi
  if [[ -f $BOOT_DIR/overlays/ar0234.dtbo ]]; then
    echo "OK    $BOOT_DIR/overlays/ar0234.dtbo"
  else
    echo "FAIL  $BOOT_DIR/overlays/ar0234.dtbo does not exist"; ok=0
  fi
  echo "config.txt:"
  grep -nE '^[[:space:]]*(camera_auto_detect|dtoverlay=ar0234)' "$CONFIG_TXT" | sed 's/^/      /' || true
  if grep -q '^ar0234 ' /proc/modules; then
    echo "OK    module ar0234 is loaded"
  else
    echo "FAIL  module ar0234 is not loaded (reboot after the install)"; ok=0
  fi
  for m in /dev/media*; do
    [[ -e $m ]] || continue
    if media-ctl -d "$m" -p 2>/dev/null | grep ar0234 >/dev/null; then media=$m; fi
  done
  if [[ -n $media ]]; then
    echo "OK    media graph $media has the ar0234 entity"
    media-ctl -d "$media" -p 2>/dev/null | grep -A2 -E 'entity .*rp1-cfe-csi2_ch0' | grep -o '/dev/video[0-9]*' | head -n 1 |
      sed 's/^/      rp1-cfe-csi2_ch0 = /'
  else
    echo "FAIL  no media graph has the ar0234 entity"; ok=0
  fi
  [[ $ok -eq 1 ]] && echo "RESULT OK" || { echo "RESULT FAIL"; return 1; }
}

PORT=cam1
VIEWER=0
DRIVER_DIR="$SCRIPT_DIR/resources/ar0234-rpi-driver"
while [[ $# -gt 0 ]]; do
  case $1 in
    --port) PORT=${2:?}; shift 2 ;;
    --with-viewer) VIEWER=1; shift ;;
    --check) check; exit $? ;;
    -h|--help) usage 0 ;;
    -*) echo "unknown option: $1" >&2; usage 1 ;;
    *) DRIVER_DIR=$1; shift ;;
  esac
done

[[ $PORT == cam1 || $PORT == cam0 ]] || die "--port must be cam1 or cam0"
DRIVER_DIR=$(realpath "$DRIVER_DIR")
require_root
require_pi5

# The branch rpi5-ubuntu2404 has ar0234_s_stream() for kernels before 6.11.
# Without it, STREAMON is successful but no frames come in on Ubuntu 6.8.
[[ -f $DRIVER_DIR/setup.sh && -f $DRIVER_DIR/ar0234.c ]] || die "$DRIVER_DIR is not an ar0234-rpi-driver tree"
grep -q 'ar0234_s_stream' "$DRIVER_DIR/ar0234.c" ||
  die "$DRIVER_DIR/ar0234.c has no ar0234_s_stream(). Use the branch rpi5-ubuntu2404."
grep -q '/boot/firmware/overlays' "$DRIVER_DIR/dkms.postinst" ||
  die "$DRIVER_DIR/dkms.postinst does not know /boot/firmware/overlays. Use the branch rpi5-ubuntu2404."
if git -C "$DRIVER_DIR" rev-parse --abbrev-ref HEAD >/dev/null 2>&1; then
  log SOURCE "$DRIVER_DIR ($(git -C "$DRIVER_DIR" rev-parse --abbrev-ref HEAD) $(git -C "$DRIVER_DIR" rev-parse --short HEAD))"
else
  log SOURCE "$DRIVER_DIR (see resources/SOURCES.md)"
fi

echo "== 1. Packages"
apt-get update
apt-get install -y "linux-headers-$(uname -r)" device-tree-compiler v4l-utils
apt-get install -y --no-install-recommends dkms
if [[ $VIEWER -eq 1 ]]; then
  apt-get install -y python3-numpy python3-tk python3-pil python3-pil.imagetk
fi

echo "== 2. Driver (DKMS) and overlay"
# A copy over zip or FAT can lose the execute bit. DKMS runs dkms.postinst.
chmod +x "$DRIVER_DIR/setup.sh" "$DRIVER_DIR/dkms.postinst"
(cd "$DRIVER_DIR" && sh ./setup.sh)
[[ -f $BOOT_DIR/overlays/ar0234.dtbo ]] || die "setup.sh did not install $BOOT_DIR/overlays/ar0234.dtbo"

echo "== 3. $CONFIG_TXT"
OVERLAY_LINE="dtoverlay=ar0234"
[[ $PORT == cam0 ]] && OVERLAY_LINE="dtoverlay=ar0234,cam0"
begin_config_edit
set_key camera_auto_detect 0
# Comment the other camera overlays of the same port. An overlay without a
# 'cam0' parameter uses CAM/DISP 1.
same_port() {
  if [[ $PORT == cam0 ]]; then [[ $1 == *cam0* ]]; else [[ $1 != *cam0* ]]; fi
}
comment_lines "$CAMERA_OVERLAY_RE" "$OVERLAY_LINE" same_port
if has_line_in_last_all "$OVERLAY_LINE"; then
  log OK "$OVERLAY_LINE"
else
  append_to_all "$OVERLAY_LINE"
fi
end_config_edit bak-ar0234

echo
echo "Done. The driver works after a reboot:"
echo "  sudo reboot"
echo "  bash $0 --check"
