#!/usr/bin/env bash
# Install the systemd service that starts the granite live VIO at boot.
#
#   sudo ./setup_vio_service.sh [--user USER] [--deploy DIR] [--use-deployed]
#                               [--skip-apt] [--no-enable]
#   ./setup_vio_service.sh --check
#   sudo ./setup_vio_service.sh --uninstall
#
# The script does these steps:
#
#   1. Install the Ubuntu packages of the granite programs (OpenCV, TBB,
#      GLEW, GL) with apt, only the missing ones. This step needs the
#      Ubuntu apt archive (internet). --skip-apt skips it. The script does
#      not run 'apt upgrade'.
#   2. Install the granite build of resources/vio/granite/ (the Pi 5 cross
#      build, the live config and the live script; see resources/SOURCES.md)
#      into DIR (default /home/USER/deploy/granite, USER default: the user
#      that runs sudo). The script checks the SHA256 of each file first.
#      The runs/ folder stays. --use-deployed skips the copy and uses the
#      granite of DIR (for example from cross/pi5/deploy.sh of the granite
#      repository).
#      When DIR/config/service/ does not exist, the script copies the 672x600
#      calibration and config of the bundle into it. The service reads this
#      folder. An existing folder stays: it can hold a calibration of
#      renew-config.sh.
#   3. Write resources/vio/granite-vio.service with USER and DIR to
#      /etc/systemd/system/granite-vio.service.
#   4. Write resources/vio/granite-vio.default to /etc/default/granite-vio
#      when the file does not exist. An existing file stays.
#   5. Enable the service (it starts at the next boot). --no-enable skips
#      this step. The script does not start the service, because the
#      service takes the camera and the FC IMU port.
#
# The service runs pi5_live_kurokesu_vio.sh --no-gui once for each boot
# (no restart). It sends the VIO state to the FC as ODOMETRY. Stop it
# before a capture or a calibration: sudo systemctl stop granite-vio
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=lib/bootcfg.sh
source "$SCRIPT_DIR/lib/bootcfg.sh"

SERVICE=granite-vio.service
UNIT_SRC="$SCRIPT_DIR/resources/vio/granite-vio.service"
UNIT_DST=/etc/systemd/system/$SERVICE
DEFAULT_SRC="$SCRIPT_DIR/resources/vio/granite-vio.default"
DEFAULT_DST=/etc/default/granite-vio
BUNDLE="$SCRIPT_DIR/resources/vio/granite"
# Ubuntu 24.04 packages of the libraries that the granite programs link (readelf NEEDED, dpkg -S on uvify-pi5-01).
# libopenh264-7: libgranite links OpenH264 (BSD) for vis_msg_encode h264.
APT_PACKAGES=(libopencv-core406t64 libopencv-imgproc406t64 libopencv-imgcodecs406t64 libopencv-calib3d406t64
              libopencv-features2d406t64 libopencv-highgui406t64 libtbb12 libglew2.2 libegl1 libglx0 libopengl0
              libx11-6 libopenh264-7)

usage() { awk 'NR > 1 && !/^#/ { exit } NR > 1 { sub(/^# ?/, ""); print }' "$0"; exit "${1:-0}"; }

deploy_of_unit() { sed -n 's/^Environment=GRANITE_DEPLOY=//p' "$UNIT_DST" 2>/dev/null; }

check_deploy() {
  local dir=$1 ok=0
  for f in bin/granite_vio scripts/pi5_live_kurokesu_vio.sh config/kurokesu_672x600_calib.json \
           config/kurokesu_672x600_handheld.json config/service/kurokesu_672x600_calib.json \
           config/service/kurokesu_672x600_handheld.json; do
    if [[ -e $dir/$f ]]; then
      echo "OK    $dir/$f"
    else
      echo "FAIL  $dir/$f does not exist"; ok=1
    fi
  done
  if [[ -x $dir/bin/granite_vio ]] && ldd "$dir/bin/granite_vio" | grep 'not found' >/dev/null; then
    echo "FAIL  libraries of granite_vio are missing:"
    ldd "$dir/bin/granite_vio" | grep 'not found' | sed 's/^/      /'
    ok=1
  fi
  return $ok
}

check() {
  local ok=1 dir
  if [[ -f $UNIT_DST ]]; then
    echo "OK    $UNIT_DST"
  else
    echo "FAIL  $UNIT_DST does not exist"; ok=0
  fi
  dir=$(deploy_of_unit)
  if [[ -n $dir ]]; then
    check_deploy "$dir" || ok=0
    echo "Deployed: $(cat "$dir/DEPLOYED" 2>/dev/null || echo unknown)"
    if [[ -f $dir/config/service/SOURCE ]]; then
      echo "Service calibration (renew-config.sh):"
      sed 's/^/      /' "$dir/config/service/SOURCE"
    else
      echo "Service calibration: the calibration of the bundle"
    fi
  fi
  if [[ -f $DEFAULT_DST ]]; then
    echo "$DEFAULT_DST:"
    grep -E '^[A-Z_]+=' "$DEFAULT_DST" | sed 's/^/      /'
  else
    echo "WARN  $DEFAULT_DST does not exist: no ODOMETRY to the FC"
  fi
  if systemctl is-enabled --quiet "$SERVICE" 2>/dev/null; then
    echo "OK    $SERVICE is enabled (starts at boot)"
  else
    echo "FAIL  $SERVICE is not enabled"; ok=0
  fi
  if systemctl is-active --quiet mavlink-router.service; then
    echo "OK    mavlink-router.service is active"
  else
    echo "FAIL  mavlink-router.service is not active (setup_fc_uart_router.sh)"; ok=0
  fi
  echo "INFO  $SERVICE is $(systemctl is-active "$SERVICE" 2>/dev/null || true)"
  echo "Service log (since the last boot):"
  journalctl -u "$SERVICE" -b --no-pager -o cat 2>/dev/null | tail -n 8 | sed 's/^/      /' || true
  if [[ -n $dir && -d $dir/runs ]]; then
    echo "Last run: $(ls -1d "$dir"/runs/*/ 2>/dev/null | tail -n 1)"
  fi
  [[ $ok -eq 1 ]] && echo "RESULT OK" || { echo "RESULT FAIL"; return 1; }
}

uninstall() {
  require_root
  systemctl disable --now "$SERVICE" 2>/dev/null && log DISABLE "$SERVICE" || true
  if [[ -f $UNIT_DST ]]; then
    rm -f "$UNIT_DST"
    log REMOVE "$UNIT_DST"
  fi
  systemctl daemon-reload
  [[ -f $DEFAULT_DST ]] && log KEEP "$DEFAULT_DST (remove it by hand when it is not necessary)"
  echo "Done."
}

install_apt() {
  local missing=() p
  for p in "${APT_PACKAGES[@]}"; do
    dpkg -s "$p" >/dev/null 2>&1 || missing+=("$p")
  done
  if [[ ${#missing[@]} -eq 0 ]]; then
    log OK "all ${#APT_PACKAGES[@]} packages are installed"
    return
  fi
  log APT "${missing[*]}"
  apt-get update
  apt-get install -y --no-install-recommends "${missing[@]}"
}

install_bundle() {
  local dir=$1 user=$2 group rel mode tmp desktop
  [[ -f $BUNDLE/SHA256SUMS && -f $BUNDLE/DEPLOYED ]] || die "$BUNDLE is not complete"
  (cd "$BUNDLE" && sha256sum --quiet -c SHA256SUMS) || die "SHA256 of $BUNDLE is wrong"
  log VERIFY "$BUNDLE ($(cat "$BUNDLE/DEPLOYED"))"
  group=$(id -gn "$user")
  install -d -o "$user" -g "$group" "$dir" "$dir/bin" "$dir/lib" "$dir/config" "$dir/scripts" "$dir/runs"
  # Copy to a temporary name, then rename: a running granite_vio keeps its old files.
  while read -r _ rel; do
    mode=0644
    [[ -x $BUNDLE/$rel ]] && mode=0755
    tmp="$dir/$(dirname "$rel")/.$(basename "$rel").new"
    install -m "$mode" -o "$user" -g "$group" "$BUNDLE/$rel" "$tmp"
    mv -f "$tmp" "$dir/$rel"
  done <"$BUNDLE/SHA256SUMS"
  install -m 0644 -o "$user" -g "$group" "$BUNDLE/DEPLOYED" "$dir/DEPLOYED"
  log INSTALL "$dir ($(cat "$dir/DEPLOYED"))"
  desktop="$HOME_DIR/Desktop"
  if [[ -d $desktop ]]; then
    sed "s|/home/uvify/deploy/granite|$dir|g" "$dir/scripts/granite-live-kurokesu.desktop" >"$desktop/granite-live-kurokesu.desktop"
    chown "$user:$group" "$desktop/granite-live-kurokesu.desktop"
    chmod 0755 "$desktop/granite-live-kurokesu.desktop"
    sudo -u "$user" DBUS_SESSION_BUS_ADDRESS="unix:path=/run/user/$(id -u "$user")/bus" \
      gio set "$desktop/granite-live-kurokesu.desktop" metadata::trusted true 2>/dev/null || true
    log ICON "$desktop/granite-live-kurokesu.desktop"
  fi
}

# The service reads config/service/. Copy the bundle files only when the folder does not exist, so that a
# calibration of renew-config.sh stays.
install_service_config() {
  local dir=$1 user=$2 group f
  group=$(id -gn "$user")
  if [[ -d $dir/config/service ]]; then
    log KEEP "$dir/config/service (renew-config.sh applies a new bundle config)"
    return
  fi
  install -d -o "$user" -g "$group" "$dir/config/service"
  for f in kurokesu_672x600_calib.json kurokesu_672x600_handheld.json; do
    [[ -f $dir/config/$f ]] || die "$dir/config/$f does not exist"
    install -m 0644 -o "$user" -g "$group" "$dir/config/$f" "$dir/config/service/$f"
  done
  log WRITE "$dir/config/service (the 672x600 files of the bundle)"
}

RUN_USER=${SUDO_USER:-}
DEPLOY=""
ENABLE=1
USE_DEPLOYED=0
SKIP_APT=0
while [[ $# -gt 0 ]]; do
  case $1 in
    --user) RUN_USER=${2:?}; shift 2 ;;
    --deploy) DEPLOY=${2:?}; shift 2 ;;
    --use-deployed) USE_DEPLOYED=1; shift ;;
    --skip-apt) SKIP_APT=1; shift ;;
    --no-enable) ENABLE=0; shift ;;
    --check) check; exit $? ;;
    --uninstall) uninstall; exit 0 ;;
    -h|--help) usage 0 ;;
    *) echo "unknown argument: $1" >&2; usage 1 ;;
  esac
done

require_root
require_pi5
[[ -f $UNIT_SRC && -f $DEFAULT_SRC ]] || die "$SCRIPT_DIR/resources/vio is not complete"
[[ -n $RUN_USER && $RUN_USER != root ]] || die "give the user of the deployment with --user"
id "$RUN_USER" >/dev/null 2>&1 || die "user $RUN_USER does not exist"
HOME_DIR=$(getent passwd "$RUN_USER" | cut -d: -f6)
DEPLOY=${DEPLOY:-$HOME_DIR/deploy/granite}

echo "== 1. Ubuntu packages"
if [[ $SKIP_APT -eq 1 ]]; then
  log SKIP "apt (--skip-apt)"
else
  install_apt
fi

echo "== 2. granite in $DEPLOY"
if [[ $USE_DEPLOYED -eq 1 ]]; then
  log KEEP "$DEPLOY (--use-deployed)"
else
  install_bundle "$DEPLOY" "$RUN_USER"
fi
install_service_config "$DEPLOY" "$RUN_USER"
check_deploy "$DEPLOY" || die "granite in $DEPLOY is not complete"
log DEPLOYED "$(cat "$DEPLOY/DEPLOYED" 2>/dev/null || echo unknown)"

echo "== 3. $UNIT_DST"
NEW_UNIT=$(mktemp)
sed -e "s|@USER@|$RUN_USER|g" -e "s|@DEPLOY@|$DEPLOY|g" "$UNIT_SRC" >"$NEW_UNIT"
if [[ -f $UNIT_DST ]] && cmp -s "$NEW_UNIT" "$UNIT_DST"; then
  log OK "$UNIT_DST"
else
  install -m 0644 "$NEW_UNIT" "$UNIT_DST"
  log WRITE "$UNIT_DST (user $RUN_USER)"
fi
rm -f "$NEW_UNIT"
systemctl daemon-reload

echo "== 4. $DEFAULT_DST"
if [[ -f $DEFAULT_DST ]]; then
  log KEEP "$DEFAULT_DST"
else
  install -m 0644 "$DEFAULT_SRC" "$DEFAULT_DST"
  log WRITE "$DEFAULT_DST"
fi

echo "== 5. Enable $SERVICE"
if [[ $ENABLE -eq 1 ]]; then
  systemctl enable "$SERVICE" >/dev/null 2>&1
  log ENABLE "$SERVICE (starts at the next boot)"
else
  log SKIP "enable (--no-enable)"
fi

echo
echo "Done. The script did not start the service. To start it now:"
echo "  sudo systemctl start granite-vio"
echo "Stop it before a capture or a calibration:"
echo "  sudo systemctl stop granite-vio"
echo "Check:"
echo "  bash $0 --check"
