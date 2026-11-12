#!/usr/bin/env bash
# Set up the Raspberry Pi 5 side of the FC UART link and install mavlink-routerd.
#
#   sudo ./setup_fc_uart_router.sh [--no-flow-control] [--router-binary FILE]
#                                  [--keep-conf]
#   ./setup_fc_uart_router.sh --check
#
# The script does these steps:
#
#   1. Enable UART0 (GPIO14/15) in /boot/firmware/config.txt
#      (copy: config.txt.bak-uart0): dtoverlay=uart0-pi5,ctsrts, or
#      dtparam=uart0=on with --no-flow-control.
#   2. Remove a kernel console on ttyAMA0 from cmdline.txt and disable
#      serial-getty@ttyAMA0.
#   3. Write resources/mavlink-router/main.conf to
#      /etc/mavlink-router/main.conf. The old file is copied to
#      main.conf.bak-<time>. --keep-conf keeps the old file.
#   4. Install mavlink-routerd and its systemd service with
#      resources/mavlink-router/install.sh (from device-setup, branch master).
#      The binary is resources/mavlink-router/mavlink-routerd-v4-<arch>, or
#      FILE of --router-binary. The script checks the SHA256 of the
#      resources binary. No internet connection is necessary.
#
# The FC parameters (SERIAL3_*) are not set by this script. Use fc_mav.py.
# Reboot the Pi when the script says so. Then run the script with --check.
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=lib/bootcfg.sh
source "$SCRIPT_DIR/lib/bootcfg.sh"

DEVICE=/dev/ttyAMA0
ROUTER_DIR="$SCRIPT_DIR/resources/mavlink-router"
CONF_SRC="$ROUTER_DIR/main.conf"
CONF_DST=/etc/mavlink-router/main.conf

usage() { awk 'NR > 1 && !/^#/ { exit } NR > 1 { sub(/^# ?/, ""); print }' "$0"; exit "${1:-0}"; }

check() {
  local ok=1
  echo "config.txt:"
  grep -nE '^[[:space:]]*(dtparam=uart0|dtoverlay=uart0)' "$CONFIG_TXT" | sed 's/^/      /' || true
  echo "cmdline.txt: $(cat "$CMDLINE_TXT" 2>/dev/null)"
  if [[ -c $DEVICE ]]; then
    echo "OK    $DEVICE ($(stat -c '%G %a' "$DEVICE"))"
  else
    echo "FAIL  $DEVICE does not exist (reboot after the setup)"; ok=0
  fi
  if grep -qE "console=(ttyAMA0)" /proc/cmdline; then
    echo "FAIL  the running kernel has a console on ttyAMA0"; ok=0
  fi
  if systemctl is-active --quiet serial-getty@ttyAMA0.service; then
    echo "FAIL  serial-getty@ttyAMA0 is active"; ok=0
  fi
  if [[ -x /usr/local/bin/mavlink-routerd ]]; then
    echo "OK    /usr/local/bin/mavlink-routerd"
  else
    echo "FAIL  /usr/local/bin/mavlink-routerd does not exist"; ok=0
  fi
  if [[ -f $CONF_DST ]]; then
    echo "$CONF_DST:"
    sed -n '/^\[UartEndpoint/,/^\[/p' "$CONF_DST" | grep -E '^(Device|Baud|FlowControl)' | sed 's/^/      /'
  else
    echo "FAIL  $CONF_DST does not exist"; ok=0
  fi
  if systemctl is-active --quiet mavlink-router.service; then
    echo "OK    mavlink-router.service is active"
  else
    echo "FAIL  mavlink-router.service is not active"; ok=0
  fi
  echo "Router log (since the last start):"
  journalctl -u mavlink-router.service -b --no-pager -o cat 2>/dev/null |
    grep -E 'Opened|rror|failed' | tail -n 8 | sed 's/^/      /' || true
  [[ $ok -eq 1 ]] && echo "RESULT OK (now check the FC data: fc_mav.py imu)" || { echo "RESULT FAIL"; return 1; }
}

FLOW=1
ROUTER_BIN=""
KEEP_CONF=0
while [[ $# -gt 0 ]]; do
  case $1 in
    --no-flow-control) FLOW=0; shift ;;
    --router-binary) ROUTER_BIN=$(realpath "${2:?}"); shift 2 ;;
    --keep-conf) KEEP_CONF=1; shift ;;
    --check) check; exit $? ;;
    -h|--help) usage 0 ;;
    *) echo "unknown argument: $1" >&2; usage 1 ;;
  esac
done

require_root
require_pi5
[[ -f $ROUTER_DIR/install.sh && -f $ROUTER_DIR/mavlink-router.service && -f $CONF_SRC ]] ||
  die "$ROUTER_DIR is not complete"
if [[ -z $ROUTER_BIN ]]; then
  ROUTER_BIN="$ROUTER_DIR/mavlink-routerd-v4-$(uname -m)"
  [[ -f $ROUTER_BIN && -f $ROUTER_BIN.sha256 ]] || die "$ROUTER_BIN does not exist. Give --router-binary."
  (cd "$ROUTER_DIR" && sha256sum --quiet -c "$(basename "$ROUTER_BIN").sha256") || die "SHA256 of $ROUTER_BIN is wrong"
  log VERIFY "$ROUTER_BIN"
fi
[[ -f $ROUTER_BIN ]] || die "$ROUTER_BIN does not exist"

echo "== 1. UART0 in $CONFIG_TXT"
if [[ $FLOW -eq 1 ]]; then
  UART_LINE="dtoverlay=uart0-pi5,ctsrts"
  [[ -f $BOOT_DIR/overlays/uart0-pi5.dtbo ]] || die "$BOOT_DIR/overlays/uart0-pi5.dtbo does not exist"
else
  UART_LINE="dtparam=uart0=on"
fi
begin_config_edit
comment_lines '^(dtparam=uart0(=.*)?|dtoverlay=uart0(-pi5)?([,].*)?)$' "$UART_LINE"
if has_line_in_last_all "$UART_LINE"; then
  log OK "$UART_LINE"
else
  append_to_all "$UART_LINE"
fi
end_config_edit bak-uart0

echo "== 2. Serial console on ttyAMA0"
# On the Pi 5, serial0 is the debug connector (ttyAMA10) by default. Remove
# console=serial0 only when serial0 is ttyAMA0.
CONSOLE_RE='^console=ttyAMA0(,.*)?$'
if [[ $(readlink -f /dev/serial0 2>/dev/null) == "$DEVICE" ]]; then
  CONSOLE_RE='^console=(ttyAMA0|serial0)(,.*)?$'
fi
remove_cmdline_arg "$CONSOLE_RE"
if systemctl is-enabled --quiet serial-getty@ttyAMA0.service 2>/dev/null ||
   systemctl is-active --quiet serial-getty@ttyAMA0.service; then
  systemctl disable --now serial-getty@ttyAMA0.service
  log DISABLE serial-getty@ttyAMA0
fi
systemctl mask serial-getty@ttyAMA0.service >/dev/null 2>&1 && log MASK serial-getty@ttyAMA0

echo "== 3. $CONF_DST"
install -d /etc/mavlink-router
if [[ $KEEP_CONF -eq 1 && -f $CONF_DST ]]; then
  log KEEP "$CONF_DST"
else
  NEW_CONF=$(mktemp)
  sed -E "s|^Device = .*|Device = $DEVICE|" "$CONF_SRC" >"$NEW_CONF"
  if [[ $FLOW -eq 0 ]]; then
    sed -i -E 's/^FlowControl = .*/FlowControl = false/' "$NEW_CONF"
  fi
  if [[ -f $CONF_DST ]] && cmp -s "$NEW_CONF" "$CONF_DST"; then
    log OK "$CONF_DST"
  else
    [[ -f $CONF_DST ]] && save_backup "$CONF_DST" "$CONF_DST" "bak-$(date +%Y%m%d-%H%M%S)"
    install -m 0644 "$NEW_CONF" "$CONF_DST"
    log WRITE "$CONF_DST"
  fi
  rm -f "$NEW_CONF"
fi

echo "== 4. mavlink-routerd"
# install.sh keeps the existing main.conf, so the file of step 3 stays.
bash "$ROUTER_DIR/install.sh" "$ROUTER_BIN"
# The service runs as a dynamic user in the dialout group. Make sure that
# the group can open the port.
[[ -c $DEVICE ]] && log PORT "$DEVICE $(stat -c '%G %a' "$DEVICE")"

echo
if [[ $BOOT_CHANGED -eq 1 || ! -c $DEVICE ]]; then
  echo "Done. Reboot the Pi. The router opens $DEVICE after the reboot:"
  echo "  sudo reboot"
else
  echo "Done. No reboot is necessary."
fi
echo "Then check the Pi side and the FC data:"
echo "  bash $0 --check"
echo "  python3 $SCRIPT_DIR/fc_mav.py --conn udpin:127.0.0.1:14601 imu"
