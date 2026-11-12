#!/usr/bin/env bash
# Install mavlink-routerd as a systemd service on a Jetson.
#
#   sudo bash install.sh [BINARY]
#
# BINARY defaults to the NAS build for this architecture; it is downloaded and
# verified against its .sha256. An existing /etc/mavlink-router/main.conf is
# kept; pass FORCE_CONFIG=1 to replace it with main.conf from this directory.
set -euo pipefail

if [[ ${EUID} -ne 0 ]]; then
  echo "run with sudo" >&2
  exit 1
fi

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
VERSION=v4
BIN_NAME="mavlink-routerd-${VERSION}-$(uname -m)"
BASE_URL='https://uvifya16f.synology.me:49152/upload/devel/device-setup/orin-nx/mavlink-router'

WORK_DIR=$(mktemp -d)
trap 'rm -rf "${WORK_DIR}"' EXIT

if [[ $# -ge 1 ]]; then
  BINARY=$(realpath "$1")
else
  curl -fsSL -o "${WORK_DIR}/${BIN_NAME}" "${BASE_URL}/${BIN_NAME}"
  curl -fsSL -o "${WORK_DIR}/${BIN_NAME}.sha256" "${BASE_URL}/${BIN_NAME}.sha256"
  (cd "${WORK_DIR}" && sha256sum -c "${BIN_NAME}.sha256")
  BINARY="${WORK_DIR}/${BIN_NAME}"
fi

install -m 0755 "${BINARY}" /usr/local/bin/mavlink-routerd
install -d /etc/mavlink-router
if [[ ! -f /etc/mavlink-router/main.conf || ${FORCE_CONFIG:-0} == 1 ]]; then
  install -m 0644 "${SCRIPT_DIR}/main.conf" /etc/mavlink-router/main.conf
else
  echo "keeping existing /etc/mavlink-router/main.conf"
fi
install -m 0644 "${SCRIPT_DIR}/mavlink-router.service" \
  /etc/systemd/system/mavlink-router.service

systemctl daemon-reload
systemctl enable --now mavlink-router.service
systemctl --no-pager --lines=5 status mavlink-router.service
