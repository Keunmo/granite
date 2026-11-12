#!/usr/bin/env bash
# Make the calibration and the config of granite-vio.service from a new
# camera-IMU calibration of xeye.
#
#   ./renew-config.sh CAPTURE_YAML [--deploy DIR] [--restart]
#
# CAPTURE_YAML is the capture.yaml of xeye with the calibration (a capture
# folder also works). Its camera node must be kurokesu_ar0234_pi at 1920x1200
# with the crop [288, 0, 1344, 1200].
#
# The script does these steps:
#
#   1. Convert the calibration with resources/vio/capture_to_granite.py
#      --downscale 2. The 2x2 binning mode of the sensor gives 960x600, and
#      the halved crop gives 672x600 images.
#   2. Write DIR/config/service/kurokesu_672x600_calib.json: the new
#      calibration (intrinsics, camera-IMU transformation, time offset, IMU
#      biases).
#   3. Write DIR/config/service/kurokesu_672x600_handheld.json: the config of
#      the granite bundle (DIR/config/kurokesu_672x600_handheld.json) with the
#      pixel thresholds of the rest detection scaled by the focal length of
#      the new calibration over the focal length of the bundle calibration.
#   4. Keep the old files in DIR/config/service/backup/<date_time>/.
#
# granite-vio.service reads DIR/config/service/. setup_vio_service.sh does not
# replace these files when it installs a new granite bundle. Run this script
# again after a new bundle to apply the new bundle config.
#
# --restart restarts granite-vio.service (sudo). Without it, the new files
# apply at the next start of the service (for example the next boot).
# DIR default: the deployment folder of granite-vio.service, else
# ~/deploy/granite. Run the script as the user of the deployment, not with
# sudo.
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
# shellcheck source=lib/bootcfg.sh
source "$SCRIPT_DIR/lib/bootcfg.sh"

CONVERTER="$SCRIPT_DIR/resources/vio/capture_to_granite.py"
UNIT=/etc/systemd/system/granite-vio.service
CALIB_NAME=kurokesu_672x600_calib.json
CONFIG_NAME=kurokesu_672x600_handheld.json

usage() { awk 'NR > 1 && !/^#/ { exit } NR > 1 { sub(/^# ?/, ""); print }' "$0"; exit "${1:-0}"; }

CAPTURE=""
DEPLOY=""
RESTART=0
while [[ $# -gt 0 ]]; do
  case $1 in
    --deploy) DEPLOY=${2:?}; shift 2 ;;
    --restart) RESTART=1; shift ;;
    -h|--help) usage 0 ;;
    -*) echo "unknown argument: $1" >&2; usage 1 ;;
    *) [[ -z $CAPTURE ]] || usage 1; CAPTURE=$1; shift ;;
  esac
done

[[ ${EUID} -ne 0 ]] || die "run without sudo: the files belong to the user of the deployment"
[[ -n $CAPTURE ]] || usage 1
if [[ -d $CAPTURE ]]; then CAPTURE=$CAPTURE/capture.yaml; fi
[[ -f $CAPTURE ]] || die "$CAPTURE does not exist"
if [[ -z $DEPLOY ]]; then
  DEPLOY=$(sed -n 's/^Environment=GRANITE_DEPLOY=//p' "$UNIT" 2>/dev/null || true)
  DEPLOY=${DEPLOY:-$HOME/deploy/granite}
fi
BASE_CALIB=$DEPLOY/config/$CALIB_NAME
BASE_CONFIG=$DEPLOY/config/$CONFIG_NAME
OUT=$DEPLOY/config/service
for f in "$CONVERTER" "$BASE_CALIB" "$BASE_CONFIG"; do
  [[ -f $f ]] || die "$f does not exist (install the granite bundle with setup_vio_service.sh first)"
done
python3 -c 'import numpy, scipy, yaml' 2>/dev/null ||
  die "python3 needs numpy, scipy and yaml (apt packages python3-numpy python3-scipy python3-yaml)"

echo "== 1. Convert $CAPTURE"
python3 -I - "$CAPTURE" <<'EOF' || die "$CAPTURE is not a 1920x1200 kurokesu_ar0234_pi capture with the crop [288, 0, 1344, 1200]"
import sys, yaml
capture = yaml.safe_load(open(sys.argv[1]))
cams = [v for v in capture.values() if isinstance(v, dict) and v.get("model") == "kurokesu_ar0234_pi"]
if len(cams) != 1:
    sys.exit(f"  found {len(cams)} kurokesu_ar0234_pi nodes, expected 1")
prop = cams[0].get("property", {})
got = (prop.get("width"), prop.get("height"), list(prop.get("crop") or []))
if got != (1920, 1200, [288, 0, 1344, 1200]):
    sys.exit(f"  width, height, crop: {got}")
if "calib" not in capture:
    sys.exit("  the capture has no calib section")
EOF
TMP=$(mktemp -d)
trap 'rm -rf "$TMP"' EXIT
python3 -I "$CONVERTER" "$CAPTURE" --downscale 2 --target live --calib-out "$TMP/calib.json" >"$TMP/convert.log" 2>&1 ||
  { cat "$TMP/convert.log" >&2; die "capture_to_granite.py failed"; }
sed 's/^/      /' "$TMP/convert.log" | grep -E 'binning|cam_time_offset' || true

echo "== 2. Merge and check"
python3 -I - "$TMP/calib.json" "$BASE_CALIB" "$BASE_CONFIG" "$TMP/config.json" "$OUT/$CALIB_NAME" "$CONVERTER" <<'EOF'
import collections, importlib.util, json, os, sys
calib_path, base_calib_path, base_path, out_path, old_calib_path, converter_path = sys.argv[1:7]
spec = importlib.util.spec_from_file_location("capture_to_granite", converter_path)
converter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(converter)
load = lambda p: json.load(open(p), object_pairs_hook=collections.OrderedDict)
calib = load(calib_path)["value0"]
if calib["resolution"] != [[672, 600]]:
    sys.exit(f"Error: the new calibration is {calib['resolution']}, expected [[672, 600]]")
# Image scale [px/rad] at the image center, the same measure for both calibrations
ratio = converter.base_pixels_per_radian(calib_path) / converter.base_pixels_per_radian(base_calib_path)
config = load(base_path)
keys = ["config.vio_mono_init_max_still_disparity", "config.vio_mono_init_max_rest_disparity"]
generated = {}
for k in keys:
    generated[k] = round(config["value0"][k] * ratio, 3)
    config["value0"][k] = generated[k]
print(f"  image scale ratio (new / bundle calibration): {ratio:.4f}")
with open(out_path, "w") as f:
    json.dump(config, f, indent=4)
    f.write("\n")
new = calib["intrinsics"][0]["intrinsics"]
old = load(old_calib_path)["value0"] if os.path.exists(old_calib_path) else None
print(f"  {'':24s} {'old':>12s} {'new':>12s}")
for k in ["fx", "fy", "cx", "cy"]:
    o = f"{old['intrinsics'][0]['intrinsics'][k]:12.3f}" if old else f"{'-':>12s}"
    print(f"  {k:24s} {o} {new[k]:12.3f}")
o = f"{old['cam_time_offset_ns'] * 1e-6:12.3f}" if old else f"{'-':>12s}"
print(f"  {'cam_time_offset [ms]':24s} {o} {calib['cam_time_offset_ns'] * 1e-6:12.3f}")
for k in keys:
    print(f"  {k.split('.')[-1]:24s} {'':12s} {generated[k]:12.3f}")
EOF

echo "== 3. Write $OUT"
mkdir -p "$OUT"
if [[ -f $OUT/$CALIB_NAME || -f $OUT/$CONFIG_NAME ]]; then
  BACKUP=$OUT/backup/$(date +%Y%m%d_%H%M%S)
  mkdir -p "$BACKUP"
  for f in "$CALIB_NAME" "$CONFIG_NAME" SOURCE; do
    if [[ -f $OUT/$f ]]; then cp -p "$OUT/$f" "$BACKUP/"; fi
  done
  log BACKUP "$BACKUP"
fi
install -m 0644 "$TMP/calib.json" "$OUT/$CALIB_NAME"
install -m 0644 "$TMP/config.json" "$OUT/$CONFIG_NAME"
{
  echo "capture: $(realpath "$CAPTURE")"
  echo "capture sha256: $(sha256sum "$CAPTURE" | cut -d' ' -f1)"
  echo "bundle: $(cat "$DEPLOY/DEPLOYED" 2>/dev/null || echo unknown)"
  echo "date: $(date -Is)"
} >"$OUT/SOURCE"
log WRITE "$OUT/$CALIB_NAME"
log WRITE "$OUT/$CONFIG_NAME"

if [[ $RESTART -eq 1 ]]; then
  echo "== 4. Restart granite-vio.service"
  sudo systemctl restart granite-vio
  log RESTART "granite-vio.service"
else
  echo
  echo "Done. The service uses the new files at its next start. To restart it now:"
  echo "  sudo systemctl restart granite-vio"
fi
