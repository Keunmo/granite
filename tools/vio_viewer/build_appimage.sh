#!/bin/bash
# Builds the UVIFY VIO Viewer and packs it as an AppImage: tools/vio_viewer/dist/UVIFY_VIO_Viewer-x86_64.AppImage
#
# Usage: tools/vio_viewer/build_appimage.sh
# Env:   BUILD_DIR (default build_viewer in the repository), JOBS (default 6),
#        APPIMAGETOOL (default appimagetool in PATH),
#        OPENH264_ROOT (default ~/cross/openh264-amd64): the Ubuntu 24.04 packages libopenh264-dev and
#        libopenh264-7 extracted with dpkg-deb -x (or / when they are installed)
# The AppImage has GLFW, libjpeg and OpenH264. GL, X11, glibc and libstdc++ come from the host (glibc 2.38 or
# later: Ubuntu 24.04).
set -eu
HERE=$(cd "$(dirname "$0")" && pwd)
G=$(cd "$HERE/../.." && pwd)
B=${BUILD_DIR:-$G/build_viewer}
APPIMAGETOOL=${APPIMAGETOOL:-appimagetool}
A=$B/AppDir
OUT=$HERE/dist/UVIFY_VIO_Viewer-x86_64.AppImage

OPENH264_ROOT=${OPENH264_ROOT:-$HOME/cross/openh264-amd64}
OPENH264_SO="$OPENH264_ROOT/usr/lib/x86_64-linux-gnu/libopenh264.so.7"
[ -f "$OPENH264_SO" ] || { echo "No $OPENH264_SO (set OPENH264_ROOT)"; exit 1; }

cmake -S "$HERE" -B "$B" -DCMAKE_BUILD_TYPE=Release -DOPENH264_ROOT="$OPENH264_ROOT" > "$B.cmake.log"
cmake --build "$B" -j "${JOBS:-6}"

rm -rf "$A"
mkdir -p "$A/usr/bin" "$A/usr/lib" "$HERE/dist"
cp "$B/uvify-vio-viewer" "$A/usr/bin/"
for lib in libglfw.so.3 libjpeg.so.8; do
  path=$(ldconfig -p | awk -v l="$lib" '$1 == l && /x86-64/ {print $NF; exit}')
  [ -n "$path" ] || { echo "No $lib on this system"; exit 1; }
  cp -L "$path" "$A/usr/lib/$lib"
done
cp -L "$OPENH264_SO" "$A/usr/lib/libopenh264.so.7"
cp "$HERE/uvify-vio-viewer.desktop" "$A/"
cp "$G/scripts/pi4_live_mono_vio.svg" "$A/uvify-vio-viewer.svg"
cat > "$A/AppRun" <<'EOF'
#!/bin/sh
HERE=$(dirname "$(readlink -f "$0")")
exec "$HERE/usr/bin/uvify-vio-viewer" "$@"
EOF
chmod +x "$A/AppRun"
git -C "$G" rev-parse --short HEAD > "$A/VERSION"

ARCH=x86_64 "$APPIMAGETOOL" "$A" "$OUT"
echo "Built $OUT"
