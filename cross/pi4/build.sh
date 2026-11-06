#!/bin/bash
# Cross build of Granite for the Raspberry Pi 4 on the desktop (Ubuntu, x86_64).
# Builds Pangolin (static, once) and then libgranite.so, granite_vio, granite_pi_cam_imu_vio and
# granite_mono_vio_dataset.
#
# Usage: cross/pi4/build.sh [-j N]
# Env:   PI_SYSROOT  (default ~/sysroots/pi4-ubuntu2404): copy of the Pi 4 libraries and headers
#        BUILD_ROOT  (default ~/cross/pi4-build): build folders
#
# Needs: aarch64-linux-gnu-gcc-13 / g++-13, cmake, and the desktop GLEW header /usr/include/GL/glew.h
# (libglew-dev, version 2.2.0 as libGLEW.so.2.2 of the Pi). The Pi 4 has no GLEW header, so the build
# uses the header of the desktop.
set -eu
HERE=$(cd "$(dirname "$0")" && pwd)
SRC=$(cd "$HERE/../.." && pwd)
JOBS=$(nproc)
[ "${1:-}" = "-j" ] && JOBS=$2
export PI_SYSROOT=${PI_SYSROOT:-$HOME/sysroots/pi4-ubuntu2404}
BUILD_ROOT=${BUILD_ROOT:-$HOME/cross/pi4-build}
TC=$HERE/pi4-ubuntu2404.toolchain.cmake
CPU=-mcpu=cortex-a72+crc

[ -d "$PI_SYSROOT/usr/include" ] || { echo "No sysroot in $PI_SYSROOT (see cross/pi4/README.md)"; exit 1; }
[ -f /usr/include/GL/glew.h ] || { echo "No /usr/include/GL/glew.h: install libglew-dev"; exit 1; }

GLEW=$BUILD_ROOT/glew_include
mkdir -p "$GLEW/GL"
cp /usr/include/GL/glew.h /usr/include/GL/glxew.h "$GLEW/GL/"

# Pangolin: static library without video, tools and python. The Pi 4 libstdc++ headers do not include
# <cstdint> through other headers, and the sysroot has no GL/glu.h (GLEW_NO_GLU).
P=$BUILD_ROOT/pangolin
if [ ! -f "$P/src/libpangolin.a" ]; then
  mkdir -p "$P"
  cmake -S "$SRC/thirdparty/Pangolin" -B "$P" -DCMAKE_TOOLCHAIN_FILE="$TC" -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_SHARED_LIBS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    -DBUILD_TESTS=OFF -DBUILD_TOOLS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_PANGOLIN_VIDEO=OFF -DBUILD_PANGOLIN_PYTHON=OFF \
    -DBUILD_PANGOLIN_FFMPEG=OFF -DBUILD_PANGOLIN_LIBREALSENSE=OFF -DBUILD_PANGOLIN_LIBREALSENSE2=OFF \
    -DBUILD_PANGOLIN_OPENNI=OFF -DBUILD_PANGOLIN_OPENNI2=OFF -DBUILD_PANGOLIN_LIBUVC=OFF \
    -DBUILD_PANGOLIN_LIBDC1394=OFF -DBUILD_PANGOLIN_V4L=OFF -DBUILD_PANGOLIN_DEPTHSENSE=OFF \
    -DBUILD_PANGOLIN_TELICAM=OFF -DBUILD_PANGOLIN_PLEORA=OFF -DBUILD_PANGOLIN_TOON=OFF -DBUILD_PANGOLIN_ZSTD=OFF \
    -DBUILD_PANGOLIN_LZ4=OFF -DBUILD_PANGOLIN_LIBOPENEXR=OFF -DBUILD_PANGOLIN_LIBTIFF=OFF \
    -DBUILD_PANGOLIN_LIBJPEG=OFF -DBUILD_PANGOLIN_LIBPNG=OFF \
    -DGLEW_INCLUDE_DIR="$GLEW" -DGLEW_LIBRARY="$PI_SYSROOT/usr/lib/aarch64-linux-gnu/libGLEW.so.2.2" \
    -DEIGEN_INCLUDE_DIR="$SRC/thirdparty/granite-headers/thirdparty/eigen" \
    -DCMAKE_CXX_FLAGS="-include cstdint -DGLEW_NO_GLU $CPU -w" -DCMAKE_C_FLAGS="-DGLEW_NO_GLU $CPU" > "$P.cmake.log"
  cmake --build "$P" --target pangolin -j "$JOBS" > "$P.build.log"
fi

G=$BUILD_ROOT/granite
cmake -S "$HERE" -B "$G" -DCMAKE_TOOLCHAIN_FILE="$TC" -DGRANITE_SRC="$SRC" -DPANGOLIN_BUILD="$P" \
  -DGLEW_HEADERS="$GLEW" > "$G.cmake.log"
cmake --build "$G" -j "$JOBS"
echo "Built in $G: granite_vio granite_pi_cam_imu_vio granite_mono_vio_dataset libgranite.so"
