# Cross build for the Raspberry Pi 4 (Ubuntu 24.04, aarch64, Cortex-A72) on the desktop.
# Sysroot: ~/sysroots/pi4-ubuntu2404, a copy of the Pi 4 /usr/include, /usr/lib/aarch64-linux-gnu and
# /usr/lib/gcc/aarch64-linux-gnu, /usr/share/eigen3, /usr/lib/cmake, /usr/share/pkgconfig (rsync from 10.20.10.50; absolute links made relative).
# Compiler: aarch64-linux-gnu-g++ 13 (the same gcc 13.3 as the native compiler of the Pi).
# Executables link with --allow-shlib-undefined: some libraries of the sysroot (libgdal) need symbols of the
# BLAS/LAPACK alternative that the Pi selects at run time.
# Use: cmake -DCMAKE_TOOLCHAIN_FILE=<repo>/cross/pi4/pi4-ubuntu2404.toolchain.cmake ... (cross/pi4/build.sh does this).
# The sysroot path comes from the environment variable PI_SYSROOT (default ~/sysroots/pi4-ubuntu2404).
# -march=native is not available in a cross build: use what -march=native gives on the target: -mcpu=cortex-a72+crc (Pi 4), -mcpu=cortex-a76+crc+crypto (Pi 5).
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
if(DEFINED ENV{PI_SYSROOT})
  set(PI_SYSROOT $ENV{PI_SYSROOT})
else()
  set(PI_SYSROOT $ENV{HOME}/sysroots/pi4-ubuntu2404)
endif()
set(CMAKE_SYSROOT ${PI_SYSROOT})
set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc-13)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++-13)
set(CMAKE_FIND_ROOT_PATH ${PI_SYSROOT})
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
set(CMAKE_LIBRARY_ARCHITECTURE aarch64-linux-gnu)
set(PI_CPU_FLAGS "-mcpu=cortex-a72+crc" CACHE STRING "CPU flags of the target")
set(CMAKE_C_FLAGS_INIT "${PI_CPU_FLAGS}")
set(CMAKE_CXX_FLAGS_INIT "${PI_CPU_FLAGS}")
set(CMAKE_EXE_LINKER_FLAGS_INIT "-Wl,--allow-shlib-undefined -Wl,-rpath-link,${PI_SYSROOT}/usr/lib/aarch64-linux-gnu -Wl,-rpath-link,${PI_SYSROOT}/lib/aarch64-linux-gnu")
set(CMAKE_SHARED_LINKER_FLAGS_INIT "${CMAKE_EXE_LINKER_FLAGS_INIT}")
