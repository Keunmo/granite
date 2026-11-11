# Cross build (desktop) of the Granite library, granite_vio (Pangolin GUI), the live app and the offline mono VIO
# runner for the Raspberry Pi 5. Use cross/pi5/build.sh. Source: this repository (GRANITE_SRC). The flags are those
# of the native Pi build, with -march=native replaced by PI_CPU_FLAGS of the toolchain file.
cmake_minimum_required(VERSION 3.10)
project(granite_pi5 CXX C)
get_filename_component(GRANITE_SRC_DEFAULT ${CMAKE_CURRENT_SOURCE_DIR}/../.. ABSOLUTE)
set(GRANITE_SRC ${GRANITE_SRC_DEFAULT} CACHE PATH "Granite source")
set(G ${GRANITE_SRC})
set(CMAKE_CXX_STANDARD 17)
# Find libgranite.so in lib/ next to bin/ (cross/pi5/deploy.sh: ~/deploy/granite/{bin,lib}) or next to the runner.
set(CMAKE_BUILD_RPATH "\$ORIGIN/../lib;\$ORIGIN")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS "${PI_CPU_FLAGS} -ftemplate-backtrace-limit=0 -w")
set(EIGEN3_INCLUDE_DIR ${G}/thirdparty/granite-headers/thirdparty/eigen)
set(EIGEN_INCLUDE_DIR ${EIGEN3_INCLUDE_DIR})
set(EIGEN_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})
find_package(TBB CONFIG REQUIRED)
find_package(OpenCV REQUIRED core imgproc imgcodecs features2d calib3d highgui)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
set(BUILD_TESTS OFF CACHE BOOL "" FORCE)
macro(add_definitions)
  if(NOT ${ARGV0} STREQUAL "-march=native")
    _add_definitions(${ARGN})
  endif()
endmacro()
add_subdirectory(${G}/thirdparty/opengv opengv EXCLUDE_FROM_ALL)
set_target_properties(opengv PROPERTIES CXX_STANDARD 17 DEBUG_POSTFIX "")
target_compile_options(opengv PRIVATE ${PI_CPU_FLAGS} -w)
include_directories(${G}/thirdparty/granite-headers/thirdparty/Sophus ${G}/thirdparty/granite-headers/thirdparty/cereal/include
  ${G}/thirdparty/granite-headers/include ${G}/thirdparty/CLI11/include ${G}/thirdparty/fast/include ${G}/include
  ${OpenCV_INCLUDE_DIRS})
add_library(granite SHARED
  ${G}/src/utils/vio_config.cpp
  ${G}/src/utils/vis_publisher.cpp
  ${G}/src/optical_flow/optical_flow.cpp
  ${G}/src/vi_estimator/mono_vio.cpp
  ${G}/src/vi_estimator/mono_vio_alignment.cpp
  ${G}/src/vi_estimator/mono_inertial_initializer.cpp
  ${G}/src/vi_estimator/keypoint_vio.cpp
  ${G}/src/vi_estimator/keypoint_vio_mono.cpp
  ${G}/src/vi_estimator/keypoint_vio_linearize.cpp
  ${G}/src/vi_estimator/keypoint_vo.cpp
  ${G}/src/vi_estimator/vio_estimator.cpp
  ${G}/src/vi_estimator/ba_base.cpp
  ${G}/src/vi_estimator/landmark_database.cpp
  ${G}/src/utils/keypoints.cpp)
target_link_libraries(granite PUBLIC TBB::tbb stdc++fs ${OpenCV_LIBS} PRIVATE opengv)
# send_vis_msg with vis_msg_encode h264 uses OpenH264 (BSD; Ubuntu libopenh264-dev). Without it, the build sends
# JPEG images only.
find_path(OPENH264_INCLUDE_DIR wels/codec_api.h)
find_library(OPENH264_LIBRARY NAMES openh264)
if(OPENH264_INCLUDE_DIR AND OPENH264_LIBRARY)
  target_include_directories(granite PRIVATE ${OPENH264_INCLUDE_DIR})
  target_link_libraries(granite PRIVATE ${OPENH264_LIBRARY})
  target_compile_definitions(granite PRIVATE GRANITE_WITH_OPENH264)
  message(STATUS "send_vis_msg h264: ${OPENH264_LIBRARY}")
else()
  message(STATUS "send_vis_msg h264: no OpenH264 (OPENH264_INCLUDE_DIR, OPENH264_LIBRARY)")
endif()
# granite_vio needs the dataset reader and the marginalization saver (no rosbag: no boost in the sysroot).
target_sources(granite PRIVATE ${G}/src/io/dataset_io.cpp ${G}/src/io/marg_data_io.cpp)
target_compile_definitions(granite PRIVATE GRANITE_NO_ROSBAG)
# The offline runner of the evaluation.
set(RUNNER_SRC ${G}/eval/src/mono_vio_dataset.cpp)
add_executable(granite_mono_vio_dataset ${RUNNER_SRC})
target_link_libraries(granite_mono_vio_dataset granite pthread)
# Live app: rp1-cfe (Pi 5) or unicam (Pi 4) camera and an MPU on an I2C adapter (sources with unicam support).
add_executable(granite_pi_cam_imu_vio ${G}/src/pi_camera_imu_vio.cpp ${G}/src/device/pi_camera_imu.cpp
  ${G}/src/device/t265_gui.cpp)
target_link_libraries(granite_pi_cam_imu_vio granite pthread)
# granite_vio with the Pangolin GUI, and the live app with the same GUI (sources with src/vio_app.h).
# Pangolin: static cross build of thirdparty/Pangolin by build.sh (GLEW header of the desktop, GLEW_NO_GLU).
set(PANGOLIN_BUILD "" CACHE PATH "Pangolin cross build folder (build.sh sets it)")
set(GLEW_HEADERS "" CACHE PATH "Folder with GL/glew.h (build.sh sets it)")
if(NOT EXISTS ${PANGOLIN_BUILD}/src/libpangolin.a)
  message(FATAL_ERROR "No Pangolin cross build in PANGOLIN_BUILD=${PANGOLIN_BUILD}. Use cross/pi5/build.sh.")
endif()
set(SR ${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu)
add_library(pangolin_pi5 INTERFACE)
target_include_directories(pangolin_pi5 INTERFACE ${G}/thirdparty/Pangolin/include ${PANGOLIN_BUILD}/src/include
    ${GLEW_HEADERS})
target_compile_definitions(pangolin_pi5 INTERFACE GLEW_NO_GLU)
target_link_libraries(pangolin_pi5 INTERFACE ${PANGOLIN_BUILD}/src/libpangolin.a ${SR}/libOpenGL.so
    ${SR}/libGLX.so ${SR}/libGLEW.so.2.2 ${SR}/libEGL.so ${SR}/libX11.so ${SR}/libXext.so rt pthread dl)
add_executable(granite_vio ${G}/src/vio.cpp ${G}/src/device/pi_camera_imu.cpp)
# <mavlink/common/mavlink.h> of the c_library_v2 submodule.
target_include_directories(granite_vio SYSTEM PRIVATE ${G}/thirdparty)
target_include_directories(granite_pi_cam_imu_vio SYSTEM PRIVATE ${G}/thirdparty)
target_compile_definitions(granite_vio PRIVATE GRANITE_WITH_PI_CAMERA)
target_compile_options(granite_vio PRIVATE -include cstdint)
target_link_libraries(granite_vio granite pangolin_pi5)
target_link_libraries(granite_pi_cam_imu_vio pangolin_pi5)
target_compile_definitions(granite_pi_cam_imu_vio PRIVATE GRANITE_WITH_PI_CAMERA)
target_compile_options(granite_pi_cam_imu_vio PRIVATE -include cstdint)
