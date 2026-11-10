cmake_minimum_required(VERSION 3.10)

include("thirdparty/granite-headers/cmake_modules/PreProjectWorkarounds.cmake")

project(granite)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules/" ${CMAKE_MODULE_PATH})

if(NOT EIGEN_ROOT)
  set(EIGEN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/granite-headers/thirdparty/eigen")
endif()

# Configure CCache if available
if (NOT CMAKE_C_COMPILER_LAUNCHER AND NOT CMAKE_CXX_COMPILER_LAUNCHER)
  find_program(CCACHE_PROGRAM ccache)
  if(CCACHE_PROGRAM)
    message(STATUS "Found ccache: ${CCACHE_PROGRAM}")
    set(CMAKE_C_COMPILER_LAUNCHER   ${CCACHE_PROGRAM})
    set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
  else()
    message(STATUS "Dind't find ccache")
  endif()
else()
  message(STATUS "Compiler launcher already set. Not configuring ccache.")
  message(STATUS "CMAKE_C_COMPILER_LAUNCHER: ${CMAKE_C_COMPILER_LAUNCHER}")
  message(STATUS "CMAKE_CXX_COMPILER_LAUNCHER: ${CMAKE_CXX_COMPILER_LAUNCHER}")
endif()

if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE Release)
endif()

if(NOT CXX_MARCH)
  set(CXX_MARCH native)
endif()


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)


# Flags used for CHECK_CXX_SOURCE_COMPILES
set(CMAKE_REQUIRED_FLAGS "-Wno-error")


# save flags passed by user
set(GRANITE_PASSED_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

set(CMAKE_CXX_FLAGS_DEBUG  "-O0 -g -DEIGEN_INITIALIZE_MATRICES_BY_NAN")          # cmake default: "-g"
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O3 -g -DEIGEN_INITIALIZE_MATRICES_BY_NAN")  # cmake default: "-O2 -g -DNDEBUG"
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")                                      # cmake default: "-O3 -DNDEBUG"
set(CMAKE_CXX_FLAGS_CIDEBUG  "-O0 -DEIGEN_INITIALIZE_MATRICES_BY_NAN")          # CI version with no debug symbols
set(CMAKE_CXX_FLAGS_CIRELWITHDEBINFO "-O3 -DEIGEN_INITIALIZE_MATRICES_BY_NAN")  # CI version with no debug symbols

# base set of compile flags
set(GRANITE_CXX_FLAGS "-Wall -Wextra -Werror -Wno-error=unused-parameter -ftemplate-backtrace-limit=0")

# clang-specific compile flags
if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang" OR CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
  set(GRANITE_CXX_FLAGS "${GRANITE_CXX_FLAGS} -Wno-exceptions -fcolor-diagnostics -frelaxed-template-template-args -Wno-error=deprecated-declarations")

  if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 10)
      # These are disabled to avoid lot's of warnings in Eigen code with clang 10
      set(GRANITE_CXX_FLAGS "${GRANITE_CXX_FLAGS} -Wno-misleading-indentation -Wno-error=deprecated-copy")
  endif()

  #   - Added TBB_USE_GLIBCXX_VERSION macro to specify the version of GNU
  #     libstdc++ when it cannot be properly recognized, e.g. when used
  #     with Clang on Linux* OS. Adopted from https://github.com/wjakob/tbb
  if(NOT TBB_USE_GLIBCXX_VERSION AND UNIX AND NOT APPLE)
    if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
      string(REPLACE "." "0" TBB_USE_GLIBCXX_VERSION ${CMAKE_CXX_COMPILER_VERSION})
    endif()
    add_definitions(-DTBB_USE_GLIBCXX_VERSION=${TBB_USE_GLIBCXX_VERSION})
  endif()
else()
  set(GRANITE_CXX_FLAGS "${GRANITE_CXX_FLAGS} -Wno-error=maybe-uninitialized -Wno-error=implicit-fallthrough")

  if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 9)
      # These are disabled to avoid lot's of warnings in Eigen code with gcc-9
      set(GRANITE_CXX_FLAGS "${GRANITE_CXX_FLAGS} -Wno-error=deprecated-copy")
  endif()

  if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER_EQUAL 13)
      # gcc-13 reports false array bounds in Eigen packet loads and TBB nodes,
      # parentheses warnings in the Pangolin picojson header and missing
      # template keywords in the granite-headers tests
      set(GRANITE_CXX_FLAGS "${GRANITE_CXX_FLAGS} -Wno-error=array-bounds -Wno-error=parentheses -Wno-error=missing-template-keyword")
  endif()
endif()


# Set platform / compiler specific compile flags and checks
if(APPLE)
  # Need to investigate how to reliably detect and use OpenMP on macOS...
  set(USE_OPENMP_DEFAULT OFF)

  # Among others, setting CMAKE_FIND_FRAMEWORK to LAST fixed issues
  # with installed Mono that contains old headers (libpng, ...).
  # See: https://github.com/openMVG/openMVG/issues/1349#issuecomment-401492811
  set(CMAKE_FIND_FRAMEWORK LAST)

  if(CMAKE_SYSTEM_VERSION VERSION_LESS 19.0.0)
      # use brewed llvm's libc++
      include_directories("/usr/local/opt/llvm/include/c++/v1")
      link_directories("/usr/local/opt/llvm/lib")
      add_compile_options("-nostdinc++")
      #set(STD_CXX_FS c++fs)

      # Workaround for cmake not to filter the manually added standard include path
      # See: https://gitlab.kitware.com/cmake/cmake/issues/19227#note_669894
      list(REMOVE_ITEM CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES "/usr/local/opt/llvm/include/c++/v1")
  endif()

  if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    message(STATUS "Detected macOS with non-Apple clang")

  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
    message(STATUS "Detected macOS with Apple clang")
    # Apple clang on macOS < 10.14 Mojave is too old
    if(CMAKE_SYSTEM_VERSION VERSION_LESS 18.0.0)
      message(WARNING "Detected Darwin version ${CMAKE_SYSTEM_VERSION}, which is earlier than macos 10.14 Mojave. Apple clang is too old and not supported. Use clang from homebrew.")
    endif()

  else()
    message(WARNING "Detected macOS with unsupported compiler ${CMAKE_CXX_COMPILER_ID}")
  endif()

elseif(UNIX)
  set(USE_OPENMP_DEFAULT ON)

  # assume libstdc++
  set(STD_CXX_FS stdc++fs)

  if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    set(LINUX_CLANG 1)
    message(STATUS "Detected Linux with clang.")
    message(WARNING "Clang on Linux is currently not fully supported. You'll likely need to get a recent version of TBB.")

  elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    message(STATUS "Detected Linux with gcc.")

  else()
    message(WARNING "Detected Linux with unsupported compiler ${CMAKE_CXX_COMPILER_ID}")
  endif()

else()
  message(WARNING "Only Linux and macOS are currently supported")
endif()


# OpenMP option and compile flags
option(USE_OPENMP "Use OpenMP (e.g. for parallel computation in Eigen)" ${USE_OPENMP_DEFAULT})
if(USE_OPENMP)
  message(STATUS "OpenMP Enabled")
  set(GRANITE_CXX_FLAGS "${GRANITE_CXX_FLAGS} -fopenmp")
else()
  message(STATUS "OpenMP Disabled")
endif()


# setup combined compiler flags
set(CMAKE_CXX_FLAGS "${GRANITE_CXX_FLAGS} -march=${CXX_MARCH} ${GRANITE_PASSED_CXX_FLAGS}")


set(EIGEN_INCLUDE_DIR_HINTS ${EIGEN_ROOT})
find_package(Eigen3 3.3.7 EXACT REQUIRED MODULE)
include_directories(${EIGEN3_INCLUDE_DIR})
message(STATUS "Found Eigen headers in: ${EIGEN3_INCLUDE_DIR}")
if(NOT EIGEN3_INCLUDE_DIR MATCHES "^${EIGEN_ROOT}")
  message(WARNING "Found Eigen headers are outside of specified EIGEN_ROOT '${EIGEN_ROOT}'")
endif()

find_package(TBB CONFIG QUIET)
if(TBB_FOUND)
  set(TBB_LIBRARIES TBB::tbb)
  get_target_property(TBB_INCLUDE_DIR TBB::tbb INTERFACE_INCLUDE_DIRECTORIES)
else()
  find_package(TBB REQUIRED MODULE)
endif()
include_directories(${TBB_INCLUDE_DIR})
message(STATUS "TBB_INTERFACE_VERSION: ${TBB_INTERFACE_VERSION}")
if (DEFINED TBB_INTERFACE_VERSION AND TBB_INTERFACE_VERSION LESS 11004)
  # enable global_control header for earlier TBB versions (Ubuntu 16.04, 18.04)
  add_definitions(-DTBB_PREVIEW_GLOBAL_CONTROL)
endif()

find_package(OpenCV REQUIRED core imgproc calib3d highgui)
include_directories(${OpenCV_INCLUDE_DIR})
message(STATUS "Found OpenCV headers in: ${OpenCV_INCLUDE_DIR}")
message(STATUS "Found OpenCV_LIBS: ${OpenCV_LIBS}")


option(GRANITE_BUILD_GUI "Build Pangolin GUI applications" ON)
option(GRANITE_BUILD_REALSENSE "Build T265 live VIO examples" OFF)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  set(GRANITE_BUILD_PI_CAMERA_DEFAULT ON)
else()
  set(GRANITE_BUILD_PI_CAMERA_DEFAULT OFF)
endif()
option(GRANITE_BUILD_PI_CAMERA "Build the Raspberry Pi 5 B0429 camera and MPU-9250 live VIO example"
  ${GRANITE_BUILD_PI_CAMERA_DEFAULT})
add_subdirectory(thirdparty)


include_directories(thirdparty/granite-headers/thirdparty/Sophus)
include_directories(thirdparty/granite-headers/thirdparty/cereal/include)
include_directories(thirdparty/granite-headers/include)
include_directories(thirdparty/CLI11/include)
include_directories(thirdparty/fast/include)

include_directories(include)


add_library(granite SHARED
  src/io/dataset_io.cpp
  src/io/marg_data_io.cpp
  src/calibration/aprilgrid.cpp
  src/calibration/calibraiton_helper.cpp
  src/calibration/vignette.cpp
  src/utils/vio_config.cpp
  src/optical_flow/optical_flow.cpp
  src/vi_estimator/mono_vio.cpp
  src/vi_estimator/mono_vio_alignment.cpp
  src/vi_estimator/mono_inertial_initializer.cpp
  src/vi_estimator/keypoint_vio.cpp
  src/vi_estimator/keypoint_vio_mono.cpp
  src/vi_estimator/keypoint_vio_linearize.cpp
  src/vi_estimator/keypoint_vo.cpp
  src/vi_estimator/vio_estimator.cpp
  src/vi_estimator/ba_base.cpp
  src/vi_estimator/nfr_mapper.cpp
  src/vi_estimator/landmark_database.cpp
  src/utils/keypoints.cpp)


target_link_libraries(granite PUBLIC ${TBB_LIBRARIES} ${STD_CXX_FS} ${OpenCV_LIBS} PRIVATE rosbag apriltag opengv)


if(GRANITE_BUILD_GUI)
add_executable(granite_calibrate src/calibrate.cpp src/calibration/cam_calib.cpp)
target_link_libraries(granite_calibrate granite pangolin)

add_executable(granite_calibrate_imu src/calibrate_imu.cpp src/calibration/cam_imu_calib.cpp)
target_link_libraries(granite_calibrate_imu granite pangolin)


add_executable(granite_vio_sim src/vio_sim.cpp)
target_link_libraries(granite_vio_sim granite pangolin)

add_executable(granite_mapper_sim src/mapper_sim.cpp)
target_link_libraries(granite_mapper_sim granite pangolin)

add_executable(granite_mapper_sim_naive src/mapper_sim_naive.cpp)
target_link_libraries(granite_mapper_sim_naive granite pangolin)

add_executable(granite_mapper src/mapper.cpp)
target_link_libraries(granite_mapper granite pangolin)


add_executable(granite_opt_flow src/opt_flow.cpp)
target_link_libraries(granite_opt_flow granite pangolin)

add_executable(granite_vio src/vio.cpp)
target_link_libraries(granite_vio granite pangolin)
if(GRANITE_BUILD_PI_CAMERA)
  # granite_vio --live: the Raspberry Pi camera and IMU.
  target_sources(granite_vio PRIVATE src/device/pi_camera_imu.cpp)
  target_compile_definitions(granite_vio PRIVATE GRANITE_WITH_PI_CAMERA)
  # <mavlink/common/mavlink.h> of the c_library_v2 submodule.
  target_include_directories(granite_vio SYSTEM PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty)
endif()

add_executable(granite_time_alignment src/time_alignment.cpp)
target_link_libraries(granite_time_alignment granite pangolin)

add_executable(granite_kitti_eval src/kitti_eval.cpp)
target_link_libraries(granite_kitti_eval)



install(TARGETS granite_calibrate granite_calibrate_imu granite_vio_sim granite_mapper_sim granite_mapper_sim_naive granite_mapper granite_opt_flow granite_vio granite_kitti_eval granite_time_alignment granite
  EXPORT GraniteTargets
  RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
  LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

else()
  install(TARGETS granite LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
endif()

if(GRANITE_BUILD_REALSENSE)
  find_package(realsense2 REQUIRED)
  add_library(granite_t265 src/device/rs_t265.cpp)
  target_link_libraries(granite_t265 PUBLIC granite realsense2::realsense2)
  add_library(granite_t265_runner STATIC src/rs_t265_vio.cpp src/device/t265_gui.cpp)
  target_link_libraries(granite_t265_runner PUBLIC granite_t265)
  foreach(_mode vio mono_vio vo mono_vo)
    add_executable(granite_rs_t265_${_mode} src/rs_t265_main.cpp)
    target_link_libraries(granite_rs_t265_${_mode} granite_t265_runner)
    if(_mode MATCHES "^mono")
      target_compile_definitions(granite_rs_t265_${_mode} PRIVATE GRANITE_T265_MONO_DEFAULT=1)
    endif()
    if(_mode MATCHES "vo$")
      target_compile_definitions(granite_rs_t265_${_mode} PRIVATE GRANITE_T265_VO_DEFAULT=1)
    endif()
  endforeach()
  install(TARGETS granite_t265 granite_rs_t265_vio granite_rs_t265_mono_vio granite_rs_t265_vo granite_rs_t265_mono_vo
    RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin
    LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
    ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)
endif()

if(GRANITE_BUILD_PI_CAMERA AND GRANITE_BUILD_GUI)
  # VIO uses the Pangolin GUI of granite_vio (src/vio_app.h). The capture
  # check without VIO uses the device-independent T265 window.
  add_executable(granite_pi_cam_imu_vio src/pi_camera_imu_vio.cpp
    src/device/pi_camera_imu.cpp src/device/t265_gui.cpp)
  target_link_libraries(granite_pi_cam_imu_vio granite pangolin)
  target_compile_definitions(granite_pi_cam_imu_vio PRIVATE GRANITE_WITH_PI_CAMERA)
  target_include_directories(granite_pi_cam_imu_vio SYSTEM PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty)
  install(TARGETS granite_pi_cam_imu_vio RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
endif()

file(GLOB SCRIPTS_TO_INSTALL "${CMAKE_CURRENT_SOURCE_DIR}/scripts/granite_*.py")
install(PROGRAMS ${SCRIPTS_TO_INSTALL} DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

file(GLOB CONFIG_FILES "${CMAKE_CURRENT_SOURCE_DIR}/data/*.json")
install(FILES ${CONFIG_FILES}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/etc/granite)


# Replace install() to do-nothing macro.
macro(install)
endmacro()
# Include subproject (or any other CMake code) with "disabled" install().
enable_testing()
add_subdirectory(thirdparty/granite-headers/test)
add_subdirectory(test)
# Restore original install() behavior.
macro(install)
  _install(${ARGN})
endmacro()
