# UVIFY VIO Viewer: a standalone viewer of the visualization messages (send_vis_msg) of the Granite VIO.
# The GUI is ImGui and ImPlot (thirdparty/, the same copies as openvio) on GLFW and OpenGL 3. It does not link
# libgranite.
#
#   cmake -S tools/vio_viewer -B build_viewer -DCMAKE_BUILD_TYPE=Release && cmake --build build_viewer -j 6
cmake_minimum_required(VERSION 3.10)
project(uvify_vio_viewer CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(GRANITE_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)
set(THIRDPARTY ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty)

find_package(glfw3 3.3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(JPEG REQUIRED)
# OpenH264 (BSD) for the H.264 images: the Ubuntu 24.04 packages libopenh264-dev and libopenh264-7 (2.4.1). They
# can be extracted with dpkg-deb -x into OPENH264_ROOT; the AppImage has libopenh264.so.7.
set(OPENH264_ROOT "" CACHE PATH "Folder with usr/include/wels and usr/lib/x86_64-linux-gnu/libopenh264.so")
find_path(OPENH264_INCLUDE_DIR wels/codec_api.h HINTS ${OPENH264_ROOT}/usr/include REQUIRED)
find_library(OPENH264_LIBRARY NAMES openh264 HINTS ${OPENH264_ROOT}/usr/lib/x86_64-linux-gnu REQUIRED)

add_library(imgui STATIC
  ${THIRDPARTY}/imgui/imgui.cpp
  ${THIRDPARTY}/imgui/imgui_draw.cpp
  ${THIRDPARTY}/imgui/imgui_tables.cpp
  ${THIRDPARTY}/imgui/imgui_widgets.cpp
  ${THIRDPARTY}/imgui/backends/imgui_impl_glfw.cpp
  ${THIRDPARTY}/imgui/backends/imgui_impl_opengl3.cpp
  ${THIRDPARTY}/implot/implot.cpp
  ${THIRDPARTY}/implot/implot_items.cpp)
target_include_directories(imgui PUBLIC ${THIRDPARTY}/imgui ${THIRDPARTY}/implot)
target_link_libraries(imgui PUBLIC glfw OpenGL::GL ${CMAKE_DL_LIBS})
# ImPlot draws many vertices; 32-bit indices remove the 64k limit of one draw list.
target_compile_definitions(imgui PUBLIC "ImDrawIdx=unsigned int")

add_executable(uvify-vio-viewer vio_viewer.cpp)
target_include_directories(uvify-vio-viewer PRIVATE ${JPEG_INCLUDE_DIRS} ${OPENH264_INCLUDE_DIR}
                                                      ${GRANITE_ROOT}/thirdparty/granite-headers/thirdparty/eigen)
target_link_libraries(uvify-vio-viewer PRIVATE imgui ${JPEG_LIBRARIES} ${OPENH264_LIBRARY} pthread)
# libstdc++ and libgcc in the binary: the AppImage then runs on systems with an older libstdc++.
target_link_options(uvify-vio-viewer PRIVATE -static-libstdc++ -static-libgcc)
# The AppImage keeps GLFW, libjpeg and OpenH264 in usr/lib next to usr/bin.
set_target_properties(uvify-vio-viewer PROPERTIES BUILD_RPATH "$ORIGIN/../lib")
install(TARGETS uvify-vio-viewer RUNTIME DESTINATION bin)
