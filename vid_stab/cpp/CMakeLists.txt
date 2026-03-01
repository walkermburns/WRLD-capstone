cmake_minimum_required(VERSION 3.16)
project(gst_warp_v1 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-base-1.0 gstreamer-video-1.0)
pkg_check_modules(GLIB REQUIRED glib-2.0 gobject-2.0)

add_executable(gst_warp_v1
  src/main.cpp
  src/serial_imu.cpp
)

target_include_directories(gst_warp_v1 PRIVATE
  ${GST_INCLUDE_DIRS}
  ${GLIB_INCLUDE_DIRS}
  /usr/include/eigen3
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_directories(gst_warp_v1 PRIVATE
  ${GST_LIBRARY_DIRS}
  ${GLIB_LIBRARY_DIRS}
)

target_compile_options(gst_warp_v1 PRIVATE
  ${GST_CFLAGS_OTHER}
  ${GLIB_CFLAGS_OTHER}
)

target_link_libraries(gst_warp_v1
  ${GST_LIBRARIES}
  ${GLIB_LIBRARIES}
)