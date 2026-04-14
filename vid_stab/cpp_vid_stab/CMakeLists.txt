cmake_minimum_required(VERSION 3.16)
project(gst_vid_stab_native_appsink CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 gstreamer-app-1.0)

add_executable(gst_vid_stab main.cpp)

target_include_directories(gst_vid_stab PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${GSTREAMER_INCLUDE_DIRS}
)

target_link_directories(gst_vid_stab PRIVATE
    ${GSTREAMER_LIBRARY_DIRS}
)

target_link_libraries(gst_vid_stab PRIVATE
    ${OpenCV_LIBS}
    Eigen3::Eigen
    ${GSTREAMER_LIBRARIES}
)

target_compile_options(gst_vid_stab PRIVATE
    ${GSTREAMER_CFLAGS_OTHER}
)