cmake_minimum_required(VERSION 3.10)
project(gl_composite_test C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Protobuf REQUIRED)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ../../buoy.proto)

add_library(proto_lib STATIC ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(proto_lib
    PUBLIC
        protobuf::libprotobuf
)
target_include_directories(proto_lib PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED IMPORTED_TARGET
    gstreamer-1.0
    gstreamer-video-1.0
    gstreamer-gl-1.0
)
pkg_check_modules(GST_RTP REQUIRED IMPORTED_TARGET
    gstreamer-rtp-1.0
)
pkg_check_modules(GST_APP REQUIRED IMPORTED_TARGET
    gstreamer-app-1.0
)

find_package(OpenCV REQUIRED COMPONENTS core imgproc objdetect highgui)
find_package(yaml-cpp REQUIRED)

add_executable(gl_composite_test prototest.cpp)

target_link_libraries(gl_composite_test
    PRIVATE
        proto_lib
        PkgConfig::GST
        m
)

add_executable(base_station main.cpp Config.cpp BuoyNode.cpp VideoComposite.cpp MathHelpers.cpp)

target_include_directories(base_station PRIVATE
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(base_station PRIVATE
    proto_lib
    yaml-cpp::yaml-cpp
    PkgConfig::GST
    PkgConfig::GST_RTP
    PkgConfig::GST_APP
    ${OpenCV_LIBS}
)
