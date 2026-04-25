cmake_minimum_required(VERSION 3.10)

project(gl_composite_test C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- PROTOBUF SECTION ---
find_package(Protobuf REQUIRED)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ../../buoy.proto)

add_library(proto_lib STATIC ${PROTO_SRCS} ${PROTO_HDRS})

target_link_libraries(proto_lib
    PUBLIC
        protobuf::libprotobuf
)

target_include_directories(proto_lib
    PUBLIC
        ${CMAKE_CURRENT_BINARY_DIR}
)

# --- GSTREAMER SECTION ---
find_package(PkgConfig REQUIRED)

pkg_check_modules(GST REQUIRED IMPORTED_TARGET
    gstreamer-1.0
    gstreamer-video-1.0
    gstreamer-gl-1.0
)

pkg_check_modules(GST_RTP REQUIRED IMPORTED_TARGET
    gstreamer-rtp-1.0
)

# --- YAML SECTION ---
find_package(yaml-cpp REQUIRED)

# --- TEST EXECUTABLE ---
add_executable(gl_composite_test prototest.cpp)

target_link_libraries(gl_composite_test
    PRIVATE
        proto_lib
        PkgConfig::GST
        PkgConfig::GST_RTP
        m
)

# --- BASE STATION EXECUTABLE ---
add_executable(base_station
    main.cpp
    Config.cpp
    BuoyNode.cpp
    VideoComposite.cpp
    MathHelpers.cpp
)

target_link_libraries(base_station
    PRIVATE
        proto_lib
        yaml-cpp::yaml-cpp
        PkgConfig::GST
        PkgConfig::GST_RTP
        m
)