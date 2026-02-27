cmake_minimum_required(VERSION 3.10)
# 1. Note the addition of CXX
project(gl_composite_test C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- PROTOBUF SECTION ---
find_package(Protobuf REQUIRED)
find_package(absl REQUIRED)

# Generate the files from your proto definition
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ../buoy.proto)

# Create a mini-library for the proto code
add_library(proto_lib STATIC ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(proto_lib
    PUBLIC
        protobuf::libprotobuf
        absl::log
        absl::log_internal_check_op
        absl::base
        absl::strings
)
# This is the "include line" you need so main.c can find the headers
target_include_directories(proto_lib PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

# --- GSTREAMER SECTION ---
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED IMPORTED_TARGET
    gstreamer-1.0
    gstreamer-video-1.0
    gstreamer-gl-1.0
)

# --- FINAL EXECUTABLE ---
add_executable(gl_composite_test main.cpp)

target_link_libraries(gl_composite_test
    PRIVATE
        proto_lib      # Pulls in Protobuf code + include paths
        PkgConfig::GST # Pulls in GStreamer code + include paths
        m              # Math library
)