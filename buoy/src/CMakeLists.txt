cmake_minimum_required(VERSION 3.10)
project(buoy_app)

find_package(Protobuf REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED gstreamer-1.0)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS configs/buoy.proto)

add_library(proto_lib STATIC ${PROTO_SRCS} ${PROTO_HDRS})
target_compile_features(proto_lib PUBLIC cxx_std_17)

target_link_libraries(proto_lib PUBLIC protobuf::libprotobuf)
target_include_directories(proto_lib PUBLIC ${CMAKE_CURRENT_BINARY_DIR})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(buoy_app main.cpp BNO055.cpp I2CTransport.cpp IMUProtoSender.cpp VideoStreamer.cpp Config.cpp)

target_link_libraries(buoy_app PRIVATE proto_lib)

target_include_directories(buoy_app PRIVATE ${GST_INCLUDE_DIRS})
target_link_libraries(buoy_app PRIVATE ${GST_LIBRARIES} yaml-cpp)