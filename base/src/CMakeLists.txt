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

find_package(OpenCV REQUIRED COMPONENTS core imgproc objdetect highgui dnn)

option(ENABLE_ORT_YOLO "Enable YOLOv8 backend via ONNX Runtime C++" OFF)
set(ONNXRUNTIME_ROOT "" CACHE PATH "Path to extracted ONNX Runtime release with include/ and lib/")
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


if(ENABLE_ORT_YOLO)
    find_path(ONNXRUNTIME_INCLUDE_DIR
        NAMES onnxruntime/core/session/onnxruntime_cxx_api.h
        HINTS ${ONNXRUNTIME_ROOT}/include)
    find_library(ONNXRUNTIME_LIB
        NAMES onnxruntime
        HINTS ${ONNXRUNTIME_ROOT}/lib ${ONNXRUNTIME_ROOT}/lib64)
    if(ONNXRUNTIME_INCLUDE_DIR AND ONNXRUNTIME_LIB)
        message(STATUS "Found ONNX Runtime: ${ONNXRUNTIME_LIB}")
        target_include_directories(base_station PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
        target_link_libraries(base_station PRIVATE ${ONNXRUNTIME_LIB})
        target_compile_definitions(base_station PRIVATE HAVE_ORT=1)
        get_filename_component(ONNXRUNTIME_LIB_DIR ${ONNXRUNTIME_LIB} DIRECTORY)
        set_target_properties(base_station PROPERTIES BUILD_RPATH ${ONNXRUNTIME_LIB_DIR})
    else()
        message(FATAL_ERROR "ENABLE_ORT_YOLO=ON but ONNX Runtime was not found. Set ONNXRUNTIME_ROOT to the extracted release directory.")
    endif()
endif()
