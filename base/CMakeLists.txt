cmake_minimum_required(VERSION 3.10)
project(gl_composite_test C)

find_package(PkgConfig REQUIRED)

pkg_check_modules(GST REQUIRED
    gstreamer-1.0
    gstreamer-video-1.0
    gstreamer-gl-1.0
)

add_executable(gl_composite_test main.c)

# EXACT equivalent of `pkg-config --cflags`
target_compile_options(gl_composite_test PRIVATE
    ${GST_CFLAGS}
)

# EXACT equivalent of `pkg-config --libs`
target_link_options(gl_composite_test PRIVATE
    ${GST_LDFLAGS}
)

target_link_libraries(gl_composite_test
    ${GST_LIBRARIES}
    m
)