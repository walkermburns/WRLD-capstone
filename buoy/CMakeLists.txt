cmake_minimum_required(VERSION 3.10)
project(CameraRecorder)

# Find the OpenCV package installed on the system
find_package(OpenCV REQUIRED)

# Add the executable
add_executable(cam_record opencv_test.cpp)

# Link OpenCV libraries to your executable
target_link_libraries(cam_record ${OpenCV_LIBS})