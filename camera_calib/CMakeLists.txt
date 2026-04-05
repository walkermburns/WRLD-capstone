cmake_minimum_required(VERSION 3.10)
project(camera_calibration LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d highgui videoio aruco objdetect)

add_executable(camera_calibration camera_calibration.cpp)

target_link_libraries(camera_calibration PRIVATE ${OpenCV_LIBS})
