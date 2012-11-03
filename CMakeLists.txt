cmake_minimum_required(VERSION 2.6)

PROJECT(CameraCalibration)

include_directories(/media/portable/src/Eigen)

add_library(libCameraCalibration CameraCalibration.cpp)
#TARGET_LINK_LIBRARIES(libCameraCalibration )

ADD_EXECUTABLE(Example Example.cpp)
TARGET_LINK_LIBRARIES(Example libCameraCalibration)

ADD_EXECUTABLE(Demo Demo.cpp)
TARGET_LINK_LIBRARIES(Demo libCameraCalibration)
