cmake_minimum_required(VERSION 2.6)

#PROJECT(CameraCalibration)

include_directories(/media/portable/src/Eigen)

FIND_PACKAGE(VXL REQUIRED)
INCLUDE(${VXL_CMAKE_DIR}/UseVXL.cmake)

add_library(libCameraCalibration CameraCalibration.cpp)
TARGET_LINK_LIBRARIES(libCameraCalibration vgl vgl_algo vnl vil)

ADD_EXECUTABLE(Example Example.cpp)
TARGET_LINK_LIBRARIES(Example libCameraCalibration vgl vgl_algo vnl vil)

ADD_EXECUTABLE(Demo Demo.cpp)
TARGET_LINK_LIBRARIES(Demo libCameraCalibration vgl vgl_algo vnl vil)
