cmake_minimum_required(VERSION 2.6)

PROJECT(CameraCalibration)

include_directories(/media/portable/src/Eigen)

FIND_PACKAGE(VXL REQUIRED)
INCLUDE(${VXL_CMAKE_DIR}/UseVXL.cmake)

ADD_EXECUTABLE(CameraCalibration
Example.cpp
CameraCalibration.cpp 
)

TARGET_LINK_LIBRARIES(CameraCalibration
vgl vgl_algo vnl vil)

