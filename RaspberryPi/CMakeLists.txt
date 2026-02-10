cmake_minimum_required(VERSION 3.10)
project(CorvusGCS)

set(CMAKE_CXX_STANDARD 17)

# Packages
find_package(OpenCV REQUIRED)
find_package(MAVSDK REQUIRED)
find_package(Threads REQUIRED)

# AprilTag (Standard Linux Path)
find_path(APRILTAG_INCLUDE_DIR apriltag.h PATH_SUFFIXES apriltag)
find_library(APRILTAG_LIBRARY apriltag)

include_directories(${OpenCV_INCLUDE_DIRS} ${APRILTAG_INCLUDE_DIR})

add_executable(corvus_gcs main.cpp)

target_link_libraries(corvus_gcs 
    ${OpenCV_LIBS} 
    ${APRILTAG_LIBRARY}
    MAVSDK::mavsdk
    MAVSDK::mavsdk_telemetry
    Threads::Threads
)
