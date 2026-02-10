cmake_minimum_required(VERSION 3.10)
project(CorvusGCS)

set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)
find_package(mavsdk REQUIRED)
find_package(Threads REQUIRED)

# AprilTag
find_path(APRILTAG_INCLUDE_DIR apriltag.h PATH_SUFFIXES apriltag)
find_library(APRILTAG_LIBRARY apriltag)

add_executable(corvus_gcs main.cpp)

target_link_libraries(corvus_gcs 
    ${OpenCV_LIBS} 
    ${APRILTAG_LIBRARY}
    mavsdk::mavsdk
    mavsdk::telemetry
    Threads::Threads
)
