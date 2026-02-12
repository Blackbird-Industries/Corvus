cmake_minimum_required(VERSION 3.10)
project(CorvusGCS)

set(CMAKE_CXX_STANDARD 17)

# 1. Load PkgConfig to find AprilTag
find_package(PkgConfig REQUIRED)
pkg_check_modules(APRILTAG REQUIRED apriltag)

# 2. Find OpenCV with GStreamer support
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui videoio imgcodecs)

# 3. Find MAVSDK and Threads
find_package(MAVSDK REQUIRED)
find_package(Threads REQUIRED)

# 4. Include all headers
include_directories(
    ${OpenCV_INCLUDE_DIRS} 
    ${APRILTAG_INCLUDE_DIRS}
    ${MAVSDK_INCLUDE_DIRS}
)

# 5. Create Executable
add_executable(corvus_gcs main.cpp)

# 6. Link Libraries
# Note: ${APRILTAG_LIBRARIES} usually resolves to 'apriltag'
target_link_libraries(corvus_gcs 
    ${OpenCV_LIBS} 
    ${APRILTAG_LIBRARIES}
    MAVSDK::mavsdk
    Threads::Threads
)
