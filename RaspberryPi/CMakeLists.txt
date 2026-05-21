cmake_minimum_required(VERSION 3.10)
project(corvus_gcs)

# Set C++ standard to 17 (Required by MAVSDK and Crow)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. Find OpenCV
find_package(OpenCV REQUIRED)

# 2. Find MAVSDK
find_package(MAVSDK REQUIRED)

# 3. Find AprilTag (Using PkgConfig as it often doesn't have a CMake config)
find_package(PkgConfig REQUIRED)
pkg_check_modules(APRILTAG REQUIRED apriltag)

# 4. Find Threads (Required for Crow and our workers)
find_package(Threads REQUIRED)

# Define the executable and include the header files
add_executable(corvus_gcs 
    main.cpp 
    TagAction.hpp
)

# Include directories
target_include_directories(corvus_gcs PRIVATE 
    ${OpenCV_INCLUDE_DIRS}
    ${APRILTAG_INCLUDE_DIRS}
)

# 5. Link Libraries
target_link_libraries(corvus_gcs PRIVATE 
    ${OpenCV_LIBS}
    MAVSDK::mavsdk
    ${APRILTAG_LIBRARIES}
    Threads::Threads
    asound
)

# Optional: Add compiler flags for optimization on Pi 5
if(CMAKE_COMPILER_IS_GNUCXX)
    target_compile_options(corvus_gcs PRIVATE -O3 -march=native)
endif()
