cmake_minimum_required(VERSION 3.10)
project(drone_gui)

# 1. Set C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 2. Find Packages
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)

# 3. Find GStreamer components using PkgConfig
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-video-1.0 gstreamer-app-1.0)

# 4. Include Directories
include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${GST_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR} # This is for crow_all.h
)

# 5. Add Executable
add_executable(drone_gui main.cpp)

# 6. Link Libraries
target_link_libraries(drone_gui
    ${OpenCV_LIBS}
    ${GST_LIBRARIES}
    pthread
)
