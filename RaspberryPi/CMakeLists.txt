cmake_minimum_required(VERSION 3.10)
project(DroneGUI)

# Use C++17 standards for modern features
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. Find the OpenCV library on the Raspberry Pi
find_package(OpenCV REQUIRED)

# 2. Add the executable name and the source file
add_executable(drone_gui main.cpp)

# 3. Tell CMake where to look for header files (the current directory and OpenCV)
target_include_directories(drone_gui PRIVATE ${OpenCV_INCLUDE_DIRS} .)

# 4. Link the necessary libraries
# - OpenCV_LIBS: The vision processing power
# - pthread: Required for the background telemetry thread
# - asio: Required by Crow for networking
target_link_libraries(drone_gui ${OpenCV_LIBS} pthread asio)
