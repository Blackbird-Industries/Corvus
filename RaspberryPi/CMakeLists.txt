cmake_minimum_required(VERSION 3.10)
project(DroneGUI)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)

add_executable(drone_gui main.cpp)

# Include /usr/include to ensure asio.hpp is found
target_include_directories(drone_gui PRIVATE ${OpenCV_INCLUDE_DIRS} /usr/include .)

# REMOVED 'asio' from here to prevent the "cannot find -lasio" error
target_link_libraries(drone_gui ${OpenCV_LIBS} pthread)
