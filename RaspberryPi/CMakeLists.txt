cmake_minimum_required(VERSION 3.10)
project(corvus_gcs)

set(CMAKE_CXX_STANDARD 17)

find_package(MAVSDK REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

add_executable(corvus_gcs main.cpp)

target_link_libraries(corvus_gcs
    MAVSDK::mavsdk
    ${OpenCV_LIBS}
    Threads::Threads
)
