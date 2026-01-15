cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(guidecam)

find_package(OpenCV REQUIRED)

include_directories(
    include
    )

add_executable(guidemono src/guidemono.cpp)
target_link_libraries(guidemono
    ${OpenCV_LIBS}
    pthread
    serial
    )

add_executable(guidestereo src/guidestereo.cpp)
target_link_libraries(guidestereo
    ${OpenCV_LIBS}
    pthread
    serial
    )