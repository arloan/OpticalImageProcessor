cmake_minimum_required(VERSION 3.10)

project(OpticalImageProcessor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(OpenCV REQUIRED core imgproc imgcodecs)
find_package(NumCpp REQUIRED)

add_executable(${PROJECT_NAME} main.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC /usr/local/include)

target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${OpenCV_LIBS})
