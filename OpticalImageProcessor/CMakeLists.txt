cmake_minimum_required(VERSION 3.10)

project(OpticalImageProcessor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

find_package(OpenCV REQUIRED core imgproc imgcodecs)
find_package(NumCpp REQUIRED)
find_package(CLI11 REQUIRED)
find_package(GDAL REQUIRED)

add_executable(${PROJECT_NAME} main.cpp)

if (UNIX AND NOT APPLE)
target_include_directories(${PROJECT_NAME} PUBLIC /home/linuxbrew/.linuxbrew/include)
endif()
target_include_directories(${PROJECT_NAME} PUBLIC /usr/local/include)
target_include_directories(${PROJECT_NAME} PUBLIC ${NumCPP_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PUBLIC ${CLI11_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PUBLIC ${GDAL_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} LINK_PUBLIC ${GDAL_LIBRARIES})

if (UNIX AND NOT APPLE)
target_link_options(${PROJECT_NAME} PUBLIC "-Wl,--copy-dt-needed-entries")
endif()
