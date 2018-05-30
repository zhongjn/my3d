cmake_minimum_required(VERSION 2.8)

project(my3d)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_RELEASE  ${PROJECT_SOURCE_DIR}/bin)

include_directories(${PROJECT_SOURCE_DIR}/include)

add_library(my3d src/my3d.cpp src/my3dpresent.cpp)