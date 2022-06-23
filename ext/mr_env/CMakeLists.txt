cmake_minimum_required(VERSION 3.16)

set (CMAKE_CXX_STANDARD 17)
# set the project name
project(mrenv)

find_package( OpenCV REQUIRED )
include_directories("/usr/local/include/")
include_directories( ${OpenCV_INCLUDE_DIRS} )
include_directories( ${PROJECT_ROOT})
include_directories(.)
include_directories( ${PROJECT_SOURCE_DIR} )
include_directories( ${CMAKE_SOURCE_DIR} )




add_subdirectory(include/mrenv/)
add_subdirectory(src/)
