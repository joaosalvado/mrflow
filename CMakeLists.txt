cmake_minimum_required(VERSION 3.16)

# set the project name
project(mrflow)

set (CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++17")
#set(CMAKE_CXX_CLANG_TIDY "/usr/bin/clang-tidy-11")
#set(CMAKE_CXX_CPPCHECK "/usr/bin/cppcheck")
#set(CMAKE_CXX_CPPLINT "/usr/local/bin/cpplint")
#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
#SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")
#SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -pg")

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

#Project directories
set(EXECUTABLE_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/bin")
set(LIBRARY_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/lib")
set(PROJECT_INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include/")


# Dependencies
find_package( OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS})


include_directories(${PROJECT_SOURCE_DIR}/include/mrflow)
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR})

# External libraries
include_directories(${PROJECT_SOURCE_DIR}/ext/mrenv/include)
message(${PROJECT_SOURCE_DIR}/ext/mrenv/include)
add_subdirectory(${PROJECT_SOURCE_DIR}/ext/mrenv)

# Mrflow library and executables
add_subdirectory(${PROJECT_SOURCE_DIR}/include)
add_subdirectory(${PROJECT_SOURCE_DIR}/tests)
