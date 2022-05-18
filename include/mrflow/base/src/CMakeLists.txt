INCLUDE_DIRECTORIES(
  ${CMAKE_SOURCE_DIR}
  ${CMAKE_SOURCE_DIR}/src
  ${CMAKE_BINARY_DIR}/src
)

## Here we define an executable target. Its name is 'lemon-project' and
## is compiled from 'main.cc'. You can add more source files separated
## with whitespaces (including newlines). If you want to build more
## executables, simple repeat (and edit) the following ADD_EXECUTABLE and
## TARGET_LINK_LIBRARIES statements

INCLUDE_DIRECTORIES(include)

FIND_PACKAGE( Boost 1.40 COMPONENTS system REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

ADD_LIBRARY(base
  MultirobotNetwork.cpp
        ../MultirobotNetwork.hpp
  FlowRobots.cpp
        ../FlowRobots.hpp
)

## This tells cmake to install 'lemon-project' to $PREFIX/bin when
## 'make install' is executed. You can give more targets separated
## by whitespaces.

TARGET_LINK_LIBRARIES(
  base
  ${LEMON_LIBRARIES}
)
