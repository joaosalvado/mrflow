file(GLOB_RECURSE SOURCE_CODE *.cpp )
file(GLOB_RECURSE HEADERS_CODE *.h )
add_library(mrflow_lib STATIC ${SOURCE_CODE} ${HEADERS_CODE})

target_link_libraries(mrflow_lib cfree base mrenv)

install(TARGETS mrflow_lib  DESTINATION lib)
install(FILES ${HEADERS} DESTINATION mrflow/)
