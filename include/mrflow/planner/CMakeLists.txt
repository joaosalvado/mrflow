file(GLOB_RECURSE SOURCE_CODE *.cpp )
file(GLOB_RECURSE HEADERS_CODE *.h )
add_library(planner STATIC ${SOURCE_CODE} ${HEADERS_CODE})

target_link_libraries(planner cfree base)

install(TARGETS planner  DESTINATION lib)
install(FILES ${HEADERS} DESTINATION mrflow/)