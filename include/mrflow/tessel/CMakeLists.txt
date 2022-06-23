file(GLOB_RECURSE SOURCE_CODE *.cpp )
file(GLOB_RECURSE HEADERS_CODE *.h )
add_library(tessel_interface STATIC ${SOURCE_CODE} ${HEADERS_CODE})

target_link_libraries(tessel_interface mrenv cfree )


install(TARGETS tessel_interface DESTINATION lib)
install(FILES ${HEADERS} DESTINATION mrflow/)