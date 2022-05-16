file(GLOB_RECURSE SOURCE_CODE *.cpp )
file(GLOB_RECURSE HEADERS_CODE *.h )
add_library(cfree STATIC ${SOURCE_CODE} ${HEADERS_CODE})

target_include_directories(cfree PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(cfree ${OpenCV_LIBS})


install(TARGETS cfree DESTINATION lib)
install(FILES ${HEADERS} DESTINATION mrrm/)