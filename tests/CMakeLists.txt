set( L_EXE "")
add_executable(main main.cpp)
#list(APPEND L_EXE 3robots)
#add_executable(testing testing.cpp)

FOREACH(EXE_NAME IN LISTS L_EXE)
    target_link_libraries(${EXE_NAME}
            cfree
            )

    target_include_directories(${EXE_NAME}
            PUBLIC ${PROJECT_INCLUDE_DIR}
            PUBLIC ${PROJECT_SOURCE_DIR}/ext/mrenv/include
            )

    install (TARGETS ${EXE_NAME} RUNTIME DESTINATION ${EXECUTABLE_OUTPUT_PATH})
ENDFOREACH()