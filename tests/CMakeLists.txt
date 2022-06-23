set( L_EXE "")

set(mrflow_exe "mrflow_exe")
add_executable(mrflow_exe test.cpp)
list(APPEND L_EXE mrflow_exe)
#add_executable(testing testing.cpp)

FOREACH(EXE_NAME IN LISTS L_EXE)
    #target_include_directories(${EXE_NAME}  PUBLIC
    #        ${PROJECT_INCLUDE_DIR}
    #        #PUBLIC ${PROJECT_SOURCE_DIR}/ext/mrenv/include
    #        )

    target_link_libraries(${EXE_NAME}
            mrflow_lib
            tessel_interface
            #mrenv
            #cfree
            #planner
            #PUBLIC cfree
            )

    install (TARGETS ${EXE_NAME} RUNTIME DESTINATION ${EXECUTABLE_OUTPUT_PATH})
ENDFOREACH()