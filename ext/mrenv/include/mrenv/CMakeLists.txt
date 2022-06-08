add_library(mrenv STATIC Tesselation.h Tesselation.cpp)
target_include_directories(mrenv
        PUBLIC ${OpenCV_INCLUDE_DIRS}
        PUBLIC /usr/local/include/yaml-cpp/)
target_link_libraries( mrenv
    ${OpenCV_LIBS}
    /usr/local/lib/libyaml-cpp.so
)