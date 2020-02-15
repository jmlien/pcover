#######################################################
# Binary for TSP
#######################################################

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR})

set(LIB_NAME tsp)

set(LIB_SRC
    GraphTSP.cpp
    tsp_writer.cpp)

include_directories(
    "${CMAKE_SOURCE_DIR}/src"
    "${CMAKE_SOURCE_DIR}/libs"
)

add_library(${LIB_NAME} STATIC ${LIB_SRC})


set(LIBS tsp)

