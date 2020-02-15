cmake_minimum_required(VERSION 3.7)


project(pcover)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/")

find_package(SDL2 REQUIRED)
find_package(SDL2_image REQUIRED)
find_package(SDL2_ttf REQUIRED)
find_package(SDL2_mixer REQUIRED)
include_directories(${SDL2_INCLUDE_DIRS} ${SDL2_IMAGE_INCLUDE_DIRS} ${SDL2_TTF_INCLUDE_DIRS} ${SDL2_MIXER_INCLUDE_DIRS})



link_directories(
    "${CMAKE_SOURCE_DIR}/glpk")


add_subdirectory(mathtool)
add_subdirectory(tsp)

file(GLOB MY_SRC 
    "*.h"
    "*.cpp"
)

add_executable(pcover ${MY_SRC})
target_link_libraries(pcover mathtool tsp ${SDL2_LIBRARIES} ${SDL2_IMAGE_LIBRARIES} ${SDL2_TTF_LIBRARIES} ${SDL2_MIXER_LIBRARIES} libglpk.osx.a)
