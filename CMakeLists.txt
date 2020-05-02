# set required version
cmake_minimum_required(VERSION 2.8)

# set project name
project(MotionProfile)

add_compile_options(-std=c++11)

# set include drectories
include_directories(${CMAKE_SOURCE_DIR}/include)

find_package(Boost COMPONENTS system signals thread REQUIRED)

# set output paths
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

	
#libs
add_library(MotionProfile SHARED src/MotionProfile.cpp)
#install
install(TARGETS MotionProfile DESTINATION /usr/lib)
install(FILES src/MotionProfile.h DESTINATION /usr/include/MotionProfile)

