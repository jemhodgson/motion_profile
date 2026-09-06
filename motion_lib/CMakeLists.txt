cmake_minimum_required(VERSION 3.10)
project(motion_lib CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(motion_lib STATIC
    src/motion_profile.cpp
)
target_include_directories(motion_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(motion_lib_example examples/example.cpp)
target_link_libraries(motion_lib_example PRIVATE motion_lib)

enable_testing()
add_executable(motion_lib_tests tests/test_motion_profile.cpp)
target_link_libraries(motion_lib_tests PRIVATE motion_lib)
add_test(NAME motion_lib_tests COMMAND motion_lib_tests)
