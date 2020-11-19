cmake_minimum_required(VERSION 2.8.12)
project(Utilities)
set (CMAKE_CXX_FLAGS "--std=gnu++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_BUILD_TYPE Release)

find_library(GLPKLIB glpk REQUIRED)

link_libraries(${GLPKLIB})
include_directories(/usr/include/eigen3)

add_executable(test_utilities test.cpp utilities.cpp sample.cpp contact_kinematics.cpp)
