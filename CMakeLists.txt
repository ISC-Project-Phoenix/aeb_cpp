cmake_minimum_required(VERSION 3.23)
project(aeb)
set(CMAKE_CXX_STANDARD 17)

add_library(aeb STATIC grid.hpp aeb.hpp)

set_target_properties(aeb PROPERTIES LINKER_LANGUAGE CXX)

add_executable(dummy test.cpp)
target_link_libraries(dummy PUBLIC aeb)