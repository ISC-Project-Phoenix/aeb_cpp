cmake_minimum_required(VERSION 3.16)
project(aeb_cpp)
set(CMAKE_CXX_STANDARD 17)

add_library(aeb_cpp INTERFACE)
target_include_directories(aeb_cpp INTERFACE include)

add_subdirectory(examples)
add_subdirectory(test)