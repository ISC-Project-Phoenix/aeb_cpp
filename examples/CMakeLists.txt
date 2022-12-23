cmake_minimum_required(VERSION 3.16)
project(aeb_examples)
set(CMAKE_CXX_STANDARD 17)

add_executable(kinematics_demo kinematics_demo.cpp)
target_link_libraries(kinematics_demo aeb_cpp)