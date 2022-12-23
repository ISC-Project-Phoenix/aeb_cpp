cmake_minimum_required(VERSION 3.16)
project(tests)
set(CMAKE_CXX_STANDARD 17)

include(FetchContent)
FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(
        unit_tests
        grid_tests.cpp
        aeb_tests.cpp
)
target_link_libraries(
        unit_tests
        GTest::gtest_main
        aeb_cpp
)

include(GoogleTest)
gtest_discover_tests(unit_tests)