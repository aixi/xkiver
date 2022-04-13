cmake_minimum_required(VERSION 2.6)

project(xkiver C CXX)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

option(XKIVER_BUILD_EXAMPLES "Build Xkiver examples" ON)
option(ENABLE_TESTING "enable test" OFF)

if (${ENABLE_TESTING} STREQUAL "ON")
    enable_testing()
endif()

set(CXX_FLAGS
    -g
    -Wall
    -Wextra
    -Werror
    -Wconversion
    -Wunused-parameter
    -Wold-style-cast
    -Woverloaded-virtual
    -Wpointer-arith
    -Wshadow
    -Wwrite-strings
    -std=c++17
    -rdynamic
)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

find_package(Eigen3 REQUIRED)

find_package(OpenCV REQUIRED)

find_package(Boost REQUIRED)

include_directories(${Boost_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR})

string(TOUPPER ${CMAKE_BUILD_TYPE} BUILD_TYPE)


add_subdirectory(xkiver)

if(XKIVER_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()