project (ublox_linux)
cmake_minimum_required(VERSION 2.8)

set(
    CMAKE_RUNTIME_OUTPUT_DIRECTORY
    ${CMAKE_HOME_DIRECTORY}/bin
    )

set(
    CMAKE_LIBRARY_OUTPUT_DIRECTORY
    ${CMAKE_HOME_DIRECTORY}/lib
    )

add_definitions(
    -std=c++11 -Wno-deprecated -fconcepts -Wall -DLINUX_PLATFORM
    )

link_directories(
    /usr/local/lib
    /usr/lib
    ${CMAKE_HOME_DIRECTORY}/lib
    )

include_directories(
    ${CMAKE_HOME_DIRECTORY}/inc
    ${CMAKE_HOME_DIRECTORY}/../src
    )

set(
    ublox_parser_src
    ../src/SparkFun_Ublox_Arduino_Library.cpp
    src/Print.cpp
    src/Utils.cpp
    )

add_library(
    ublox_linux SHARED ${ublox_parser_src}
    )

#
# Ublox f9p Test
#

set(
    ublox_f9p_test_src
    examples/ublox_f9p_test.cpp
    )

add_executable(
    ublox_f9p_test ${ublox_f9p_test_src}
    )

target_link_libraries(ublox_f9p_test ublox_linux util)
