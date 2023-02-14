# this script adds all subprojects to a single build to allow IDEs understand the full project
# structure.

cmake_minimum_required(VERSION 3.14...3.22)

project(BuildAll LANGUAGES CXX)

include(../cmake/tools.cmake)

CPMAddPackage("gh:TheLartians/Format.cmake@1.7.3")

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../standalone ${CMAKE_BINARY_DIR}/standalone)
