cmake_minimum_required(VERSION 3.8)
project(thirdparty)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set_property(GLOBAL PROPERTY ALLOW_DUPLICATE_CUSTOM_TARGETS TRUE)

# find dependencies
find_package(ament_cmake REQUIRED)

include(../../cmake/CPM.cmake)

CPMAddPackage(
  NAME DBoW3
  GIT_REPOSITORY https://github.com/yan99033/DBow3.git
  GIT_TAG master
  GIT_SHALLOW TRUE GIT_PROGRESS TRUE
)

CPMAddPackage(
  NAME g2o
  GIT_REPOSITORY https://github.com/RainerKuemmerle/g2o.git
  GIT_TAG e8df200 # 20230223_git
  GIT_SHALLOW TRUE GIT_PROGRESS TRUE
  OPTIONS "G2O_BUILD_APPS OFF" "G2O_BUILD_EXAMPLES OFF"
)

CPMAddPackage(
  NAME Eigen
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0
  GIT_SHALLOW TRUE GIT_PROGRESS TRUE
  OPTIONS "EIGEN_BUILD_DOC OFF" "BUILD_TESTING OFF"
)

ament_package()
