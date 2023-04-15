cmake_minimum_required(VERSION 3.8)
project(thirdparty)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

include(../../cmake/CPM.cmake)

CPMAddPackage(
  NAME g2o
  GIT_REPOSITORY https://github.com/RainerKuemmerle/g2o.git
  GIT_TAG e8df200 # 20230223_git
  OPTIONS "G2O_BUILD_APPS OFF" "G2O_BUILD_EXAMPLES OFF"
)

CPMAddPackage(
  NAME Eigen 
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG master
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE
  OPTIONS "EIGEN_BUILD_DOC OFF" "BUILD_TESTING OFF"
)

ament_package()
