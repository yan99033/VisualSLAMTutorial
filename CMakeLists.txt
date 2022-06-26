cmake_minimum_required(VERSION 3.14...3.22)

# ---- Project ----

# Note: update this to your new project's name and version
project(
  VisualSlamLibraries
  VERSION 1.0
  LANGUAGES CXX
)

# ---- Include guards ----

if(PROJECT_SOURCE_DIR STREQUAL PROJECT_BINARY_DIR)
  message(
    FATAL_ERROR
      "In-source builds not allowed. Please make a new directory (called a build directory) and run CMake from there."
  )
endif()

# ---- Add dependencies ----

find_package(OpenCV REQUIRED)

# ---- Add dependencies via CPM ----
# see https://github.com/TheLartians/CPM.cmake for more info

include(cmake/CPM.cmake)

# PackageProject.cmake will be used to make our target installable
CPMAddPackage("gh:TheLartians/PackageProject.cmake@1.8.0")

CPMAddPackage(
  NAME fmt
  GIT_TAG 8.1.1
  GITHUB_REPOSITORY fmtlib/fmt
  OPTIONS "FMT_INSTALL YES" # create an installable target
          "CMAKE_POSITION_INDEPENDENT_CODE ON"
)

CPMAddPackage(
  NAME GSL
  GITHUB_REPOSITORY "Microsoft/GSL"
  VERSION "4.0.0"
  GIT_SHALLOW ON
)

# ---- Add source files ----

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

add_subdirectory(libraries/feature_detector)
add_subdirectory(libraries/datastructure)
add_subdirectory(libraries/image_loader)
add_subdirectory(libraries/camera_tracker)
add_subdirectory(libraries/mapping)
add_subdirectory(libraries/visualizer)
