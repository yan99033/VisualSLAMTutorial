cmake_minimum_required(VERSION 3.8)
project(vslam_datastructure)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(vslam_msgs REQUIRED)
find_package(vslam_utils REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(sources
  src/frame.cpp
  src/point.cpp)

set(dependencies
  OpenCV
  vslam_msgs
  vslam_utils
  tf2_eigen
  tf2_geometry_msgs)

# let the compiler search for headers in the include folder
include_directories(include)

add_library(${PROJECT_NAME} ${sources})
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY include/vslam_datastructure
  DESTINATION include
)

ament_export_include_directories(
  include
)

ament_export_libraries(${PROJECT_NAME})

ament_package()
