cmake_minimum_required(VERSION 3.8)
project(data_loader_nodes)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pluginlib REQUIRED)
find_package(camera_plugins_base REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vslam_msgs REQUIRED)
find_package(vslam_utils REQUIRED)
find_package(OpenCV REQUIRED)

set(dependencies
  rclcpp
  rclcpp_components
  pluginlib
  camera_plugins_base
  vslam_msgs
  vslam_utils
  OpenCV
)

set(sources
  src/data_loader_node.cpp
)

include_directories(include)

# create ament index resource which references the libraries in the binary dir
add_library(${PROJECT_NAME} SHARED ${sources})
target_compile_definitions(${PROJECT_NAME}
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(
  ${PROJECT_NAME}
  ${dependencies}
)

rclcpp_components_register_nodes(${PROJECT_NAME} "vslam_components::data_loader_nodes::DataLoaderNode")


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
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
