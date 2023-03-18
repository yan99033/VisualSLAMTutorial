cmake_minimum_required(VERSION 3.8)
project(feature_matching_nodes)

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
find_package(vslam_msgs REQUIRED)
find_package(vslam_srvs REQUIRED)
find_package(vslam_utils REQUIRED)
find_package(feature_extraction_nodes REQUIRED)
find_package(OpenCV REQUIRED)

set(dependencies
  rclcpp
  rclcpp_components
  vslam_msgs
  vslam_srvs
  vslam_utils
  feature_extraction_nodes
)

include_directories(include)

# create ament index resource which references the libraries in the binary dir
set(LIB_NAME feature_matching_nodes)
add_library(${LIB_NAME} SHARED
  src/orb_matcher_node.cpp)
target_link_libraries(
  ${LIB_NAME} ${OpenCV_LIBRARIES}
)
target_compile_definitions(${LIB_NAME}
  PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies(${LIB_NAME} 
  ${dependencies}
)
rclcpp_components_register_nodes(${LIB_NAME} "vslam_components::feature_matching_nodes::OrbMatcherNode")

add_executable(${PROJECT_NAME}_exe
  src/main.cpp
)
target_link_libraries(${PROJECT_NAME}_exe
  ${LIB_NAME}
)
ament_target_dependencies(${PROJECT_NAME}_exe
  ${dependencies}
)

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

install(TARGETS
  feature_matching_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  ${PROJECT_NAME}_exe
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
