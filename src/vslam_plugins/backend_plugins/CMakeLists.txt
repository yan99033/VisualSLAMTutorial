cmake_minimum_required(VERSION 3.8)
project(vslam_backend_plugins)

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
find_package(OpenCV REQUIRED)
find_package(pluginlib REQUIRED)
find_package(vslam_plugins_base REQUIRED)
find_package(vslam_datastructure REQUIRED)
find_package(vslam_msgs REQUIRED)
find_package(vslam_utils REQUIRED)
find_package(g2o REQUIRED)
find_package(Eigen3 REQUIRED)

set(dependencies OpenCV pluginlib vslam_plugins_base vslam_datastructure vslam_msgs vslam_utils)

set(sources src/backend.cpp src/indirect_optimizer.cpp src/utils.cpp)

add_library(${PROJECT_NAME} SHARED ${sources})
target_link_libraries(
  ${PROJECT_NAME}
  g2o::core
  g2o::types_sba
  g2o::types_sim3
  g2o::solver_cholmod
  g2o::solver_eigen
  Eigen3::Eigen
)
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

pluginlib_export_plugin_description_file(vslam_plugins_base plugins.xml)

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights comment the line when a
  # copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo) comment the line when this package
  # is in a git repo and when a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME})

ament_package()
