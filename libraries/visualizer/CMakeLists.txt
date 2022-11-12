set(LIB_NAME visualizer)

# pangolin
find_package(Pangolin 0.8 REQUIRED)

add_library(${LIB_NAME} SHARED source/visualizer.cpp)

target_include_directories(
  ${LIB_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/${PROJECT_NAME}-${PROJECT_VERSION}> ${Pangolin_INCLUDE_DIRS}
)

target_link_libraries(${LIB_NAME} PUBLIC ${Pangolin_LIBRARIES})

packageProject(
  NAME ${LIB_NAME}
  VERSION ${PROJECT_VERSION}
  NAMESPACE ${PROJECT_NAME}
  BINARY_DIR ${PROJECT_BINARY_DIR}
  INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include
  INCLUDE_DESTINATION include/${LIB_NAME}-${PROJECT_VERSION}
  VERSION_HEADER "${VERSION_HEADER_LOCATION}"
  COMPATIBILITY SameMajorVersion
  DEPENDENCIES "Pangolin v0.8"
)
