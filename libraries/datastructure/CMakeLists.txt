set(LIB_NAME datastructure)

add_library(${LIB_NAME} SHARED source/frame.cpp)

target_include_directories(
  ${LIB_NAME}
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/${PROJECT_NAME}-${PROJECT_VERSION}> ${OpenCV_INCLUDE_DIRS}
         ${EIGEN3_INCLUDE_DIR}
)

target_link_libraries(
  ${LIB_NAME}
  PUBLIC ${OpenCV_LIBRARIES}
  PRIVATE feature_detector fmt::fmt
)

packageProject(
  NAME ${LIB_NAME}
  VERSION ${PROJECT_VERSION}
  NAMESPACE ${PROJECT_NAME}
  BINARY_DIR ${PROJECT_BINARY_DIR}
  INCLUDE_DIR ${PROJECT_SOURCE_DIR}/include
  INCLUDE_DESTINATION include/${LIB_NAME}-${PROJECT_VERSION}
  VERSION_HEADER "${VERSION_HEADER_LOCATION}"
  COMPATIBILITY SameMajorVersion
  DEPENDENCIES "fmt 8.1.1"
)
