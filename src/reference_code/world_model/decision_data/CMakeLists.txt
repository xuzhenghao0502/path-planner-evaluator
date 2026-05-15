project(decision_data)

pnc_add_library(
  ${PROJECT_NAME}
  OBJECT
  DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  INCLUDE_DIR
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/local_view/include>
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/ellipse_object_info/include>
  LIBRARY
  PUBLIC
  gpal::pnc_common_util
  gpal::pnc_solver
  PRIVATE
  memorized_route
  center_lines
  adas_data
  navigation_data
  config_proto
  reference_line
  obstacle
  road_instance
  trajectory_boundary
)

if(BUILD_TESTING)
  target_compile_options(${PROJECT_NAME} PRIVATE -fprofile-arcs -ftest-coverage)
endif()

install(DIRECTORY include/ DESTINATION include/${CMAKE_PROJECT_NAME})
