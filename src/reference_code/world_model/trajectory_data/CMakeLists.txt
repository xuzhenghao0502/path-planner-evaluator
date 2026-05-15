project(trajectory_data)

pnc_add_library(
  ${PROJECT_NAME}
  OBJECT
  DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  INCLUDE_DIR
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  LIBRARY
  PUBLIC
  gpal::pnc_common_util
  gpal::pnc_solver
  PRIVATE
  obstacle
  decision_data
  reference_line
  center_lines
  adas_data
  navigation_data
  memorized_route
  config_proto
  speed_data
)

install(DIRECTORY include/ DESTINATION include/${CMAKE_PROJECT_NAME})
