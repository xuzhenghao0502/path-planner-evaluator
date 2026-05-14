project(local_view)

pnc_add_library(
  ${PROJECT_NAME}
  OBJECT
  DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  INCLUDE_DIR
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  LIBRARY
  PUBLIC
  OsqpEigen::OsqpEigen
  gpal::pnc_common_util
  gpal::pnc_solver
  PRIVATE
  config_proto
  decision_data
  memorized_route
  reference_line
  obstacle
  center_lines
  adas_data
  navigation_data
  road_instance
)

if(BUILD_TESTING)
  target_compile_options(${PROJECT_NAME} PRIVATE -fprofile-arcs -ftest-coverage)
endif()

install(DIRECTORY include/ DESTINATION include/${CMAKE_PROJECT_NAME})
