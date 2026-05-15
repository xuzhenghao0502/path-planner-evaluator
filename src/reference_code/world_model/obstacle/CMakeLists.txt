project(obstacle)

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
  gpal::gpal-interface
  gpal::pnc_common_util
  gpal::pnc_solver
  PRIVATE
  memorized_route
  reference_line
  center_lines
  adas_data
  navigation_data
  config_proto
)

if(BUILD_TESTING)
  target_compile_options(${PROJECT_NAME} PRIVATE -fprofile-arcs -ftest-coverage)
endif()

install(DIRECTORY include/ DESTINATION include/${CMAKE_PROJECT_NAME})
