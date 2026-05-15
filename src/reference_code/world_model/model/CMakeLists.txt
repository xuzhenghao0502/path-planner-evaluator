project(model)

pnc_add_library(
  ${PROJECT_NAME}
  OBJECT
  DIRECTORY
  ${CMAKE_CURRENT_SOURCE_DIR}
  INCLUDE_DIR
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
  LIBRARY
  PUBLIC
  OsqpEigen::OsqpEigen
  gpal::pnc_common_util
  gpal::pnc_solver
  PRIVATE
  config_proto
)
