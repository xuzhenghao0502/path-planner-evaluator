project(config_proto)

generate_proto(
  ROOT
  ${CMAKE_CURRENT_SOURCE_DIR} # proto root path
  IMPORT
  ${CMAKE_INSTALL_PREFIX}/share/pnc_common_util/config_proto
  ${CMAKE_INSTALL_PREFIX}/share/pnc_solver/config_proto
  TARGET
  ${PROJECT_NAME}
  OBJECT
  INSTALL_INCLUDE # generate library, optional OBJECT to generate objects
)
