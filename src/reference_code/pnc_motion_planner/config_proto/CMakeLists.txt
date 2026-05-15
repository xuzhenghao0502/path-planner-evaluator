project(config_proto)

# world_model config_proto 提供 base_object_parser_config / base_lateral_path_bound_parser_config
set(WORLD_MODEL_CONFIG_PROTO_DIR "${CMAKE_SOURCE_DIR}/../world_model/config_proto")
if(EXISTS "${WORLD_MODEL_CONFIG_PROTO_DIR}/config/base_object_parser_config.proto")
  set(CONFIG_PROTO_IMPORT "${WORLD_MODEL_CONFIG_PROTO_DIR}" ${CMAKE_INSTALL_PREFIX}/share/pnc_solver/config_proto)
else()
  set(CONFIG_PROTO_IMPORT ${CMAKE_INSTALL_PREFIX}/share/world_model/config_proto ${CMAKE_INSTALL_PREFIX}/share/pnc_solver/config_proto)
endif()

generate_proto(
  ROOT ${CMAKE_CURRENT_SOURCE_DIR}
  IMPORT ${CONFIG_PROTO_IMPORT}
  TARGET ${PROJECT_NAME} OBJECT INSTALL_INCLUDE
)
