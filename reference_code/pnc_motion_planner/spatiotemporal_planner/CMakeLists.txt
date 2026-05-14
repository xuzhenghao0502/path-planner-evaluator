project(spatiotemporal_planner)

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
    gpal::world_model
    path_planner
	PRIVATE
	config_proto
)

install(DIRECTORY include/ DESTINATION include/${CMAKE_PROJECT_NAME})
