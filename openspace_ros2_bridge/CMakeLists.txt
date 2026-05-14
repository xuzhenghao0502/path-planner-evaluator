cmake_minimum_required(VERSION 3.14)
project(openspace_ros2_bridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find ROS2 humble
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(std_srvs REQUIRED)

# Reference code paths
set(REFERENCE_ROOT ${CMAKE_SOURCE_DIR}/../reference_code)
set(WORLD_MODEL_ROOT ${REFERENCE_ROOT}/world_model)
set(PNC_MOTION_ROOT ${REFERENCE_ROOT}/pnc_motion_planner)

# ============================================================
# Stub headers for missing GPAL internal dependencies
# ============================================================
set(STUB_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/stubs)

# ============================================================
# Build lib: core algorithm library (subset from reference_code)
# ============================================================
set(ALGORITHM_SOURCES
  # World model - Freespace + GridMap
  ${WORLD_MODEL_ROOT}/local_view/src/Freespace.cpp

  # Path planner - openspace core utilities
  ${PNC_MOTION_ROOT}/path_planner/src/openspace_path_planner/core/generator/rs_path.cpp
  ${PNC_MOTION_ROOT}/path_planner/src/openspace_path_planner/utils/openspace_tools.cpp
  ${PNC_MOTION_ROOT}/path_planner/src/openspace_path_planner/utils/geometry_utils.cpp
)

add_library(openspace_algorithm OBJECT ${ALGORITHM_SOURCES})

target_include_directories(openspace_algorithm PUBLIC
  ${STUB_INCLUDE_DIR}
  ${WORLD_MODEL_ROOT}/local_view/include
  ${WORLD_MODEL_ROOT}/obstacle/include
  ${WORLD_MODEL_ROOT}/path_data/include
  ${WORLD_MODEL_ROOT}/reference_line/include
  ${WORLD_MODEL_ROOT}/reference_line_info/include
  ${WORLD_MODEL_ROOT}/decision_data/include
  ${WORLD_MODEL_ROOT}/navigation_data/include
  ${WORLD_MODEL_ROOT}/center_lines/include
  ${WORLD_MODEL_ROOT}/adas_data/include
  ${WORLD_MODEL_ROOT}/memorized_route/include
  ${WORLD_MODEL_ROOT}/road_instance/include
  ${WORLD_MODEL_ROOT}/ellipse_object_info/include
  ${WORLD_MODEL_ROOT}/trajectory_boundary/include
  ${WORLD_MODEL_ROOT}/trajectory_data/include
  ${WORLD_MODEL_ROOT}/speed_data/include
  ${WORLD_MODEL_ROOT}/model/bicycle_trajectory_tracker
  ${PNC_MOTION_ROOT}/path_planner/include
  ${PNC_MOTION_ROOT}/speed_planner/include
  ${PNC_MOTION_ROOT}/spatiotemporal_planner/include
  ${PNC_MOTION_ROOT}/redundant_planner/include
  ${EIGEN3_INCLUDE_DIRS}
)

target_compile_definitions(openspace_algorithm PRIVATE OPENSPACE_STANDALONE)
target_link_libraries(openspace_algorithm PUBLIC fmt::fmt)

# ============================================================
# ROS2 Node: planner_bridge
# ============================================================
add_executable(planner_bridge_node
  src/planner_bridge_node.cpp
  src/freespace_adapter.cpp
)
ament_target_dependencies(planner_bridge_node
  rclcpp nav_msgs visualization_msgs geometry_msgs std_msgs std_srvs Eigen3
)
target_link_libraries(planner_bridge_node
  openspace_algorithm
  fmt::fmt
)
target_include_directories(planner_bridge_node PRIVATE
  ${CMAKE_SOURCE_DIR}/include
)

# ============================================================
# Install
# ============================================================

install(TARGETS planner_bridge_node DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/map_editor_node.py scripts/map_tf_publisher.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY scenes DESTINATION share/${PROJECT_NAME})

ament_package()
