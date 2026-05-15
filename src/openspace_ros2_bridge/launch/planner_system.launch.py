"""Launch file for openspace path planner + map editor + RViz2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("openspace_ros2_bridge")
    default_rviz = os.path.join(pkg_dir, "rviz", "polygon_editor.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz,
                description="RViz2 config file",
            ),
            DeclareLaunchArgument(
                "scene_name",
                default_value="customer_map.json",
                description="Scene file to load at startup (empty = no scene)",
            ),
            # TF publisher: map -> odom (10 Hz, required by RViz2 for fixed frame "map")
            Node(
                package="openspace_ros2_bridge",
                executable="map_tf_publisher.py",
                name="map_tf_publisher",
                output="screen",
            ),
            # Map Editor Node
            Node(
                package="openspace_ros2_bridge",
                executable="map_editor_node.py",
                name="map_editor_node",
                output="screen",
                parameters=[{
                    "scene_name": LaunchConfiguration("scene_name"),
                }],
            ),
            # Planner Bridge Node
            Node(
                package="openspace_ros2_bridge",
                executable="planner_bridge_node",
                name="planner_bridge_node",
                output="screen",
            ),
            # RViz2
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", LaunchConfiguration("rviz_config")],
            ),
        ]
    )
