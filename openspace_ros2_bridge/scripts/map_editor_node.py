#!/usr/bin/env python3
"""
Map Editor Node - RViz2 interactive polygon drawing tool for openspace planner.

Usage in RViz2:
  1. Use "Publish Point" tool to click polygon vertices
  2. Call ~/finish_polygon service to close the current polygon
  3. Call ~/undo_last_point to remove the last vertex
  4. Call ~/set_mode with "obstacle" or "free_space" to switch drawing mode
  5. Call ~/clear_all to remove all polygons
  6. Call ~/save_scene / ~/load_scene to persist scenes as JSON
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter

from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import MarkerArray, Marker
from std_srvs.srv import Trigger
from std_msgs.msg import String, ColorRGBA

import json
import os


class MapEditorNode(Node):
    def __init__(self):
        super().__init__("map_editor_node")

        self.declare_parameter("scene_dir", "/root/ros2_ws/src/openspace_ros2_bridge/scenes")
        self.declare_parameter("scene_name", "scene.json")

        # State
        self.mode = "obstacle"  # "obstacle" or "free_space"
        self.current_polygon = []  # list of (x, y) tuples being drawn
        self.polygons = []  # list of dicts: {"vertices": [(x,y),...], "is_occupied": bool}

        # Publisher: polygon MarkerArray to /obstacle_polygons (consumed by planner)
        self.polygon_pub = self.create_publisher(MarkerArray, "/obstacle_polygons", 10)
        # Publisher: drawing preview as MarkerArray
        self.viz_pub = self.create_publisher(MarkerArray, "/polygon_viz", 10)
        # Publisher: mode feedback
        self.mode_pub = self.create_publisher(String, "/map_editor/mode", 10)

        # Subscriber: clicked points from RViz2 "Publish Point" tool
        self.click_sub = self.create_subscription(
            PointStamped, "/clicked_point", self.on_click, 10
        )

        # Services
        self.create_service(Trigger, "~/finish_polygon", self.on_finish_polygon)
        self.create_service(Trigger, "~/undo_last_point", self.on_undo_last_point)
        self.create_service(Trigger, "~/clear_all", self.on_clear_all)
        self.create_service(Trigger, "~/save_scene", self.on_save_scene)
        self.create_service(Trigger, "~/load_scene", self.on_load_scene)
        self.create_service(Trigger, "~/set_mode_obstacle", self.on_set_mode_obstacle)
        self.create_service(Trigger, "~/set_mode_free", self.on_set_mode_free)

        self.get_logger().info(
            'Map Editor Node ready. Mode: obstacle. '
            'Use RViz2 "Publish Point" to click vertices, '
            'then call ~/finish_polygon to close.'
        )

    # ---- Topic callbacks ----

    def on_click(self, msg: PointStamped):
        x, y = msg.point.x, msg.point.y
        self.current_polygon.append((x, y))
        self.get_logger().info(
            f"Added vertex ({x:.2f}, {y:.2f}) — "
            f"polygon now has {len(self.current_polygon)} points"
        )
        self._publish_all()

    # ---- Service callbacks ----

    def on_finish_polygon(self, request, response):
        if len(self.current_polygon) < 3:
            response.success = False
            response.message = f"Need at least 3 points, have {len(self.current_polygon)}"
            self.get_logger().warn(response.message)
            return response

        poly = {
            "vertices": list(self.current_polygon),
            "is_occupied": (self.mode == "obstacle"),
        }
        self.polygons.append(poly)
        n = len(self.current_polygon)
        self.current_polygon = []
        self._publish_all()

        response.success = True
        response.message = f"Polygon closed ({n} vertices, mode={self.mode}). Total polygons: {len(self.polygons)}"
        self.get_logger().info(response.message)
        return response

    def on_undo_last_point(self, request, response):
        if self.current_polygon:
            p = self.current_polygon.pop()
            response.success = True
            response.message = f"Removed vertex ({p[0]:.2f}, {p[1]:.2f}). Remaining: {len(self.current_polygon)}"
            self._publish_all()
        else:
            response.success = False
            response.message = "No vertices in current polygon"
        return response

    def on_clear_all(self, request, response):
        n = len(self.polygons)
        self.polygons = []
        self.current_polygon = []
        self._publish_all()
        response.success = True
        response.message = f"Cleared {n} polygons"
        self.get_logger().info(response.message)
        return response

    def on_set_mode_obstacle(self, request, response):
        self.mode = "obstacle"
        msg = String(data="obstacle")
        self.mode_pub.publish(msg)
        response.success = True
        response.message = "Mode set to obstacle (red)"
        self.get_logger().info(response.message)
        self._publish_all()
        return response

    def on_set_mode_free(self, request, response):
        self.mode = "free_space"
        msg = String(data="free_space")
        self.mode_pub.publish(msg)
        response.success = True
        response.message = "Mode set to free_space (green)"
        self.get_logger().info(response.message)
        self._publish_all()
        return response

    def on_save_scene(self, request, response):
        scene_dir = self.get_parameter("scene_dir").value
        scene_name = self.get_parameter("scene_name").value
        os.makedirs(scene_dir, exist_ok=True)
        path = os.path.join(scene_dir, scene_name)

        data = {"polygons": self.polygons}
        with open(path, "w") as f:
            json.dump(data, f, indent=2)

        response.success = True
        response.message = f"Saved {len(self.polygons)} polygons to {path}"
        self.get_logger().info(response.message)
        return response

    def on_load_scene(self, request, response):
        scene_dir = self.get_parameter("scene_dir").value
        scene_name = self.get_parameter("scene_name").value
        path = os.path.join(scene_dir, scene_name)

        if not os.path.exists(path):
            response.success = False
            response.message = f"Scene file not found: {path}"
            self.get_logger().warn(response.message)
            return response

        with open(path, "r") as f:
            data = json.load(f)

        self.polygons = data.get("polygons", [])
        self.current_polygon = []
        self._publish_all()

        response.success = True
        response.message = f"Loaded {len(self.polygons)} polygons from {path}"
        self.get_logger().info(response.message)
        return response

    # ---- Visualization ----

    def _publish_all(self):
        """Build MarkerArray for the planner and visualization."""
        arr = MarkerArray()

        # Published polygons
        for i, poly in enumerate(self.polygons):
            marker = self._make_polygon_marker(
                poly["vertices"], i, poly["is_occupied"]
            )
            arr.markers.append(marker)

        # Current drawing polygon (preview)
        if self.current_polygon:
            marker = self._make_polygon_marker(
                self.current_polygon,
                len(self.polygons),
                (self.mode == "obstacle"),
                alpha=0.3,
            )
            arr.markers.append(marker)

        self.polygon_pub.publish(arr)

        # Also publish to viz topic with point markers for vertices
        self._publish_viz(arr)

    def _publish_viz(self, arr: MarkerArray):
        """Publish additional visualization (vertex points, edges)."""
        viz = MarkerArray()

        # Add polygon edge markers
        for i, marker in enumerate(arr.markers):
            viz.markers.append(marker)

        # Add vertex sphere markers for current polygon
        for j, (x, y) in enumerate(self.current_polygon):
            pt = Marker()
            pt.header.frame_id = "map"
            pt.header.stamp = self.get_clock().now().to_msg()
            pt.ns = "vertex"
            pt.id = j
            pt.type = Marker.SPHERE
            pt.action = Marker.ADD
            pt.pose.position.x = x
            pt.pose.position.y = y
            pt.pose.position.z = 0.0
            pt.pose.orientation.w = 1.0
            pt.scale.x = 0.15
            pt.scale.y = 0.15
            pt.scale.z = 0.15
            pt.color.r = 1.0 if self.mode == "obstacle" else 0.0
            pt.color.g = 0.0 if self.mode == "obstacle" else 1.0
            pt.color.b = 0.0
            pt.color.a = 0.8
            viz.markers.append(pt)

        self.viz_pub.publish(viz)

    def _make_polygon_marker(self, vertices, idx, is_occupied, alpha=0.5):
        """Create a LINE_STRIP marker for a polygon contour."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "polygon"
        marker.id = idx
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05  # line width
        marker.color.a = alpha

        if is_occupied:
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
        else:
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

        for x, y in vertices:
            pt = Point(x=x, y=y, z=0.0)
            marker.points.append(pt)
        # Close the polygon loop
        if vertices:
            marker.points.append(Point(x=vertices[0][0], y=vertices[0][1], z=0.0))

        return marker


def main():
    rclpy.init()
    node = MapEditorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
