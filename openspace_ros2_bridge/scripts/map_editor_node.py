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
        """Publish planner input + visualization."""
        planner_arr = MarkerArray()   # OUTLINE only → /obstacle_polygons
        viz_arr = MarkerArray()       # Fill + outline + labels + vertices → /polygon_viz

        # Invisible ground plane (Publish Point needs a surface to click on)
        ground = Marker()
        ground.header.frame_id = "map"
        ground.header.stamp = self.get_clock().now().to_msg()
        ground.ns = "ground"
        ground.id = 0
        ground.type = Marker.CUBE
        ground.action = Marker.ADD
        ground.pose.position.z = -0.1
        ground.pose.orientation.w = 1.0
        ground.scale.x = 50.0
        ground.scale.y = 50.0
        ground.scale.z = 0.01
        ground.color.a = 0.01  # nearly invisible
        ground.color.r = 0.5
        ground.color.g = 0.5
        ground.color.b = 0.5
        viz_arr.markers.append(ground)

        marker_id = 1

        # Completed polygons
        for i, poly in enumerate(self.polygons):
            vertices = poly["vertices"]
            is_occupied = poly["is_occupied"]

            # Fill (TRIANGLE_LIST) — viz only
            fill = self._make_fill_marker(vertices, marker_id, is_occupied)
            marker_id += 1
            if fill is not None:
                viz_arr.markers.append(fill)

            # Thick outline (LINE_STRIP) — both planner and viz
            outline = self._make_polygon_marker(
                vertices, marker_id, is_occupied, scale=0.08, alpha=0.8
            )
            marker_id += 1
            planner_arr.markers.append(outline)
            viz_arr.markers.append(outline)

            # Text label — viz only
            cx = sum(v[0] for v in vertices) / len(vertices)
            cy = sum(v[1] for v in vertices) / len(vertices)
            label = self._make_text_marker(
                str(i + 1), cx, cy, marker_id, is_occupied
            )
            marker_id += 1
            viz_arr.markers.append(label)

        # In-progress polygon: thin outline — viz only (NOT sent to planner)
        if self.current_polygon:
            preview = self._make_polygon_marker(
                self.current_polygon,
                marker_id,
                (self.mode == "obstacle"),
                scale=0.03,
                alpha=0.3,
            )
            marker_id += 1
            viz_arr.markers.append(preview)

        # Vertex spheres — viz only
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
            pt.pose.position.z = 0.05
            pt.pose.orientation.w = 1.0
            pt.scale.x = 0.15
            pt.scale.y = 0.15
            pt.scale.z = 0.15
            pt.color.r = 1.0 if self.mode == "obstacle" else 0.0
            pt.color.g = 0.0 if self.mode == "obstacle" else 1.0
            pt.color.b = 0.0
            pt.color.a = 0.9
            viz_arr.markers.append(pt)

        self.polygon_pub.publish(planner_arr)
        self.viz_pub.publish(viz_arr)

    def _make_polygon_marker(self, vertices, idx, is_occupied, scale=0.05, alpha=0.5):
        """Create a LINE_STRIP marker for a polygon contour."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "polygon_outline"
        marker.id = idx
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = scale
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

    def _make_fill_marker(self, vertices, idx, is_occupied):
        """Create a filled TRIANGLE_LIST marker (fan from centroid)."""
        if len(vertices) < 3:
            return None

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "polygon_fill"
        marker.id = idx
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD
        marker.pose.position.z = -0.01  # slightly below outline
        marker.color.a = 0.25

        if is_occupied:
            marker.color.r = 1.0
            marker.color.g = 0.0
        else:
            marker.color.r = 0.0
            marker.color.g = 1.0
        marker.color.b = 0.0

        # Compute centroid for fan triangulation
        cx = sum(v[0] for v in vertices) / len(vertices)
        cy = sum(v[1] for v in vertices) / len(vertices)

        for i in range(len(vertices)):
            j = (i + 1) % len(vertices)
            x1, y1 = vertices[i]
            x2, y2 = vertices[j]
            marker.points.append(Point(x=cx, y=cy, z=0.0))
            marker.points.append(Point(x=x1, y=y1, z=0.0))
            marker.points.append(Point(x=x2, y=y2, z=0.0))

        return marker

    def _make_text_marker(self, text, x, y, idx, is_occupied):
        """Create a TEXT_VIEW_FACING marker for polygon label."""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "polygon_label"
        marker.id = idx
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.1
        marker.scale.z = 0.4  # text height
        marker.text = text
        marker.color.a = 0.9
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0

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
