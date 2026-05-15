#!/usr/bin/env python3
"""Test obstacle polygon → freespace pipeline."""

import sys, time, math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Point
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Path
from std_srvs.srv import Trigger


class ObstacleTester(Node):
    def __init__(self):
        super().__init__("obstacle_tester")
        self.start_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.poly_pub = self.create_publisher(MarkerArray, "/obstacle_polygons", 10)
        self.client = self.create_client(Trigger, "/planner_bridge_node/plan_path")
        self.path_sub = self.create_subscription(Path, "/trajectory_path", self.on_path, 10)
        self.latest_path = None

    def on_path(self, msg):
        self.latest_path = msg

    def publish_obstacle(self, vertices):
        arr = MarkerArray()
        m = Marker()
        m.header.frame_id = "map"
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "polygon_outline"
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.05
        m.color.r = 1.0
        m.color.a = 0.8
        for x, y in vertices:
            m.points.append(Point(x=float(x), y=float(y), z=0.0))
        m.points.append(Point(x=float(vertices[0][0]), y=float(vertices[0][1]), z=0.0))
        arr.markers.append(m)
        self.poly_pub.publish(arr)

    def clear_obstacles(self):
        self.poly_pub.publish(MarkerArray())

    def plan(self, start, goal):
        self.latest_path = None
        # Start
        sm = PoseWithCovarianceStamped()
        sm.header.frame_id = "map"
        sm.header.stamp = self.get_clock().now().to_msg()
        sm.pose.pose.position.x = float(start[0])
        sm.pose.pose.position.y = float(start[1])
        yaw = math.radians(start[2])
        sm.pose.pose.orientation.z = math.sin(yaw / 2.0)
        sm.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.start_pub.publish(sm)
        time.sleep(0.3)

        # Goal
        gm = PoseStamped()
        gm.header.frame_id = "map"
        gm.header.stamp = self.get_clock().now().to_msg()
        gm.pose.position.x = float(goal[0])
        gm.pose.position.y = float(goal[1])
        yaw = math.radians(goal[2])
        gm.pose.orientation.z = math.sin(yaw / 2.0)
        gm.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_pub.publish(gm)
        time.sleep(0.3)

        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service unavailable")
            return None

        fut = self.client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)

        if not fut.done():
            self.get_logger().error("Timeout")
            return None

        resp = fut.result()
        if not resp.success:
            self.get_logger().error(f"Planning failed: {resp.message}")
            return None

        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=1.0)
        return self.latest_path

    def analyze_path(self, path, goal, obstacle_box=None):
        if path is None or not path.poses:
            print("  -> No path")
            return
        last = path.poses[-1].pose
        dx = last.position.x - goal[0]
        dy = last.position.y - goal[1]
        dist = math.sqrt(dx * dx + dy * dy)
        print(f"  -> Path: {len(path.poses)} pts, last=({last.position.x:.2f},{last.position.y:.2f}), dist_to_goal={dist:.2f}m")

        if obstacle_box:
            min_x, max_x, min_y, max_y = obstacle_box
            in_zone = False
            for p in path.poses:
                if min_x < p.pose.position.x < max_x and min_y < p.pose.position.y < max_y:
                    in_zone = True
                    break
            if in_zone:
                print(f"  -> FAIL: path goes through obstacle zone!")
            else:
                print(f"  -> OK: path avoids obstacle zone")


def main():
    rclpy.init()
    tester = ObstacleTester()
    time.sleep(2.0)

    obstacle_vertices = [(4, -2), (7, -2), (7, 2), (4, 2)]
    obstacle_box = (4, 7, -2, 2)

    # Test 1: No obstacle
    print("=== Test 1: No obstacle ===")
    tester.clear_obstacles()
    path = tester.plan((0, 0, 0), (10, 0, 0))
    tester.analyze_path(path, (10, 0, 0))

    # Test 2: With obstacle
    print("\n=== Test 2: Obstacle at x=[4,7], y=[-2,2] ===")
    tester.publish_obstacle(obstacle_vertices)
    time.sleep(0.5)
    path = tester.plan((0, 0, 0), (10, 0, 0))
    tester.analyze_path(path, (10, 0, 0), obstacle_box)

    # Test 3: Larger obstacle, forcing detour
    print("\n=== Test 3: Large obstacle x=[3,8], y=[-4,4], forced detour ===")
    tester.publish_obstacle([(3, -4), (8, -4), (8, 4), (3, 4)])
    time.sleep(0.5)
    path = tester.plan((0, 0, 0), (10, 0, 0))
    tester.analyze_path(path, (10, 0, 0), (3, 8, -4, 4))

    tester.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
