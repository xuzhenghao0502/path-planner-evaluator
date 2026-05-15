#!/usr/bin/env python3
"""Automated planner test: 5 start/goal pairs, no obstacles."""

import sys
import time
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Pose
from nav_msgs.msg import Path
from std_srvs.srv import Trigger


def make_pose(x, y, yaw_deg):
    yaw = math.radians(yaw_deg)
    p = Pose()
    p.position.x = float(x)
    p.position.y = float(y)
    p.position.z = 0.0
    p.orientation.z = math.sin(yaw / 2.0)
    p.orientation.w = math.cos(yaw / 2.0)
    return p


# Test cases: label, start, goal
TEST_CASES = [
    ("1. Straight line",       make_pose(0, 0, 0),   make_pose(10, 0, 0)),
    ("2. Simple turn 90deg",   make_pose(0, 0, 0),   make_pose(5, 5, 90)),
    ("3. Shift left",          make_pose(0, 0, 0),   make_pose(8, 2, 0)),
    ("4. Parallel offset",     make_pose(0, 0, 90),  make_pose(3, 3, 90)),
    ("5. Long diagonal",       make_pose(-5, -5, 45),make_pose(5, 5, -45)),
]


class PlannerTester(Node):
    def __init__(self):
        super().__init__("planner_tester")
        self.start_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.plan_client = self.create_client(Trigger, "/planner_bridge_node/plan_path")
        self.path_sub = self.create_subscription(Path, "/trajectory_path", self.on_path, 10)
        self.latest_path = None
        self.results = []

    def on_path(self, msg):
        self.latest_path = msg

    def run_test(self, label, start_pose, goal_pose):
        self.get_logger().info(f"\n{'='*60}\n  {label}\n  Start: ({start_pose.position.x:.1f}, {start_pose.position.y:.1f})"
                              f"  Goal: ({goal_pose.position.x:.1f}, {goal_pose.position.y:.1f})\n{'='*60}")
        self.latest_path = None

        # Publish start (PoseWithCovarianceStamped to /initialpose)
        start_msg = PoseWithCovarianceStamped()
        start_msg.header.frame_id = "map"
        start_msg.header.stamp = self.get_clock().now().to_msg()
        start_msg.pose.pose = start_pose
        self.start_pub.publish(start_msg)
        time.sleep(0.3)

        # Publish goal
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose = goal_pose
        self.goal_pub.publish(goal_msg)
        time.sleep(0.3)

        # Wait for service
        if not self.plan_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Plan service not available")
            self.results.append((label, False, "Service unavailable"))
            return

        req = Trigger.Request()
        future = self.plan_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

        if not future.done():
            self.get_logger().error("Plan service timed out")
            self.results.append((label, False, "Timeout"))
            return

        resp = future.result()
        if not resp.success:
            self.get_logger().error(f"Planning FAILED: {resp.message}")
            self.results.append((label, False, resp.message))
            return

        # Wait a bit for path message
        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=1.0)

        if self.latest_path is None or not self.latest_path.poses:
            self.get_logger().error("No trajectory received")
            self.results.append((label, False, "No trajectory"))
            return

        # Check last point vs goal
        last = self.latest_path.poses[-1].pose
        dx = last.position.x - goal_pose.position.x
        dy = last.position.y - goal_pose.position.y
        dist = math.sqrt(dx * dx + dy * dy)

        n = len(self.latest_path.poses)
        self.get_logger().info(f"Trajectory: {n} points, last=({last.position.x:.2f}, {last.position.y:.2f}), "
                              f"dist_to_goal={dist:.2f}m")

        if dist < 1.0:
            self.get_logger().info(f"  => PASS (dist={dist:.2f}m < 1.0m)")
            self.results.append((label, True, f"OK dist={dist:.2f}m"))
        else:
            self.get_logger().warn(f"  => FAIL (dist={dist:.2f}m >= 1.0m, trajectory doesn't reach goal!)")
            # Print first 5 and last 5 points
            for i in range(min(5, n)):
                p = self.latest_path.poses[i].pose
                q = p.orientation
                yaw = math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
                self.get_logger().info(f"  [{i}] ({p.position.x:.2f}, {p.position.y:.2f}, {yaw:.1f} deg)")
            if n > 10:
                self.get_logger().info(f"  ... ({n-10} points omitted) ...")
            for i in range(max(5, n-5), n):
                p = self.latest_path.poses[i].pose
                q = p.orientation
                yaw = math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
                self.get_logger().info(f"  [{i}] ({p.position.x:.2f}, {p.position.y:.2f}, {yaw:.1f} deg)")
            self.results.append((label, False, f"dist={dist:.2f}m"))


def main():
    rclpy.init()
    tester = PlannerTester()

    # Give the system time to stabilize
    time.sleep(2.0)

    for label, start, goal in TEST_CASES:
        tester.run_test(label, start, goal)
        time.sleep(1.0)

    # Summary
    print("\n" + "=" * 60)
    print("  RESULTS SUMMARY")
    print("=" * 60)
    for label, ok, msg in tester.results:
        status = "PASS" if ok else "FAIL"
        print(f"  [{status}] {label}: {msg}")
    print("=" * 60)

    tester.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
