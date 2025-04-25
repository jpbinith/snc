#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
import time
from math import sin, cos

class MazeEscapeNode(Node):

    def __init__(self):
        super().__init__('maze_escape_node')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.trigger_sub = self.create_subscription(Empty, '/trigger_start', self.trigger_callback, 10)

        self.path_pub = self.create_publisher(Path, '/path_explore', 10)
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'

        self.timer = self.create_timer(0.5, self.escape_maze)

        self.scan_data = None
        self.exploration_started = False

        # Simulated pose (for demonstration, use odometry or TF in real use)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

    def scan_callback(self, msg):
        self.scan_data = msg

    def trigger_callback(self, msg):
        self.get_logger().info("Exploration triggered!")
        self.exploration_started = True

    def stop(self):
        msg = Twist()
        self.cmd_vel_pub.publish(msg)

    def update_pose(self, linear_x, angular_z, duration):
        # Simple simulated odometry update
        self.yaw += angular_z * duration
        self.x += linear_x * cos(self.yaw) * duration
        self.y += linear_x * sin(self.yaw) * duration

        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0  # No orientation tracking for simplicity

        self.path_msg.poses.append(pose)
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path_msg)

    def move(self, linear_x=0.0, angular_z=0.0, duration=1.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(msg)
        time.sleep(duration)
        self.stop()
        self.update_pose(linear_x, angular_z, duration)

    def escape_maze(self):
        if not self.exploration_started:
            return

        if self.scan_data is None:
            self.get_logger().info("Waiting for LaserScan data...")
            return

        ranges = self.scan_data.ranges
        front = min(min(ranges[0:10] + ranges[-10:]), 10.0)
        left = min(ranges[80:100])
        right = min(ranges[260:280])

        self.get_logger().info(f"Front: {front:.2f}, Left: {left:.2f}, Right: {right:.2f}")

        if left < 0.5:
            self.move(angular_z=-0.5, duration=1.0)
        elif front < 0.5:
            self.move(linear_x=-0.36, duration=1.0)
            self.move(angular_z=-0.5, duration=1.0)
        elif right > 1.5:
            self.move(linear_x=0.2, duration=1.0)
            self.move(angular_z=0.5, duration=2.0)
        else:
            self.move(linear_x=0.2, duration=1.0)

def main(args=None):
    rclpy.init(args=args)
    node = MazeEscapeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
