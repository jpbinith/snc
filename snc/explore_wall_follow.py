#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time

class MazeEscapeNode(Node):

    def __init__(self):
        super().__init__('maze_escape_node')

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.trigger_sub = self.create_subscription(Empty, '/trigger_start', self.trigger_callback, 10)

        self.timer = self.create_timer(0.5, self.escape_maze)

        self.scan_data = None
        self.exploration_started = False

    def scan_callback(self, msg):
        self.scan_data = msg

    def trigger_callback(self, msg):
        self.get_logger().info("Exploration triggered!")
        self.exploration_started = True

    def stop(self):
        msg = Twist()
        self.cmd_vel_pub.publish(msg)

    def move(self, linear_x=0.0, angular_z=0.0, duration=1.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(msg)
        time.sleep(duration)
        self.stop()

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
