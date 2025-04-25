#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_ros import TransformException
from geometry_msgs.msg import PoseStamped, Point
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.duration import Duration
from action_msgs.msg import GoalStatus
from collections import deque
import tf_transformations
import math
from nav_msgs.msg import Path
from std_msgs.msg import Empty
from std_msgs.msg import String


class TrackHome(Node):
    def __init__(self):
        super().__init__('track_home_node')

        # Set up TF2 listener to receive transforms from 'base_link' to 'map'
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Set up Nav2 action client to send NavigateToPose goals
        self.nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Publishers for exploration and return path visualization
        self.exploration_path_pub = self.create_publisher(Path, 'exploration_path', 10)
        self.return_path_pub = self.create_publisher(Path, 'return_path', 10)

        self.status_pub = self.create_publisher(String, '/snc_status', 10)
        self.trigger_pub = self.create_publisher(Empty, '/trigger_home', 10)

        # Immediately publish 'exploring' state on startup
        status_msg = String()
        status_msg.data = "exploring"
        self.status_pub.publish(status_msg)
        self.get_logger().info("Status set to: exploring")

        self.status_sub = self.create_subscription(
            String,
            '/snc_status',
            self.status_callback,
            10
        )


        # Subscriber for manual return-to-home trigger
        self.trigger_sub = self.create_subscription(
            Empty,
            '/trigger_home',
            self.manual_return_home_callback,
            10
        )

        # Path message containers
        self.exploration_path_msg = Path()
        self.exploration_path_msg.header.frame_id = 'map'

        self.return_path_msg = Path()
        self.return_path_msg.header.frame_id = 'map'

        # --- CONFIGURABLE PARAMETERS ---
        self.min_distance = 0.1  # Minimum distance (meters) before saving a new waypoint
        exploration_duration_seconds = 180.0  # Max duration of exploration before auto-return
        # --------------------------------

        # Waypoint management and state
        self.waypoints = deque()
        self.tracking_active = True
        self.transform_ready = False
        self.returning_home = False
        self.current_goal_index = 0

        self.get_logger().info("Waypoint tracker started. Saving waypoints every 10s if moved > 0.1m.")
        self.get_logger().info("Waiting for transform between 'map' and 'base_link'. SLAM or localization must be active.")

        # Initial position saving timer
        self.create_timer(1.0, self.save_initial_position)

        # Periodic timers for tracking
        self.timer = self.create_timer(10.0, self.save_waypoint)  # Save waypoint every 10s
        self.position_logger_timer = self.create_timer(2.0, self.log_current_position)  # Log position every 2s
        self.timer_stop = self.create_timer(exploration_duration_seconds, self.start_return_home)

    def manual_return_home_callback(self, msg):
        if self.returning_home:
            self.get_logger().warn("Return to home already in progress. Ignoring manual trigger.")
            return
        self.get_logger().warn("/trigger_home received. Stopping exploration and initiating return.")
        self.start_return_home()

    def status_callback(self, msg):
        if msg.data.strip().lower() == 'returning' and not self.returning_home:
            self.get_logger().info("Received 'returning' status. Initiating return home sequence.")
            self.start_return_home()


    def get_current_transform(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(), timeout=Duration(seconds=2.0)
            )
            if not self.transform_ready:
                self.get_logger().info("Transform available. SLAM initialized.")
                self.transform_ready = True
            return transform
        except TransformException as e:
            self.get_logger().warn(f"Transform Error: {e}")
            return None

    def euclidean_distance(self, p1, p2):
        return math.sqrt((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2)

    def save_waypoint(self):
        if not self.tracking_active:
            return
        transform = self.get_current_transform()
        if not transform:
            return
        current = transform.transform.translation
        if not self.waypoints or self.euclidean_distance(current, self.waypoints[-1].transform.translation) >= self.min_distance:
            self.waypoints.append(transform)
            pose_stamped = self.transform_to_pose_stamped(transform)
            self.exploration_path_msg.header.stamp = self.get_clock().now().to_msg()
            self.exploration_path_msg.poses.append(pose_stamped)
            self.exploration_path_pub.publish(self.exploration_path_msg)
            self.get_logger().info(f"Waypoint saved: x={current.x:.2f}, y={current.y:.2f}")

    def start_return_home(self):
        self.timer_stop.cancel()
        self.tracking_active = False
        self.returning_home = True
        self.get_logger().info("Exploration complete. Stopping tracking and returning to home.")
        self.print_path_to_home()
        self.current_goal_index = 0
        # Publish status
        status_msg = String()
        status_msg.data = "returning"
        self.status_pub.publish(status_msg)
        self.get_logger().info("Status set to: returning")

        # Publish trigger to shut down wall-following node
        self.trigger_pub.publish(Empty())
        self.get_logger().info("Published return trigger to /trigger_home")

        self.send_next_goal()

    def print_path_to_home(self):
        self.get_logger().info("--- Path to Follow Home ---")
        for i, wp in enumerate(reversed(self.waypoints)):
            p = wp.transform.translation
            o = wp.transform.rotation
            self.get_logger().info(f"Waypoint {i + 1}: x={p.x:.2f}, y={p.y:.2f}, z={p.z:.2f}, orientation w={o.w:.2f}")
        self.get_logger().info("--- End of Path ---")

    def save_initial_position(self):
        if not self.transform_ready or self.waypoints:
            return
        transform = self.get_current_transform()
        if transform:
            self.waypoints.append(transform)
            p = transform.transform.translation
            pose_stamped = self.transform_to_pose_stamped(transform)
            self.exploration_path_msg.header.stamp = self.get_clock().now().to_msg()
            self.exploration_path_msg.poses.append(pose_stamped)
            self.exploration_path_pub.publish(self.exploration_path_msg)
            self.get_logger().info(f"Initial position saved: x={p.x:.2f}, y={p.y:.2f}")

    def transform_to_pose_stamped(self, transform):
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.position = Point(
            x=transform.transform.translation.x,
            y=transform.transform.translation.y,
            z=transform.transform.translation.z
        )
        pose.pose.orientation = transform.transform.rotation
        return pose

    def send_next_goal(self):
        if self.current_goal_index >= len(self.waypoints):
            self.get_logger().info("Robot successfully returned home. Shutting down node.")
            self.destroy_node()
            rclpy.shutdown()
            return

        reversed_waypoints = list(reversed(self.waypoints))
        transform = reversed_waypoints[self.current_goal_index]
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = 'map'

        goal_msg.pose.pose.position.x = transform.transform.translation.x
        goal_msg.pose.pose.position.y = transform.transform.translation.y
        goal_msg.pose.pose.position.z = 0.0

        # Determine orientation
        if self.current_goal_index == len(reversed_waypoints) - 1:
            goal_msg.pose.pose.orientation = transform.transform.rotation
        else:
            current_pos = transform.transform.translation
            next_pos = reversed_waypoints[self.current_goal_index + 1].transform.translation
            dx = next_pos.x - current_pos.x
            dy = next_pos.y - current_pos.y
            yaw = math.atan2(dy, dx)
            quat = tf_transformations.quaternion_from_euler(0, 0, yaw)
            goal_msg.pose.pose.orientation.x = quat[0]
            goal_msg.pose.pose.orientation.y = quat[1]
            goal_msg.pose.pose.orientation.z = quat[2]
            goal_msg.pose.pose.orientation.w = quat[3]

        self.get_logger().info(
            f"Sending goal {self.current_goal_index + 1} to x={goal_msg.pose.pose.position.x:.2f}, y={goal_msg.pose.pose.position.y:.2f}"
        )

        # Publish return path step
        pose_stamped = self.transform_to_pose_stamped(transform)
        self.return_path_msg.header.stamp = self.get_clock().now().to_msg()
        self.return_path_msg.poses.append(pose_stamped)
        self.return_path_pub.publish(self.return_path_msg)

        self.nav2_client.wait_for_server()
        self.nav2_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)\
            .add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        pass  # Feedback logging can be added here if needed

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f"Goal {self.current_goal_index + 1} was rejected.")
            return
        self.get_logger().info(f"Goal {self.current_goal_index + 1} accepted.")
        goal_handle.get_result_async().add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result()
        status = result.status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Goal {self.current_goal_index + 1} succeeded.")
        else:
            self.get_logger().warn(f"Goal {self.current_goal_index + 1} failed. Status: {status}")
            return  # Do not proceed if a goal fails

        self.current_goal_index += 1
        self.send_next_goal()

    def log_current_position(self):
        transform = self.get_current_transform()
        if transform:
            p = transform.transform.translation
            self.get_logger().info(f"Current position: x={p.x:.2f}, y={p.y:.2f}, z={p.z:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = TrackHome()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested via keyboard interrupt.")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
