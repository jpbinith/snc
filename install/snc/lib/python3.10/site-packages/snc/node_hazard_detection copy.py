#!/usr/bin/env python3

import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, PointStamped
from visualization_msgs.msg import Marker
from find_object_2d.msg import ObjectsStamped
from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import LaserScan


class ObjectAndLaserSync(Node):

    def __init__(self):
        super().__init__('object_and_laser_sync')

        # Subscribers for object, laser, and depth image messages
        self.objects_sub = self.create_subscription(ObjectsStamped, '/objectsStamped', self.objects_callback, 10)
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        
        # Updated depth topic
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 10)

        # Publisher for hazard locations (PointStamped)
        self.hazard_pub = self.create_publisher(PointStamped, '/hazard', 10)

        # Publisher for visualization markers
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)

        # TF2 listener to transform hazard locations to map frame
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.bridge = CvBridge()
        self.depth_image = None
        self.camera_info = None

    def objects_callback(self, msg):
        # Process each object in the message
        for i in range(0, len(msg.objects.data), 12):
            obj_id = msg.objects.data[i]
            self.get_logger().warn(f"object ID {obj_id} detected.")
            
            # Assuming the object is detected in the center of the depth image (this can be adjusted)
            x_2d = int(msg.objects.data[i+1])  # Get x-coordinate from object data
            y_2d = int(msg.objects.data[i+2])  # Get y-coordinate from object data

            if self.depth_image is not None and self.camera_info is not None:
                # Check image bounds
                if y_2d >= self.depth_image.shape[0] or x_2d >= self.depth_image.shape[1]:
                    self.get_logger().warn(f"Pixel coordinates out of bounds for object ID {obj_id}. Skipping.")
                    continue

                # Get depth value
                depth_value = self.depth_image[y_2d, x_2d]

                # Validate depth
                if depth_value == 0 or np.isnan(depth_value) or np.isinf(depth_value):
                    self.get_logger().warn(f"Invalid depth ({depth_value}) for object ID {obj_id}. Skipping.")
                    continue

                # Camera intrinsics
                fx = self.camera_info.k[0]
                fy = self.camera_info.k[4]
                cx = self.camera_info.k[2]
                cy = self.camera_info.k[5]

                # Validate intrinsics
                if fx == 0 or fy == 0:
                    self.get_logger().warn(f"Invalid camera intrinsics (fx: {fx}, fy: {fy}). Skipping.")
                    continue

                # Compute 3D coordinates
                x_3d = (x_2d - cx) * depth_value / fx
                y_3d = (y_2d - cy) * depth_value / fy
                z_3d = depth_value

                # Prepare the hazard location message (PointStamped)
                hazard_local = PointStamped()
                hazard_local.header.stamp = msg.header.stamp
                hazard_local.header.frame_id = 'camera_color_optical_frame'
                hazard_local.point.x = x_3d
                hazard_local.point.y = y_3d
                hazard_local.point.z = z_3d

                try:
                    # Check if the transform is available
                    if self.tf_buffer.can_transform(
                        'map',
                        hazard_local.header.frame_id,
                        hazard_local.header.stamp,
                        timeout=rclpy.duration.Duration(seconds=0.5)
                    ):
                        transform = self.tf_buffer.lookup_transform(
                            'map',
                            hazard_local.header.frame_id,
                            hazard_local.header.stamp,
                            timeout=rclpy.duration.Duration(seconds=0.5)
                        )

                        # Transform point
                        hazard_global = tf2_geometry_msgs.do_transform_point(hazard_local, transform)

                        # Publish transformed hazard and marker (same as before)
                        self.hazard_pub.publish(hazard_global)

                        marker = Marker()
                        marker.header.frame_id = 'map'
                        marker.header.stamp = msg.header.stamp
                        marker.ns = 'hazards'
                        marker.id = int(obj_id)
                        marker.type = Marker.SPHERE
                        marker.action = Marker.ADD
                        marker.pose.position.x = hazard_global.point.x
                        marker.pose.position.y = hazard_global.point.y
                        marker.pose.position.z = hazard_global.point.z
                        marker.scale.x = marker.scale.y = marker.scale.z = 0.2
                        marker.color.a = 1.0
                        marker.color.r = 1.0
                        marker.color.g = 0.0
                        marker.color.b = 0.0

                        self.marker_pub.publish(marker)

                        self.get_logger().info(
                            f'Hazard (ID {obj_id}) at map position x={hazard_global.point.x:.2f}, y={hazard_global.point.y:.2f}, z={hazard_global.point.z:.2f}'
                        )
                    else:
                        self.get_logger().warn(f"Transform not available yet for object ID {obj_id}. Skipping this frame.")
                except Exception as e:
                    self.get_logger().warn(f'Could not transform to map frame for object ID {obj_id}: {e}')


    def laser_callback(self, msg):
        # Handle laser scan data (if needed)
        pass

    def depth_callback(self, msg):
    
        try:
            # Convert depth image to numpy array
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
            # Assuming 'depth_image' is your NumPy array representing the depth image
            height, width = self.depth_image.shape[:2]
            print(f"Depth image dimensions: width={width}, height={height}")
            # Check the depth image size (just as a sanity check)
            if self.depth_image is None:
                self.get_logger().warn("Received an empty depth image.")
                return
            
            # Log some basic statistics about the depth image
            min_depth = np.min(self.depth_image)
            max_depth = np.max(self.depth_image)
            mean_depth = np.mean(self.depth_image)
            std_depth = np.std(self.depth_image)
            self.get_logger().info(f"Depth image stats - min: {min_depth}, max: {max_depth}, mean: {mean_depth}, std: {std_depth}")
            
            # Check for NaN, Inf, or invalid depth values (e.g., values that are out of range)
            if np.any(np.isnan(self.depth_image)) or np.any(np.isinf(self.depth_image)):
                self.get_logger().warn("Depth image contains NaN or Inf values.")
                # Optional: You can replace NaN/Inf values with a default value, like 0 or the mean depth
                self.depth_image = np.nan_to_num(self.depth_image, nan=0.0, posinf=0.0, neginf=0.0)

            # Check if the depth image dimensions match expected camera resolution
            if (self.depth_image.shape[0] != self.camera_info.height or 
                self.depth_image.shape[1] != self.camera_info.width):
                self.get_logger().warn(f"Depth image dimensions mismatch: expected ({self.camera_info.height}, {self.camera_info.width}), got {self.depth_image.shape}")
        
        except Exception as e:
            self.get_logger().error(f"Error in depth_callback: {e}")


    def camera_info_callback(self, msg):
        # Store camera info
        self.camera_info = msg


def main(args=None):
    rclpy.init(args=args)
    node = ObjectAndLaserSync()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
