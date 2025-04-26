import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from find_object_2d.msg import ObjectsStamped
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from message_filters import ApproximateTimeSynchronizer, Subscriber
import tf2_ros
import tf2_geometry_msgs
import math
from tf_transformations import euler_from_quaternion

class ObjectAndLaserSync(Node):
    def __init__(self):
        super().__init__('object_and_laser_sync')

        # Subscribers
        self.objects_sub = Subscriber(self, ObjectsStamped, '/objectsStamped')
        self.laser_sub = Subscriber(self, LaserScan, '/scan')

        # Publishers
        self.hazard_pub = self.create_publisher(PointStamped, '/hazard', 10)
        self.marker_pub = self.create_publisher(Marker, '/hazards', 10)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Hazard memory
        self.detected_ids = set()

        # Synchronizer
        self.ts = ApproximateTimeSynchronizer(
            [self.objects_sub, self.laser_sub], queue_size=10, slop=0.1
        )
        self.ts.registerCallback(self.synced_callback)

    def get_yaw(self, q):
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw

    def synced_callback(self, objects_msg, laser_msg):
        timestamp = objects_msg.header.stamp

        for i in range(0, len(objects_msg.objects.data), 12):
            obj_id = objects_msg.objects.data[i]

            if obj_id in self.detected_ids:
                continue

            #center_index = len(laser_msg.ranges) // 2
            distance = laser_msg.ranges[0]

            if distance == float('inf'):
                self.get_logger().warn(f"Laser returned inf for object ID {obj_id}. Skipping.")
                continue

            try:
                # Lookup transform from base_link to map
                transform = self.tf_buffer.lookup_transform(
                                'map',
                                'base_link',
                                rclpy.time.Time(),
                                timeout=rclpy.duration.Duration(seconds=0.5)
                            )

                yaw = self.get_yaw(transform.transform.rotation)

                # Apply transform to laser data (local x only, assuming y_local = 0)
                x_map = transform.transform.translation.x + distance * math.cos(yaw)
                y_map = transform.transform.translation.y + distance * math.sin(yaw)

                # Mark as detected
                self.detected_ids.add(obj_id)

                # Publish hazard location
                hazard_global = PointStamped()
                hazard_global.header.stamp = timestamp
                hazard_global.header.frame_id = 'map'
                hazard_global.point.x = x_map
                hazard_global.point.y = y_map
                hazard_global.point.z = 0.0

                self.hazard_pub.publish(hazard_global)

                # Publish marker
                marker = Marker()
                marker.header.frame_id = 'map'
                marker.header.stamp = timestamp
                marker.ns = 'hazards'
                marker.id = int(obj_id)
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = x_map
                marker.pose.position.y = y_map
                marker.pose.position.z = 0.1
                marker.scale.x = marker.scale.y = marker.scale.z = 0.2
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0

                self.marker_pub.publish(marker)

                self.get_logger().info(
                    f'Hazard (ID {obj_id}) at map position x={x_map:.2f}, y={y_map:.2f}'
                )

            except Exception as e:
                self.get_logger().warn(f'Could not transform for object ID {obj_id}: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ObjectAndLaserSync()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
