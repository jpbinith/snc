import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from find_object_2d.msg import ObjectsStamped
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker
from message_filters import ApproximateTimeSynchronizer, Subscriber
import tf2_ros
import tf2_geometry_msgs


class ObjectAndLaserSync(Node):

    def __init__(self):
        super().__init__('object_and_laser_sync')

        # Subscribers for object and laser messages
        self.objects_sub = Subscriber(self, ObjectsStamped, '/objectsStamped')
        self.laser_sub = Subscriber(self, LaserScan, '/scan')

        # Publisher for hazard locations (PointStamped)
        self.hazard_pub = self.create_publisher(PointStamped, '/hazard', 10)

        # Publisher for visualization markers
        self.marker_pub = self.create_publisher(Marker, '/visualization_marker', 10)

        # TF2 listener to transform hazard locations to map frame
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Synchronize messages from object and laser topics
        self.ts = ApproximateTimeSynchronizer(
            [self.objects_sub, self.laser_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.synced_callback)

    def synced_callback(self, objects_msg, laser_msg):
        timestamp = objects_msg.header.stamp

        # Process each object in the message (based on the ID)
        for i in range(0, len(objects_msg.objects.data), 12):
            obj_id = objects_msg.objects.data[i]

            # Get laser scan data for the center (this can be adjusted if needed)
            center_index = len(laser_msg.ranges) // 2
            distance = laser_msg.ranges[center_index]

            if distance == float('inf'):
                self.get_logger().warn(f"Laser returned inf for object ID {obj_id}. Skipping.")
                continue

            # Local coordinates for hazard (assuming 2D environment)
            x_local = distance
            y_local = 0.0

            # Prepare the hazard location message (PointStamped)
            hazard_local = PointStamped()
            hazard_local.header.stamp = timestamp
            hazard_local.header.frame_id = 'base_link'
            hazard_local.point.x = x_local
            hazard_local.point.y = y_local
            hazard_local.point.z = 0.0

            try:
                # Transform the local hazard coordinates to the map frame
                hazard_global = self.tf_buffer.transform(
                    hazard_local,
                    'map',
                    timeout=rclpy.duration.Duration(seconds=0.5)
                )

                # Publish the transformed hazard location
                self.hazard_pub.publish(hazard_global)

                # Create a visualization marker (red sphere)
                marker = Marker()
                marker.header.frame_id = 'map'
                marker.header.stamp = timestamp
                marker.ns = 'hazards'
                marker.id = int(obj_id)  # Use object ID as the unique marker ID
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = hazard_global.point.x
                marker.pose.position.y = hazard_global.point.y
                marker.pose.position.z = 0.1  # Slightly above ground for visibility
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.2
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0

                # Publish the marker for RViz visualization
                self.marker_pub.publish(marker)

                # Log the position of the hazard in the map frame
                self.get_logger().info(
                    f'Hazard (ID {obj_id}) at map position x={hazard_global.point.x:.2f}, y={hazard_global.point.y:.2f}'
                )
            except Exception as e:
                self.get_logger().warn(f'Could not transform to map frame for object ID {obj_id}: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = ObjectAndLaserSync()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
