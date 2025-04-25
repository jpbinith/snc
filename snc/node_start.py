import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
from find_object_2d.msg import ObjectsStamped  # Make sure this message is available

class ObjectCmdVel(Node):

    def __init__(self):
        super().__init__('object_cmd_vel_node')

        # Subscribe to the objectsStamped topic
        self.subscription = self.create_subscription(
            ObjectsStamped,
            '/objectsStamped',
            self.object_callback,
            10
        )

        # Publisher to trigger_start
        self.trigger_start_publisher = self.create_publisher(Empty, '/trigger_start', 10)

        # Flag to track if message was already published
        self.triggered = False

    def object_callback(self, msg):
        if self.triggered:
            return  # Do nothing if already triggered

        for obj_id in msg.objects.data[::12]:  # IDs are every 12th value
            if obj_id == 1:
                self.get_logger().info('Start detected! Publishing to /trigger_start...')
                empty_msg = Empty()
                self.trigger_start_publisher.publish(empty_msg)
                self.triggered = True  # Set flag so it doesn't publish again
                break

def main(args=None):
    rclpy.init(args=args)
    node = ObjectCmdVel()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
