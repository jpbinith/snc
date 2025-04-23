import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty
import subprocess
import os
from ament_index_python.packages import get_package_share_directory

class ExplorationTrigger(Node):
    def __init__(self):
        super().__init__('exploration_trigger')
        # Subscriber to listen for the start trigger
        self.create_subscription(Empty, '/trigger_start', self.trigger_callback, 10)
        self.get_logger().info("ExplorationTrigger node started, waiting for /trigger_start...")

        self.explore_launched = False  # flag to ensure explore_lite is launched only once
    
    def trigger_callback(self, msg):
        """Called when a message is received on /trigger_start."""
        if self.explore_launched:
            self.get_logger().warn("Explore is already running. Ignoring additional trigger.")
            return

        self.get_logger().info("Received start trigger! Launching explore_lite...")

        # Construct path to the explore_lite param file in our package
        pkg_share = get_package_share_directory('snc')
        params_file = os.path.join(pkg_share, 'config', 'explore_params.yaml')

        # Launch the explore_lite node as a separate process
        try:
            subprocess.Popen([
                'ros2', 'run', 'explore_lite', 'explore',
                '--ros-args', '--params-file', params_file
            ])
            self.explore_launched = True
            self.get_logger().info(f"'explore_lite' node launched with params: {params_file}")
        except Exception as e:
            self.get_logger().error(f"Failed to launch explore_lite node: {e}")
            return

        # (Optional) publish a status or perform additional setup if needed
        # e.g., publish to /snc_status that exploration started.

def main(args=None):
    rclpy.init(args=args)
    node = ExplorationTrigger()
    try:
        rclpy.spin(node)  # keep the node alive, waiting for triggers
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
