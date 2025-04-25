from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='snc',  # Replace with the actual package name if different
            executable='track_home',  # Replace with your entry point if it's different
            name='track_home_node',
            output='screen',
            parameters=[],
        ),
    ])
