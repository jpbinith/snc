from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='snc',  
            executable='track_home',  
            name='track_home_node',
            output='screen',
            parameters=[],
        ),
    ])
