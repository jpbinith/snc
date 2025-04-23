from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 1) Launch argument to control simulation time
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulated clock if true'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    # 2) Paths to external launch/param files
    pkg_nav2 = get_package_share_directory('nav2_bringup')
    nav2_launch_file = os.path.join(pkg_nav2, 'launch', 'navigation_launch.py')
    nav2_params_file = os.path.join(
        get_package_share_directory('snc'), 'config', 'nav2_params.yaml'
    )

    slam_params_file = os.path.join(
        get_package_share_directory('snc'), 'config', 'slam_toolbox_params.yaml'
    )

    # 3) SLAM Toolbox node (mapping)
    slam_node = Node(
        package='slam_toolbox',
        executable='sync_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            slam_params_file,
        ]
    )

    # 4) Nav2 bringup in SLAM mode
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch_file),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam': 'True',              # run SLAM rather than AMCL
            'autostart': 'True',         # auto-start lifecycle nodes
            'params_file': nav2_params_file
        }.items()
    )

    # 5) Exploration trigger node
    trigger_node = Node(
        package='snc',
        executable='trigger_explore',
        name='exploration_trigger',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        slam_node,
        nav2_launch,
        trigger_node,
    ])
