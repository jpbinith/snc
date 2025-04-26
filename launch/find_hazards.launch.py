from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():

    image_topic = '/camera/color/image_raw'
    image_topic_repeat = image_topic + '/repeat'
    use_compressed = 'false'  # Use raw image mode

    camera_info_topic = '/camera/color/camera_info'
    camera_info_topic_repeat = camera_info_topic + '/repeat'

    depth_topic = '/camera/depth/image_raw'
    depth_topic_repeat = depth_topic + '/repeat'

    #snc_launch_dir = os.path.join(get_package_share_directory('snc'), 'launch')


    return LaunchDescription([

        SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '0'),

       # Declare shared launch arguments
        DeclareLaunchArgument('gui', default_value='true', description='Launch GUI.'),
        DeclareLaunchArgument('image_topic', default_value=image_topic),
        DeclareLaunchArgument('image_topic_repeat', default_value=image_topic_repeat),
        DeclareLaunchArgument('use_compressed', default_value=use_compressed),
        DeclareLaunchArgument('objects_path',
            default_value=[EnvironmentVariable(name='AIIL_CHECKOUT_DIR'), '/humble_workspace/src/snc/resource/hazards'],
            description='Path to hazard object images'),
        DeclareLaunchArgument('settings_path', default_value='~/.ros/find_object_2d.ini'),
        DeclareLaunchArgument('depth_topic', default_value=depth_topic),
        DeclareLaunchArgument('depth_topic_repeat', default_value=depth_topic_repeat),
        DeclareLaunchArgument('camera_info_topic', default_value=camera_info_topic),
        DeclareLaunchArgument('camera_info_topic_repeat', default_value=camera_info_topic_repeat),
         # Object detection (Find Object 2D)
        Node(
            package='find_object_2d',
            executable='find_object_2d',
            output='screen',
            parameters=[{
                'subscribe_depth': False,
                'gui': LaunchConfiguration('gui'),
                'objects_path': LaunchConfiguration('objects_path'),
                'settings_path': LaunchConfiguration('settings_path'),
            }],
            remappings=[
                ('image', LaunchConfiguration('image_topic')),
                ('camera_info', LaunchConfiguration('camera_info_topic_repeat')),
            ]
        ),

        # Image repeater
        Node(
            package='aiil_rosbot_demo',
            executable='best_effort_repeater',
            name='best_effort_repeater',
            output='screen',
            parameters=[
                {'sub_topic_name': LaunchConfiguration('image_topic')},
                {'repeat_topic_name': LaunchConfiguration('image_topic_repeat')},
                {'use_compressed': LaunchConfiguration('use_compressed')},
            ]
        ),

        # Camera info repeater
        Node(
            package='aiil_rosbot_demo',
            executable='camera_info_best_effort_repeater',
            name='camera_info_best_effort_repeater',
            output='screen',
            parameters=[
                {'sub_topic_name': LaunchConfiguration('camera_info_topic')},
                {'repeat_topic_name': LaunchConfiguration('camera_info_topic_repeat')},
                {'use_compressed': LaunchConfiguration('use_compressed')},
            ]
        ),
    ])