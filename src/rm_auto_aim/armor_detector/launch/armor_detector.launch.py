from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'color',
            default_value='blue',
            description='Target color for detection: red or blue'
        ),
        DeclareLaunchArgument(
            'debug',
            default_value='false',
            description='Enable debug mode'
        ),
        DeclareLaunchArgument(
            'param_file',
            default_value='',
            description='Full path to parameter YAML file'
        ),

        Node(
            package='armor_detector',
            executable='armor_detector_node',
            name='armor_detector',
            output='screen',
            parameters=[
                LaunchConfiguration('param_file'),
                {'color': LaunchConfiguration('color')},
                {'debug': LaunchConfiguration('debug')}
            ]
        )
    ])
