from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction


def generate_launch_description():
    return LaunchDescription([
        # 启动相机节点（MindVision）
        Node(
            package='ros2-mindvision-camera',
            executable='mv_camera_node',
            name='mv_camera_node',
            output='screen'
        ),
        # 启动装甲板识别节点
        Node(
            package='armor_detector',
            executable='detector_node',
            name='armor_detector',
            output='screen'
        ),
        # 启动弹道解算节点
        Node(
            package='ballistic_solver',
            executable='ballistic_solver',
            name='ballistic_solver',
            output='screen'
        ),
    ])