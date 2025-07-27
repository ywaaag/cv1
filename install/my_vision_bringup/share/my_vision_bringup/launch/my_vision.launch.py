import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    # 获取参数文件路径
    node_params = os.path.join(
        get_package_share_directory('rm_vision_bringup'), 'config', 'node_params.yaml'
    )
    
    # 定义 MindVision 相机节点
    mv_camera_node = ComposableNode(
        package='mindvision_camera',
        plugin='mindvision_camera::MVCameraNode',
        name='camera_node',
        parameters=[node_params],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # 定义识别节点 (Armor Detector)
    armor_detector_node = ComposableNode(
        package='armor_detector',
        plugin='rm_auto_aim::ArmorDetectorNode',
        name='armor_detector',
        parameters=[node_params],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    # 定义一个容器来高效地运行相机和识别节点
    camera_detector_container = ComposableNodeContainer(
        name='camera_detector_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            mv_camera_node,
            armor_detector_node
        ],
        output='screen',
        emulate_tty=True,
    )

    # 定义弹道解算节点
    ballistic_solver_node = Node(
        package='ballistic_solver',
        executable='ballistic_solver_node',
        name='ballistic_solver',
        output='screen',
        emulate_tty=True,
        parameters=[node_params],
    )

    return LaunchDescription([
        camera_detector_container,
        ballistic_solver_node,
    ])