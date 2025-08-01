import os
import sys
from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('rm_vision_bringup'), 'launch'))

def generate_launch_description():
    from common import node_params, launch_params, robot_state_publisher, tracker_node
    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer, Node
    from launch.actions import TimerAction, Shutdown
    from launch import LaunchDescription

    def get_camera_node(package, plugin):
        return ComposableNode(
            package=package,
            plugin=plugin,
            name='camera_node',
            parameters=[node_params],
            extra_arguments=[{'use_intra_process_comms': True}]
        )

    def get_camera_detector_container(camera_node):
        return ComposableNodeContainer(
            name='camera_detector_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                camera_node,
                ComposableNode(
                    package='armor_detector',
                    plugin='rm_auto_aim::ArmorDetectorNode',
                    name='armor_detector',
                    parameters=[node_params],
                    extra_arguments=[{'use_intra_process_comms': True}]
                )
            ],
            output='both',
            emulate_tty=True,
            ros_arguments=['--ros-args', '--log-level',
                           'armor_detector:='+launch_params['detector_log_level']],
            on_exit=Shutdown(),
        )

    # 弹道解算配置文件路径
    ballistic_config_path = os.path.join(
        get_package_share_directory('rm_vision_bringup'), 
        'config', 
        'ballistic_solver.yaml'
    )
    
    # 弹道解算节点参数合并
    ballistic_params = [node_params]  # 使用通用参数
    if os.path.exists(ballistic_config_path):
        ballistic_params.append(ballistic_config_path)  # 添加专用配置
    
    # 弹道解算节点
    ballistic_solver_node = Node(
        package='ballistic_solver',
        executable='ballistic_solver_node',
        name='ballistic_solver',
        output='screen',
        emulate_tty=True,
        parameters=ballistic_params,
        ros_arguments=['--ros-args', '--log-level',
                      'ballistic_solver:='+launch_params.get('ballistic_log_level', 'info')],
        respawn=True,           # 自动重启
        respawn_delay=2.0,      # 重启延迟
    )

    # 相机节点选择
    hik_camera_node = get_camera_node('hik_camera', 'hik_camera::HikCameraNode')
    mv_camera_node = get_camera_node('mindvision_camera', 'mindvision_camera::MVCameraNode')
    
    if launch_params['camera'] == 'mv':
        cam_detector = get_camera_detector_container(mv_camera_node)
    elif launch_params['camera'] == 'hik':
        cam_detector = get_camera_detector_container(hik_camera_node)

    # 跟踪器节点延迟启动
    # delay_tracker_node = TimerAction(
    #     period=2.0,
    #     actions=[tracker_node],
    # )

    # 弹道解算节点延迟启动（确保检测节点先启动）
    delay_ballistic_node = TimerAction(
        period=1.5,  # 在tracker之前启动
        actions=[ballistic_solver_node],
    )

    return LaunchDescription([
        robot_state_publisher,
        cam_detector,
        delay_ballistic_node,    # 弹道解算先启动
        # delay_tracker_node,      # 跟踪器后启动
    ])