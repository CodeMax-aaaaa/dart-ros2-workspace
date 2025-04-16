import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 获取参数文件路径
    config_file = os.path.join(
        os.getenv('HOME'), 'dart-ros2-workspace', 'src', 'dart_detector', 'config', 'camera_lccv.yaml'
    )

    return LaunchDescription([
        # 启动 launcher_detector_node
        Node(
            package='dart_detector',
            executable='launcher_detector_node',
            name='launcher_detector_node',
            output='screen',
            parameters=[config_file]
        )
    ])