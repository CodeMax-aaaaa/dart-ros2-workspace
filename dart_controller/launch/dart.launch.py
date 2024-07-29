# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    node_lvgl_ui = Node(package="dart_controller", executable="node_lvgl_ui")
    node_can_agent = Node(package="dart_controller", executable="node_can_agent")
    node_dart_config = Node(package="dart_controller", executable="node_dart_config")
    node_camera = Node(package="detect", executable="camera_node")
    node_detect = Node(package="detect", executable="detect_node")
    node_logger_dog = Node(package="dart_controller", executable="node_dart_logger_dog")
    # DOMAIN ID = 7
    os.environ["ROS_DOMAIN_ID"] = "7"
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription(
        [
            node_logger_dog,
            node_camera,
            node_dart_config,
            node_detect,
            node_lvgl_ui,
            node_can_agent,
        ]
    )
    # 返回让ROS2根据launch描述执行节点
    return launch_description
