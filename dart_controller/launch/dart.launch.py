# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    node_lvgl_ui = Node(
        package="dart_controller",
        executable="node_lvgl_ui"
    )
    node_can_agent = Node(
        package="dart_controller",
        executable="node_can_agent"
    )
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription(
        [node_lvgl_ui, node_can_agent])
    # 返回让ROS2根据launch描述执行节点
    return launch_description
