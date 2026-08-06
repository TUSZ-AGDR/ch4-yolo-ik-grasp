"""
Launch: 仅启动 YOLO 检测节点

用法: ros2 launch yolo_vision detect.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='best.pt',
                              description='YOLO model file'),
        DeclareLaunchArgument('confidence', default_value='0.5',
                              description='Detection confidence threshold'),
        DeclareLaunchArgument('device', default_value='cuda:0',
                              description='Device: cuda:0 / cpu'),

        Node(
            package='yolo_vision',
            executable='detector',
            name='yolo_detector',
            output='screen',
            parameters=[{
                'model': LaunchConfiguration('model'),
                'confidence': LaunchConfiguration('confidence'),
                'device': LaunchConfiguration('device'),
            }],
        ),
    ])
