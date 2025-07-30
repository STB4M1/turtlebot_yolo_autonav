from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yolo_nav',
            executable='yolo_node',
            name='yolo_detector',
            output='screen'
        ),
        Node(
            package='yolo_nav',
            executable='obstacle_avoidance_node',
            name='obstacle_avoider',
            output='screen'
        )
    ])
