from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    gazebo_pkg = get_package_share_directory('gazebo_ros')
    yolo_nav_pkg = get_package_share_directory('yolo_nav')
    world_path = os.path.join(yolo_nav_pkg, 'my_worlds', 'mixed_room.world')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')
            ),
            launch_arguments={
                'world': world_path,
                'verbose': 'true',           # ログ出す
                'gui': 'true',
                'server_required': 'true'    # ←サーバ明示
            }.items()
        )
    ])
