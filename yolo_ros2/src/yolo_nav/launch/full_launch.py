#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # === パッケージパス ===
    pkg_tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_yolo_nav   = get_package_share_directory('yolo_nav')

    # === パラメータ ===
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-4.0')
    y_pose = LaunchConfiguration('y_pose', default='2.0')

    # === 使いたい自作ワールド（my_world） ===
    world_file = os.path.join(pkg_yolo_nav, 'my_worlds', 'mixed_room.world')
    # 必要に応じて ↑ を他の .world に差し替え

    # === Waffle Pi を明示 ===
    set_tb3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'waffle_pi')

    # === モデル/リソースの検索パス（my_worlds を恒久追加） ===
    share_root   = os.path.join(pkg_yolo_nav, 'my_worlds')
    share_models = os.path.join(share_root, 'models')

    prev_model = os.environ.get('GAZEBO_MODEL_PATH', '')
    prev_res   = os.environ.get('GAZEBO_RESOURCE_PATH', '')

    new_model_path = share_models if not prev_model else f"{prev_model}:{share_models}"
    new_res_path   = share_root   if not prev_res   else f"{prev_res}:{share_root}"

    set_gazebo_model_path    = SetEnvironmentVariable('GAZEBO_MODEL_PATH', new_model_path)
    set_gazebo_resource_path = SetEnvironmentVariable('GAZEBO_RESOURCE_PATH', new_res_path)

    # Fuel への自動取得を無効化（任意：名前違いで待たされるのを防ぐ）
    disable_fuel = SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', '')

    # === Gazebo を一括起動（Factory プラグイン含む） ===
    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # === TB3 の状態公開 ===
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # === TB3 のスポーン ===
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb3_gazebo, 'launch', 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose
            # TURTLEBOT3_MODEL=waffle_pi を使うので model 引数は不要
        }.items()
    )

    # === LaunchDescription ===
    ld = LaunchDescription()

    # Gazebo 起動前に環境変数を適用（ここ重要！）
    ld.add_action(set_tb3_model)
    ld.add_action(set_gazebo_model_path)
    ld.add_action(set_gazebo_resource_path)
    ld.add_action(disable_fuel)

    ld.add_action(gazebo_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)

    return ld
