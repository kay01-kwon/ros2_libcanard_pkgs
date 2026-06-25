import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ros2_libcanard_pkg_share = get_package_share_directory('ros2_libcanard')

    config_file_path = os.path.join(ros2_libcanard_pkg_share, 'config', 'slcan_single_esc_config.yaml')
    print(f"Using configuration file: {config_file_path}")

    esc_bringup_node = Node(
        package='ros2_libcanard',
        executable='ros2_libcanard_node',
        name='uav',
        output='screen',
        emulate_tty=True,
        parameters=[config_file_path]
    )

    return LaunchDescription([
        esc_bringup_node
    ])
