import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Get the directory of the package
    drone_libcanard_test_pkg_share = get_package_share_directory('ros2_libcanard_test')

    # Define the path to the configuration file
    config_file_path = os.path.join(drone_libcanard_test_pkg_share, 'config', 'rc_config.yaml')
    print(f"Using configuration file: {config_file_path}")

    # Create the Node action for the esc_test_node
    esc_test_node = Node(
        package='ros2_libcanard_test',
        executable='esc_test_node',
        name='esc_test_node',
        output='screen',
        parameters=[config_file_path,
                    {'use_sim_time': False}]
    )

    # Create and return the LaunchDescription
    return LaunchDescription([
        esc_test_node
    ])