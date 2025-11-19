import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Create the Node action for the esc_test_node
    epos4_test_node = Node(
        package='ros2_libcanard_test',
        executable='epos4_test_node',
        name='epos4_test_node',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # Create and return the LaunchDescription
    return LaunchDescription([
        epos4_test_node
    ])