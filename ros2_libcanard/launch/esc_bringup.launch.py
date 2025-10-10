from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    esc_bringup_node = Node(
        package='ros2_libcanard',
        executable='ros2_libcanard_node',
        name='uav',
        output='screen',
        emulate_tty=True,
        parameters=[
            {"interface_name": "slcan0"},
            {"num_esc": 1}
        ]
    )

    return LaunchDescription([
        esc_bringup_node
    ])