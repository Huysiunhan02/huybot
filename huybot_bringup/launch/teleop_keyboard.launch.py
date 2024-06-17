from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    teleop_node = Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard_node',
            output='screen',
            prefix = 'xterm -e',
         )

    return LaunchDescription([
        teleop_node,
    ])