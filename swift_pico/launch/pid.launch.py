from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='swift_pico',
            executable='pico_controller.py',  # Do not include the .py extension
            name='pico_controller',
            output='screen',
        ),
        Node(
            package='swift_pico',
            executable='pid_auto_tuning.py',  # Do not include the .py extension
            name='pid_auto_tuning',
            output='screen',
        ),
    ])
