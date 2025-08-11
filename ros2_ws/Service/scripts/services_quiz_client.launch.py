from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='services_quiz',
            executable='turn_s_client_executable',
            name='turn_s_client'
        )
    ])
