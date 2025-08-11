from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='services_quiz',
            executable='turn_s_server_executable',
            name='turn_s_server'
        )
    ])
        
        # Log successful launch
        LogInfo(msg=['Turn Service Server launched successfully!']),
    ])
