#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        
        # Declare launch arguments
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Log level for the service server node'
        ),
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if running in simulation'
        ),
        
        # Log launch information
        LogInfo(msg=['Launching Turn Service Server...']),
        
        # Turn Service Server Node
        Node(
            package='services_quiz',
            executable='turn_s_server',
            name='turn_service_server',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            arguments=[
                '--ros-args', 
                '--log-level', LaunchConfiguration('log_level')
            ]
        ),
        
        # Log successful launch
        LogInfo(msg=['Turn Service Server launched successfully!']),
    ])
