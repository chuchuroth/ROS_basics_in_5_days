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
            description='Log level for the service client node'
        ),
        
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if running in simulation'
        ),
        
        # Log launch information
        LogInfo(msg=['Launching Turn Service Client...']),
        
        # Turn Service Client Node
        Node(
            package='services_quiz',
            executable='turn_s_client',
            name='turn_service_client',
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
        LogInfo(msg=['Turn Service Client launched successfully!']),
    ])
