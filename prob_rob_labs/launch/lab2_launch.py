import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_SPEED = 0.5

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='set to true for simulation'),
        DeclareLaunchArgument('speed', default_value=str(DEFAULT_SPEED),
                              description='set the speed for the lab2 node'),
        Node(
            package='prob_rob_labs',
            executable='lab2',
            name='lab2',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'speed': LaunchConfiguration('speed')},
            ],
            remappings=[
                ('door_torque', '/hinged_glass_door/torque'),
                ('cmd_vel', '/cmd_vel')
            ],
        )
    ])
