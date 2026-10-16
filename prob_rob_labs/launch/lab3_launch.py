import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_SPEED = 0.5

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='set to true for simulation'),
        DeclareLaunchArgument('speed', default_value=str(DEFAULT_SPEED),
                              description='set the speed for the lab3 node'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('prob_rob_labs'),
                    'launch', 
                    'turtlebot3_door_and_firetruck_launch.py'
                    # 'turtlebot3_and_door_launch.py'
                )
            ),
            launch_arguments={
                'run_vision_processor': str(True),
                'run_door_opener': str(True),
            }.items()
        ),
        # Node(
        #     package='prob_rob_labs',
        #     executable='door_calibration',
        #     name='door_calibration',
        #     parameters=[
        #         {'use_sim_time': LaunchConfiguration('use_sim_time')},
        #     ],
        #     remappings=[
        #         ('door_torque', '/hinged_glass_door/torque'),
        #         ('feature_mean_x', '/feature_mean'),
        #     ],
        # ),
        Node(
            package='prob_rob_labs',
            executable='door_estimator',
            name='door_estimator',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            remappings=[
                ('door_open', '/door_open'),
                ('feature_mean_x', '/feature_mean'),
                ('is_door_opened', '/is_door_opened'),
            ],
        ),
        Node(
            package='prob_rob_labs',
            executable='lab3',
            name='lab3',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
                {'speed': LaunchConfiguration('speed')},
            ],
            remappings=[
                ('door_torque', '/hinged_glass_door/torque'),
                ('door_open', '/door_open'),
                ('cmd_vel', '/cmd_vel'),
                # ('feature_mean_x', '/feature_mean'),
            ],
        )
    ])
