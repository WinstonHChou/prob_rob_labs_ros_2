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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('prob_rob_labs'),
                    'launch', 
                    'turtlebot3_empty_launch.py'
                )
            ),
        ),
        Node(
            package='prob_rob_labs',
            executable='twist_calibration',
            name='twist_calibration',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            remappings=[
                ('gt_twist', '/tb3/ground_truth/twist'),
            ],
        ),
        # Node(
        #     package='prob_rob_labs',
        #     executable='door_estimator',
        #     name='door_estimator',
        #     parameters=[
        #         {'use_sim_time': LaunchConfiguration('use_sim_time')},
        #     ],
        #     remappings=[
        #         ('door_open', '/door_open'),
        #         ('feature_mean_x', '/feature_mean'),
        #         ('is_door_opened', '/is_door_opened'),
        #     ],
        # ),
        # Node(
        #     package='prob_rob_labs',
        #     executable='lab4',
        #     name='lab4',
        #     parameters=[
        #         {'use_sim_time': LaunchConfiguration('use_sim_time')},
        #         {'speed': LaunchConfiguration('speed')},
        #     ],
        #     remappings=[
        #         ('door_torque', '/hinged_glass_door/torque'),
        #         ('door_open', '/door_open'),
        #         ('twist', '/twist'),
        #         # ('feature_mean_x', '/feature_mean'),
        #     ],
        # )
    ])
