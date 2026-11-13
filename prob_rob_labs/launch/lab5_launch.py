import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='set to true for simulation'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('prob_rob_labs'),
                    'launch', 
                    'turtlebot3_among_landmarks_launch.py'
                )
            ),
        ),
        # Node(
        #     package='prob_rob_labs',
        #     executable='landmark_center_processor',
        #     name='landmark_center_processor',
        #     parameters=[
        #         {'use_sim_time': LaunchConfiguration('use_sim_time')},
        #     ],
        #     remappings=[],
        # ),
        Node(
            package='prob_rob_labs',
            executable='landmark_measurement_processor',
            name='landmark_measurement_processor',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            remappings=[],
        ),
        Node(
            package='prob_rob_labs',
            executable='measurement_error_logger',
            name='measurement_error_logger',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            remappings=[],
        ),
    ])
