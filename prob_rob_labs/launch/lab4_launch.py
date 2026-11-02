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
                    'turtlebot3_empty_launch.py'
                )
            ),
        ),
        # Node(
        #     package='prob_rob_labs',
        #     executable='twist_calibration',
        #     name='twist_calibration',
        #     parameters=[
        #         {'use_sim_time': LaunchConfiguration('use_sim_time')},
        #     ],
        #     remappings=[
        #         ('gt_twist', '/tb3/ground_truth/twist'),
        #     ],
        # ),
        Node(
            package='prob_rob_labs',
            executable='ekf_odometry',
            name='ekf_odometry',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            remappings=[
                ('cmd_vel', '/cmd_vel'),
                ('joint_states', '/joint_states'),
                ('imu', '/imu'),
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['--x', '0', '--y', '0', '--z', '0', 
                       '--yaw', '0', '--pitch', '0', '--roll', '0', 
                       '--frame-id', 'world', '--child-frame-id', 'odom']
        ),
        Node(
            package='prob_rob_labs',
            executable='ekf_odom_metric',
            name='ekf_odom_metric',
            parameters=[
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
            remappings=[
                ('ekf_odom', '/ekf_odom'),
                ('gt_pose', '/tb3/ground_truth/pose'),
            ],
        ),
    ])
