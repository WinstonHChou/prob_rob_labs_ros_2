
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    lab_dir = Path(__file__).resolve().parent.parent / 'src' / 'lab5'
    launch_dir = Path(__file__).resolve().parent

    height = LaunchConfiguration('height')
    color = LaunchConfiguration('color')
    landmark_x = LaunchConfiguration('landmark_x')
    landmark_y = LaunchConfiguration('landmark_y')

    return LaunchDescription([
        DeclareLaunchArgument('height', default_value='0.5'),
        DeclareLaunchArgument('color', default_value='cyan'),
        DeclareLaunchArgument('landmark_x', default_value='0.0'),
        DeclareLaunchArgument('landmark_y', default_value='0.0'),

        # Start Assignment 2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_dir / 'lab5_assignment2_launch.py')
            ),
            launch_arguments={
                'height': height,
                'color': color
            }.items()
        ),

        # Start Assignment 3
        ExecuteProcess(
            cmd=[
                sys.executable,
                str(lab_dir / 'measurement_error_estimator.py'),
                '--ros-args',
                '-p', ['landmark_x:=', landmark_x],
                '-p', ['landmark_y:=', landmark_y]
            ],
            cwd=str(lab_dir),
            output='screen'
        )
    ])
