
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    lab_dir = Path(__file__).resolve().parent.parent / 'src' / 'lab5'

    height = LaunchConfiguration('height')
    color = LaunchConfiguration('color')

    return LaunchDescription([
        DeclareLaunchArgument('height', default_value='0.5'),
        DeclareLaunchArgument('color', default_value='cyan'),

        ExecuteProcess(
            cmd=[
                sys.executable,
                str(lab_dir / 'params_estimator.py'),
                '--ros-args',
                '-p', ['landmark_height:=', height],
                '-p', ['landmark_color:=', color]
            ],
            cwd=str(lab_dir),
            output='screen'
        )
    ])
