
import sys
from pathlib import Path

from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():
    lab_dir = Path(__file__).resolve().parent.parent / 'src' / 'lab5'

    return LaunchDescription([
        ExecuteProcess(
            cmd=[sys.executable, str(lab_dir / 'landmark_center.py')],
            cwd=str(lab_dir),
            output='screen'
        )
    ])
