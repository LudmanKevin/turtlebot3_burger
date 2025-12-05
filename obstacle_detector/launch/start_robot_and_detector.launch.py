from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    bringup_launch_file = os.path.join(
        get_package_share_directory('turtlebot3_bringup'),
        'launch',
        'robot.launch.py'
    )

    ekf_launch_file = os.path.join(
        get_package_share_directory('tb3_localization'),
        'launch',
        'ekf_launch.py'
    )

    return LaunchDescription([
        # TurtleBot3 bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch_file)
        ),

        # EKF localization (tb3_localization package-ből)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(ekf_launch_file)
        ),

        # obstacle_detector node
        Node(
            package='obstacle_detector',
            executable='obstacle_detector',
            name='obstacle_detector',
            output='screen'
        ),
    ])