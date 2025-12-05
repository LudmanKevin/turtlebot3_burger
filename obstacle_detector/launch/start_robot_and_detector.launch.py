from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # turtlebot3_bringup csomag launch könyvtára
    bringup_launch_file = os.path.join(
        get_package_share_directory('turtlebot3_bringup'),
        'launch',
        'robot.launch.py'
    )

    return LaunchDescription([
        # TurtleBot3 bringup indítása
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch_file)
        ),

        # tb3_localization node indítása
        Node(
            package='robot_localization',
            executable='ekf_node',
            output='screen',
            name='ekf_filter_node',
            parameters=['/home/arnl/ros2_ws/src/turtlebot3_burger_csomag/tb3_localization/config/ekf.yaml']
        ),

        # obstacle_detector node indítása
        Node(
            package='obstacle_detector',
            executable='obstacle_detector',
            name='obstacle_detector',
            output='screen'
        ),
    ])