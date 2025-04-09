from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get path to the YAML file
    config_file = os.path.join(
        get_package_share_directory('trajectory'),
        'param',
        'traj.yaml'
    )

    return LaunchDescription([
        Node(
            package='trajectory',
            executable='trajectory_reader_node',
            name='trajectory_reader_node',
            parameters=[config_file],
            output='screen'
        )
    ])