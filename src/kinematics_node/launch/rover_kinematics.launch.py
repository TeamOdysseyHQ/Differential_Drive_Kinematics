from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_name = 'kinematics_node'
    config = os.path.join(
        get_package_share_directory(pkg_name),
        'config',
        'rover_kinematics.yaml'
    )
    
    return LaunchDescription([
        Node(
            package=pkg_name,
            executable='rover_kinematics_node',
            name='rover_kinematics_node',
            output='screen',
            parameters=[config],
        ),
    ])
