import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('your_package_name'),
        'config',
        'zed_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='your_package_name',
            executable='zed_camera_node',
            name='zed_camera_node',
            parameters=[config],
            output='screen',
        )
    ])
