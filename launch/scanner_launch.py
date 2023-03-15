from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os;

def generate_launch_description():
    pkg_dir = get_package_share_directory("auto_scanner")
    param_file = os.path.join(pkg_dir, "config", "auto_scanner_params.yml")
    return LaunchDescription([
        Node(
            package='auto_scanner',
            executable='orb_slam3',
            name='orb_slam3',
            remappings=[
                ('/pose_image', '/auto_scanner/orbslam/pose_image')
            ],
            parameters= [param_file],
            
        ),
    ])