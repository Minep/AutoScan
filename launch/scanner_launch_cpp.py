from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os;

def generate_launch_description():
    pkg_dir = get_package_share_directory("auto_scanner")
    param_file = os.path.join(pkg_dir, "config", "auto_scanner_params.yml")
    return LaunchDescription([
        Node(
            package="auto_scanner",
            executable="gstreaming",
            name="gstreaming",
            parameters=[param_file],
            remappings= [
                ("/gstreaming/node_health", "/control_panel/inform_ready")
            ]
        ),
        Node(
            package='auto_scanner',
            executable='orb_slam3',
            name='orb_slam3',
            remappings = [
                ("/orb_slam3/rgb_in", "/gstreaming/rgb"),
                ("/orb_slam3/node_health", "/control_panel/inform_ready")
            ],
            parameters= [param_file],
        ),
        Node(
            package='auto_scanner',
            executable='extractor.py',
            name='depth_extract',
            parameters= [param_file],
            remappings=[
                ("/extractor/posed_image", "/orb_slam3/posed_image"),
                ("/extractor/node_health", "/control_panel/inform_ready")
            ]
        ),         
        Node(
            package="auto_scanner",
            executable="navigator.py",
            name="navigator",
            parameters=[param_file],
            remappings=[
                ("/navigator/rgb", "/gstreaming/rgb"),
                ("/navigator/rgbd", "/extractor/posed_rgbd"),
                ("/navigator/slam", "/orb_slam3/posed_image")
            ]
        ),
        Node(
            package="auto_scanner",
            executable="occupancy",
            name="occupancy",
            remappings=[
                ("/occupancy/rgbd", "/extractor/posed_rgbd"),
                ("/fuser/cam_pose", "/orb_slam3/pose")
            ],
            parameters= [param_file]
        ),
        Node(
            package="auto_scanner",
            executable="control_panel.py",
            name="control_panel",
            remappings=[
                ("/control_panel/recon_state", "/extractor/running")
            ],
            parameters=[
                param_file,
                {
                    "node_list": [
                        "gstreaming",
                        "orb_slam3",
                        "navigator",
                        "depth_extract",
                        "occupancy"
                    ]
                }
            ]
        )
    ])