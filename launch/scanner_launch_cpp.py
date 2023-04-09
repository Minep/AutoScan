from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os;

def generate_launch_description():
    pkg_dir = get_package_share_directory("auto_scanner")
    param_file = os.path.join(pkg_dir, "config", "auto_scanner_params.yml")

    node_list = [
        {
            'package': 'auto_scanner',
            'executable': 'orb_slam3',
            'name': 'orb_slam3',
            'remappings ':  [
                ("/orb_slam3/posed_image", "/extractor/posed_image")
            ],
            'parameters':  [param_file],
        },
        {
            'package': 'auto_scanner',
            'executable': 'extractor.py',
            'name': 'depth_extract',
            'remappings ':  [
                ("/extractor/posed_rgbd", "/occupancy/rgbd")
            ],
            'parameters':  [param_file],
        },
        {
            'package': 'auto_scanner',
            'executable': 'occupancy',
            'name': 'occupancy',
            'parameters':  [param_file],
        }
    ]

    all_nodes = [x['name'] for x in node_list]

    node_list.append({
        'package': 'auto_scanner',
        'executable': 'control_panel',
        'name': 'control_panel',
        'parameters':  [
            param_file,
            {
                'node_list': all_nodes
            }
        ]
    })

    return LaunchDescription([
        Node(**x) for x in node_list
    ])