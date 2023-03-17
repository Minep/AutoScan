#!/usr/bin/env python

from mesh.o3d_fuser import Open3DFuser
from mesh.o3d_viewer import Viewer3D
from auto_scanner.msg import PosedRGBD
from cv_bridge import CvBridge
from rclpy.node import Node
import cv2
import numpy as np

from options.utils import get_intrinsic_components, pose_msg2matrix44, get_depth_size

import rclpy

class MeshFuserNode(Node):
    def __init__(self) -> None:
        super().__init__("mesh_fuser")

        self.viewer = Viewer3D()
        self.cvBridge = CvBridge()
        self.intrinsics = get_intrinsic_components(self)
        self.fuser = Open3DFuser(self.intrinsics, get_depth_size(self))
        self.posedDepthSub = self.create_subscription(
            PosedRGBD, 
            "/mesh_fuser/posed_depth", 
            self.__depth_map_callback,
            qos_profile=50)

        self.viewer_loop = self.create_timer(1 / 20, lambda: self.viewer.tick())
    
    def __depth_map_callback(self, msg: PosedRGBD):
        rgb_msg, depth_msg, pose_msg = msg.rgb, msg.depth, msg.cam_pose
        depth_mat = self.cvBridge.imgmsg_to_cv2(depth_msg)
        rgb_mat = self.cvBridge.imgmsg_to_cv2(rgb_msg)
        T = pose_msg2matrix44(pose_msg.orientation, pose_msg.position)

        self.fuser.fuse_frames(depth_mat, T, rgb_mat)

        self.viewer.update_mesh(self.fuser.get_mesh())

def main():
    rclpy.init()

    fuser = MeshFuserNode()

    rclpy.spin(fuser)

    rclpy.shutdown()


if __name__ == '__main__':
    main()