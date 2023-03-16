#!/usr/bin/env python

import rclpy
import numpy as np
from rclpy.node import Node
from cv_bridge.core import CvBridge
from auto_scanner.msg import PosedImage
from geometry_msgs.msg import Quaternion, Vector3
from simplerecon.depth_estimate import SimpleRecon
import cv2

class DepthExtractionNode(Node):
    def __init__(self):
        super().__init__("depth_extract")
        self.__declare_parameter()
        self.__get_parameters()
        
        self.model = SimpleRecon(
            self.depth_img_sz,
            self.input_img_sz,
            self.model_weights
        )
        
        self.model.setup_intrinsic(
            self.fx, self.fy,
            self.cx, self.cy
        )

        self.poseImgSub = self.create_subscription(PosedImage, "/extractor/pose_image", self.pose_image_callback, qos_profile=20)
        self.poseDepthPub = self.create_publisher(PosedImage, "/extractor/pose_depth", qos_profile=5)

    def __declare_parameter(self):
        self.declare_parameter("input_img_w", 1280)
        self.declare_parameter("input_img_h", 720)
        self.declare_parameter("depth_img_w", 1280)
        self.declare_parameter("depth_img_h", 720)
        self.declare_parameter("weights", "")
        self.declare_parameter("fx", 10.0)
        self.declare_parameter("fy", 10.0)
        self.declare_parameter("cx", 640.0)
        self.declare_parameter("cy", 360.0)

    def __get_parameters(self):
        self.input_img_sz = (
            self.get_parameter("input_img_w").value,
            self.get_parameter("input_img_h").value
        )
        self.depth_img_sz = (
            self.get_parameter("depth_img_w").value,
            self.get_parameter("depth_img_h").value
        )

        self.model_weights = self.get_parameter("weights").value
        self.fx = self.get_parameter("fx").value
        self.fy = self.get_parameter("fy").value
        self.cx = self.get_parameter("cx").value
        self.cy = self.get_parameter("cy").value

        assert(len(self.model_weights) > 0)

    def pose_msg2matrix44(self, q: Quaternion, t: Vector3):
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        tx, ty, tz = t.x, t.y, t.z
        return np.array([
            [
                1 - 2 * qz * qz - 2 * qw * qw,
                2 * qy * qz - 2 * qx * qw,
                2 * qw * qy + 2 * qx * qz,
                tx
            ], [
                2 * qy * qz + 2 * qx * qw,
                1 - 2 * qy * qy - 2 * qw * qw,
                2 * qz * qw - 2 * qx * qy,
                ty
            ], [
                2 * qw * qy - 2 * qx * qz,
                2 * qz * qw + 2 * qx * qy,
                1 - 2 * qy * qy - 2 * qz * qz,
                tz
            ],
            [0,0,0,1]
        ])

    def pose_image_callback(self, msg: PosedImage):
        img, pose = msg.image, msg.cam_pose
        cvbridge = CvBridge()
        cvImg = cvbridge.imgmsg_to_cv2(img)
        T = self.pose_msg2matrix44(pose.orientation, pose.position);
        depth = self.model.process_frame(cvImg, T)
        depth = cv2.resize(depth, self.depth_img_sz)
        depthScaled = depth.copy() - np.min(depth)
        depthScaled = depthScaled / np.max(depthScaled) * 256
        display = cv2.applyColorMap(depthScaled.astype(np.uint8), cv2.COLORMAP_JET)
        cv2.imshow("depth", display)
        cv2.waitKey(1000)

        posedDepth = PosedImage()
        posedDepth.cam_pose = pose;
        posedDepth.image = cvbridge.cv2_to_imgmsg(depth, header=img.header)
        self.poseDepthPub.publish(posedDepth)

def main():
    rclpy.init()

    depth_extract = DepthExtractionNode()

    rclpy.spin(depth_extract)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
