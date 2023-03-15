#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from cv_bridge.core import CvBridge
from auto_scanner.msg import PosedImage
import cv2

class DepthExtractionNode(Node):
    def __init__(self):
        super().__init__("depth_extract")
        self.poseImg = self.create_subscription(PosedImage, "/auto_scanner/orbslam/pose_image", self.pose_image_callback, qos_profile=5)

    def pose_image_callback(self, msg):
        img, pose = msg.image, msg.cam_pose
        cvbridge = CvBridge()
        cvImg = cvbridge.imgmsg_to_cv2(img)
        cv2.imshow("feed", cvImg)
        cv2.waitKey(100)

def main():
    rclpy.init()

    depth_extract = DepthExtractionNode()

    rclpy.spin(depth_extract)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
