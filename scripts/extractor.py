#!/usr/bin/env python

from threading import Thread
import cv2
import rclpy
import numpy as np

from rclpy.node import Node
from cv_bridge.core import CvBridge

from auto_scanner.msg import PosedImage, PosedRGBD
from options.heath_node import HeathReportingNode
from std_msgs.msg import Bool

from geometry_msgs.msg import Quaternion, Vector3
from simplerecon.depth_estimate import SimpleRecon

from options.utils import \
    get_intrinsic_components, \
    pose_msg2matrix44,\
    get_depth_size, get_raw_image_size

class DepthExtractionNode(HeathReportingNode):
    def __init__(self):
        super().__init__("depth_extract")
        self.__declare_parameter()
        self.__get_parameters()
        
        self.poseImgSub = self.create_subscription(PosedImage, "/extractor/posed_image", self.pose_image_callback, qos_profile=50)
        self.poseDepthPub = self.create_publisher(PosedRGBD, "/extractor/posed_rgbd", qos_profile=5)
        self.create_subscription(Bool, "/extractor/running", self.__change_running_state, qos_profile=5)

        self.logger = self.get_logger()
        self.rate = self.create_rate(2)
        self.is_running = False

        self.model = SimpleRecon(
            self.depth_img_sz,
            self.input_img_sz,
            self.model_weights
        )
        
        self.model.setup_intrinsic(
            self.fx, self.fy,
            self.cx, self.cy
        )

        self._inform_ready()

    def __change_running_state(self, x: Bool):
        self.is_running = x.data

    def __declare_parameter(self):
        self.declare_parameter("weights", "")

    def __get_parameters(self):
        self.input_img_sz = get_raw_image_size(self)
        self.depth_img_sz = get_depth_size(self)

        self.model_weights = self.get_parameter("weights").value
        self.fx, self.fy, self.cx, self.cy = get_intrinsic_components(self)
        
        assert(len(self.model_weights) > 0)

    def pose_image_callback(self, msg: PosedImage):
        if not self.is_running:
            self.model.frame_history.clear()
            return
        
        img, pose = msg.image, msg.cam_pose
        cvbridge = CvBridge()
        cvImg = cvbridge.imgmsg_to_cv2(img)
        T = pose_msg2matrix44(pose.orientation, pose.position);
        
        has_result, depth = self.model.process_frame(cvImg, T)

        if not has_result:
            return
        
        depth = cv2.resize(depth, self.depth_img_sz)
        cvImg = cv2.resize(cvImg, self.depth_img_sz)
        depthScaled = depth.copy()
        depthScaled = depthScaled / np.max(depthScaled) * 255
        depthScaled = depthScaled.astype(np.uint8)

        posedRGBD = PosedRGBD()
        posedRGBD.cam_pose = pose;
        posedRGBD.rgb = cvbridge.cv2_to_imgmsg(cvImg, header=img.header)
        posedRGBD.depth = cvbridge.cv2_to_imgmsg(depthScaled, header=img.header)
        self.poseDepthPub.publish(posedRGBD)

def main():
    rclpy.init()

    depth_extract = DepthExtractionNode()

    rclpy.spin(depth_extract)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
