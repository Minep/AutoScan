#!/usr/bin/env python

from threading import Thread
import rclpy
import numpy as np
from rclpy.node import Node
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from cv_bridge.core import CvBridge
from auto_scanner.msg import PosedImage, PosedRGBD
from geometry_msgs.msg import Quaternion, Vector3
from simplerecon.depth_estimate import SimpleRecon
import cv2

from options.utils import \
    get_intrinsic_components, \
    pose_msg2matrix44,\
    get_depth_size, get_raw_image_size

class DepthExtractionNode(Node):
    def __init__(self):
        super().__init__("depth_extract")
        self.__declare_parameter()
        self.__get_parameters()
        
        self.poseImgSub = self.create_subscription(PosedImage, "/extractor/posed_image", self.pose_image_callback, qos_profile=50)
        self.poseDepthPub = self.create_publisher(PosedRGBD, "/extractor/posed_rgbd", qos_profile=5)

        self.orbslam_getState = self.create_client(GetState, "/orb_slam3/get_state")
        self.orbslam_changeState = self.create_client(ChangeState, "/orb_slam3/change_state")

        self.logger = self.get_logger();
        self.rate = self.create_rate(2)

        self.model = SimpleRecon(
            self.depth_img_sz,
            self.input_img_sz,
            self.model_weights
        )
        
        self.model.setup_intrinsic(
            self.fx, self.fy,
            self.cx, self.cy
        )

    def notify_slam(self):
        result = self.__change_state(Transition.TRANSITION_CONFIGURE)
        if not result:
            self.logger.error("Failed transition to 'configure'")

        self.rate.sleep()

        result = self.__change_state(Transition.TRANSITION_ACTIVATE)
        if not result:
            self.logger.error("Failed transition to 'activate'")
        
    def __change_state(self, newState):
        if not self.orbslam_changeState.wait_for_service(10):
            self.logger.error("ChangeState service not avaliable")
            return
        stateChangeReq = ChangeState.Request()
        stateChangeReq.transition.id = newState
        
        future = self.orbslam_changeState.call_async(stateChangeReq)

        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                break

        return future.result().success
    
    def __get_state(self):
        if not self.orbslam_getState.wait_for_service(10):
            self.logger.error("GetState service not avaliable")
            return
        stateGetReq = GetState.Request()
        result = self.orbslam_getState.call(stateGetReq)

        return result.current_state.id


    def __declare_parameter(self):
        self.declare_parameter("weights", "")

    def __get_parameters(self):
        self.input_img_sz = get_raw_image_size(self)
        self.depth_img_sz = get_depth_size(self)

        self.model_weights = self.get_parameter("weights").value
        self.fx, self.fy, self.cx, self.cy = get_intrinsic_components(self)
        
        assert(len(self.model_weights) > 0)

    

    def pose_image_callback(self, msg: PosedImage):
        img, pose = msg.image, msg.cam_pose
        cvbridge = CvBridge()
        cvImg = cvbridge.imgmsg_to_cv2(img)
        T = pose_msg2matrix44(pose.orientation, pose.position);
        
        has_result, depth = self.model.process_frame(cvImg, T)

        if not has_result:
            return
        
        depth = cv2.resize(depth, self.depth_img_sz)
        depthScaled = depth.copy()
        depthScaled = depthScaled / np.max(depthScaled) * 256
        depthScaled = depthScaled.astype(np.uint8)
        display = cv2.applyColorMap(depthScaled, cv2.COLORMAP_JET)
        cv2.imshow("depth", display)
        cv2.waitKey(100)

        posedRGBD = PosedRGBD()
        posedRGBD.cam_pose = pose;
        posedRGBD.rgb = img
        posedRGBD.depth = cvbridge.cv2_to_imgmsg(depthScaled, header=img.header)
        self.poseDepthPub.publish(posedRGBD)

def main():
    rclpy.init()

    depth_extract = DepthExtractionNode()
    depth_extract.notify_slam()

    rclpy.spin(depth_extract)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
