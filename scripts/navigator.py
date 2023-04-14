#!/usr/bin/env python

import rclpy
import numpy as np
import threading

import cv2
import open3d as o3d

from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

from options.utils import pose_msg2matrix44, get_shared_param
from options.heath_node import HeathReportingNode
from auto_scanner.srv import NodeReady
from auto_scanner.msg import PosedRGBD, PosedImage


class Navigator():
    def __init__(self, param) -> None:
        self.preview_size = (288, 512, 3)
        self.scale_ratio = (
            self.preview_size[0] / param['raw_size'][1],
            self.preview_size[1] / param['raw_size'][0]
        )
        preview_size_mono = (self.preview_size[0], self.preview_size[1])
        
        self.depth_preview = np.full(preview_size_mono, 30, order='C', dtype=np.uint8)
        self.rgb_feed = np.full(self.preview_size, 30, order='C', dtype=np.uint8)
        self.slam_feed = np.full(self.preview_size, 30, order='C', dtype=np.uint8)
        self.slam_points = []
        self.current_pose_inv = np.identity(4)
        self.goal_point = np.ones(4)
        self.__lock = threading.Lock()
        self.__lock_rgb = threading.Lock()
        self.__lock_depth = threading.Lock()

        self.rgb_changed = True
        self.depth_changed = True
        self.slam_changed = True

        self.navigator_panel = (self.preview_size[0] * 2, self.preview_size[1] * 2)
        self.frame_buffer = np.zeros(
            (self.navigator_panel[0], self.navigator_panel[1], 3), 
            order='C', dtype=np.uint8
        )
        
        self.__scale = np.diag(np.full((4,), 0.8))
        self.__scale[3,3] = 1

        self.__compass_shape = np.array([
            [ 0.00, -0.40, 0.00, 1],
            [-0.20,  0.35, 0.00, 1],
            [ 0.00,  0.20, 0.00, 1],
            [ 0.20,  0.35, 0.00, 1]
        ]) @ self.__scale
        N = 100
        Nticks = 12
        self.__compass_shape_outline = np.zeros((N, 4), dtype=np.float32)
        self.__compass_shape_ticks = np.zeros((Nticks, 2, 4), dtype=np.float32)
        self.__compass_shape_outline[:,3] = 1.0
        self.__compass_shape_outline[:,2] = 0.0
        self.__compass_shape_ticks[:,:,3] = 1.0
        self.__compass_shape_ticks[:,:,2] = 0.0
        
        inc = 2 * 3.1416 / N
        for i in range(N):
            self.__compass_shape_outline[i,0] = np.cos(i * inc) * 0.45
            self.__compass_shape_outline[i,1] = np.sin(i * inc) * 0.45
        self.__compass_shape_outline = self.__compass_shape_outline @ self.__scale
        
        inc_ticks = 2 * 3.1416 / Nticks
        for i in range(Nticks):
            scale = 0.06 if i % 3 != 0 else 0.1
            x_, y_ = np.cos(i * inc_ticks), np.sin(i * inc_ticks)
            self.__compass_shape_ticks[i, 0,0] = x_ * 0.45
            self.__compass_shape_ticks[i, 0,1] = y_ * 0.45
            self.__compass_shape_ticks[i, 1,0] = x_ * (0.45 + scale)
            self.__compass_shape_ticks[i, 1,1] = y_ * (0.45 + scale)
        self.__compass_shape_ticks = np.einsum("ijl,lk->ijk", self.__compass_shape_ticks, self.__scale)

        self.__elevation_shape = np.array([
            [ 0.00, -0.50, 0.00, 1],
            [ 0.00,  0.00, 0.00, 1]
        ]) @ self.__scale

        self.__R_compass = np.identity(4, dtype=np.float32)
        self.__S_elevation = np.identity(4, dtype=np.float32)

        cam_rot = o3d.geometry.Geometry3D.get_rotation_matrix_from_xyz(np.array([55 * 0.01745,0,0]))
        self.view_matrix = np.identity(4)
        self.view_matrix[:3,:3] = cam_rot.T

        self.proj_matrix = self.create_projection_matrix(100, 0, 1)

        self.compass_position = np.array([
            self.navigator_panel[1] / 8 * 5, 
            self.navigator_panel[0] / 4])
        self.elevation_position = np.array([
            self.navigator_panel[1] / 8 * 7 , 
            self.navigator_panel[0] / 4])
            

    def create_projection_matrix(self, fov, near, far):
        P = np.identity(4, dtype=np.float32)
        scale = 1 / np.tan(fov * 0.5 * 3.14 / 180)
        P[0,0] = P[1, 1] = scale
        P[2,2] = -far / (far - near)
        P[3,2] = -far * near / (far - near)
        P[2,3] = -1
        P[3,3] = 0
        return P

    def __update_arrow_rotation(self, z_angle):
        c = np.cos(z_angle)
        s = np.sin(z_angle)
        self.__R_compass[0,0] = c
        self.__R_compass[1,1] = c
        self.__R_compass[0,1] = s
        self.__R_compass[1,0] = -s

    def start_navigator_loop(self, fps):
        self.stop_loop = False
        self.__cv_loop(fps)

    def set_goal(self, goal):
        with self.__lock:
            self.goal_point = goal

    def set_slam(self, pose, slam_rgb, trakers):
        with self.__lock:
            h,w,d = self.preview_size
            cv2.resize(src=slam_rgb, dsize=(w, h), dst=self.slam_feed)
            cv2.cvtColor(src=self.slam_feed, code=cv2.COLOR_RGB2GRAY, dst=self.slam_feed)
            self.slam_feed[:,:,1] = self.slam_feed[:,:,0]
            self.slam_feed[:,:,2] = self.slam_feed[:,:,0]
            self.slam_points = trakers
            self.current_pose_inv = np.linalg.inv(pose)
            self.slam_changed = True

    def set_rgb(self, rgb):
        with self.__lock_rgb:
            h,w,d = self.preview_size
            cv2.resize(src=rgb, dsize=(w, h), dst=self.rgb_feed)
            cv2.cvtColor(src=self.rgb_feed, code=cv2.COLOR_RGB2BGR, dst=self.rgb_feed)
            self.rgb_changed = True
    
    def set_depth(self, depth):
        with self.__lock_depth:
            h,w,d = self.preview_size
            cv2.resize(src=depth, dsize=(w, h), dst=self.depth_preview)
            self.depth_changed = True

    def stop(self):
        self.stop_loop = True

    def update(self, delay):
        with self.__lock_depth:
            if self.depth_changed:
                self.frame_buffer[self.preview_size[0]:, self.preview_size[1]:, 0] = self.depth_preview
                self.frame_buffer[self.preview_size[0]:, self.preview_size[1]:, 1] = self.depth_preview
                self.frame_buffer[self.preview_size[0]:, self.preview_size[1]:, 2] = self.depth_preview
                self.depth_changed = False
        
        with self.__lock_rgb:
            if self.rgb_changed:
                self.frame_buffer[:self.preview_size[0], :self.preview_size[1], :] = self.rgb_feed
                self.depth_changed = False
        
        with self.__lock:
            if self.slam_changed:
                # slam view
                self.frame_buffer[
                    self.preview_size[0]:, 
                    :self.preview_size[1], :] = self.slam_feed
                
                G = np.array([0, 255, 0])
                for p in self.slam_points:
                    p: Point = p
                    if p.x < 0 or p.y < 0:
                        continue
                    self.frame_buffer[
                        int(self.preview_size[0] + p.y * self.scale_ratio[0]),
                        int(p.x * self.scale_ratio[1]),
                        :
                    ] = G
                
                self.frame_buffer[:self.preview_size[0], self.preview_size[1]:, :].fill(0)
                goal_point_local = self.current_pose_inv @ self.goal_point
                direction = goal_point_local / np.linalg.norm(goal_point_local, ord=2)
                crx = direction[0] # direction cross Z
                theta = np.arccos(direction[2])  # direction dot Z
                if crx < 0:
                    theta = 2 * 3.1416 - theta
                
                # draw compass pointer
                self.__update_arrow_rotation(theta)
                M = self.view_matrix @ self.proj_matrix
                arrow = (self.__compass_shape @ self.__R_compass @ M) * self.preview_size[0]
                outline = (self.__compass_shape_outline @ M) * self.preview_size[0]
                ticks = np.einsum("ijl,lk->ijk", self.__compass_shape_ticks, M) * self.preview_size[0]

                cv2.fillPoly(
                    self.frame_buffer, 
                    pts=[np.int32(arrow[:,:2] + self.compass_position)], 
                    color=(255, 212, 59),
                    lineType=cv2.LINE_AA)
                cv2.polylines(
                    self.frame_buffer, 
                    pts=[np.int32(outline[:,:2] + self.compass_position)], 
                    isClosed=True,
                    color=(255, 212, 59),
                    lineType=cv2.LINE_AA,
                    thickness=2)
                cv2.polylines(
                    self.frame_buffer,
                    pts=np.int32(ticks[:,:,:2] + self.compass_position),
                    isClosed=False,
                    color=(255, 212, 59),
                    lineType=cv2.LINE_AA,
                    thickness=1
                )
                
                # elevation indicator
                self.__S_elevation[1,1] = direction[1]
                elevation = (self.__elevation_shape @ self.__S_elevation) * self.preview_size[0]
                cv2.arrowedLine(
                    self.frame_buffer, 
                    pt2=np.int32(elevation[0,:2] + self.elevation_position), 
                    pt1=np.int32(elevation[1,:2] + self.elevation_position), 
                    color=(255, 212, 59),
                    line_type=cv2.LINE_AA,
                    thickness=2,
                    tipLength=0.2)
            self.slam_changed = False

        cv2.imshow("preview", self.frame_buffer)
        cv2.waitKey(delay=delay)
    
class NavigatorNode(HeathReportingNode):
    def __init__(self):
        super().__init__("navigator")

        param = get_shared_param(self)
        delay = 1 / 24
        self.navigator_window = Navigator(param)
        self.create_timer(delay, lambda: self.navigator_window.update(int(delay * 1000)))
        self.create_subscription(PosedRGBD, "/navigator/rgbd", self.__posed_rgbd_recieved, 10)
        self.create_subscription(PosedImage, "/navigator/slam", self.__slam_recieved, 10)
        self.create_subscription(Image, "/navigator/rgb", self.__posed_rgb_recieved, 10)
        self.cv_convertor = CvBridge()
        
        self._inform_ready()

    def __slam_recieved(self, msg: PosedImage):
        pose, rgb, points = msg.cam_pose, msg.image, msg.tracked_points
        pose = pose_msg2matrix44(pose.orientation, pose.position)

        rgb_cv = self.cv_convertor.imgmsg_to_cv2(rgb)

        self.navigator_window.set_slam(pose, rgb_cv, points)

    def __posed_rgbd_recieved(self, msg: PosedRGBD):
        depth = msg.depth
        depth_cv = self.cv_convertor.imgmsg_to_cv2(depth)
        depth_cv = (depth_cv / np.max(depth_cv) * 255).astype(np.uint8)
        
        self.navigator_window.set_depth(depth_cv)
    
    def __posed_rgb_recieved(self, msg: Image):
        img = self.cv_convertor.imgmsg_to_cv2(msg)

        self.navigator_window.set_rgb(img)

def main():
    rclpy.init()

    navigator = NavigatorNode()
    rclpy.spin(navigator)

    cv2.destroyAllWindows()

    rclpy.shutdown()

if __name__ == "__main__":
    main()

