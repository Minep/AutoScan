from rclpy.node import Node
from rclpy.exceptions import ParameterAlreadyDeclaredException
from geometry_msgs.msg import Quaternion, Vector3
import numpy as np

def get_intrinsic_components(ros_node: Node):
    try:
        ros_node.declare_parameter("fx", 720.)
        ros_node.declare_parameter("fy", 640.)
        ros_node.declare_parameter("cx", 10.)
        ros_node.declare_parameter("cy", 10.)
    except ParameterAlreadyDeclaredException:
        pass
    fx = ros_node.get_parameter("fx").value
    fy = ros_node.get_parameter("fy").value
    cx = ros_node.get_parameter("cx").value
    cy = ros_node.get_parameter("cy").value
    return fx, fy, cx, cy

def get_depth_size(ros_node: Node):
    try:
        ros_node.declare_parameter("depth_img_w", 1280)
        ros_node.declare_parameter("depth_img_h", 720)
    except ParameterAlreadyDeclaredException:
        pass

    w = ros_node.get_parameter("depth_img_w").value
    h = ros_node.get_parameter("depth_img_h").value

    return w, h

def get_raw_image_size(ros_node: Node):
    try:
        ros_node.declare_parameter("input_img_w", 1280)
        ros_node.declare_parameter("input_img_h", 720)
    except ParameterAlreadyDeclaredException:
        pass

    w = ros_node.get_parameter("input_img_w").value
    h = ros_node.get_parameter("input_img_h").value

    return w, h

def get_shared_param(ros_node: Node):
    return {
        'raw_size': get_raw_image_size(ros_node),
        'depth_size': get_depth_size(ros_node),
        'intrinsic': get_intrinsic_components(ros_node)
    }

def pose_msg2matrix44(q: Quaternion, t: Vector3):
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