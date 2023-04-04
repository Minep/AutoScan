#include "param_helper.hpp"

void ReadSharedParams(rclcpp::Node* node, SharedParams& params) {
    try
    {
        node->declare_parameter("input_img_w", 720);
        node->declare_parameter("input_img_h", 640);
        node->declare_parameter("depth_img_w", 720);
        node->declare_parameter("depth_img_h", 640);
    }
    catch(const rclcpp::exceptions::ParameterAlreadyDeclaredException& e)
    {
        // nothing to do
    }

    params.depth_w = node->get_parameter("depth_img_w").as_int();
    params.depth_h = node->get_parameter("depth_img_h").as_int();
    params.input_w = node->get_parameter("input_img_w").as_int();
    params.input_h = node->get_parameter("input_img_h").as_int();

    ReadCameraIntrinsic(node, params.intrinsic);
}

void ReadCameraIntrinsic(rclcpp::Node* node, CameraIntrinsic& intrinsic) {
    try
    {
        node->declare_parameter("fx", 720.0);
        node->declare_parameter("fy", 640.0);
        node->declare_parameter("cx", 10.0);
        node->declare_parameter("cy", 10.0);
    }
    catch(const rclcpp::exceptions::ParameterAlreadyDeclaredException& e)
    {
        // nothing to do
    }
    
    intrinsic.fx = node->get_parameter("fx").as_double();
    intrinsic.fy = node->get_parameter("fy").as_double();
    intrinsic.cx = node->get_parameter("cx").as_double();
    intrinsic.cy = node->get_parameter("cy").as_double();
}