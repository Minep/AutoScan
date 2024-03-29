#include "param_helper.hpp"


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

void ReadGeometryParams(rclcpp::Node* node, GeometryParams& geo_param) {
    try
    {
        node->declare_parameter("res_recon", 0.02);
        node->declare_parameter("res_process", 0.2);
    }
    catch(const rclcpp::exceptions::ParameterAlreadyDeclaredException& e)
    {
        // nothing to do
    }
    
    geo_param.resolution_proc = node->get_parameter("res_process").as_double();
    geo_param.resolution_recon = node->get_parameter("res_recon").as_double();

}

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

    params.geoParam.w_scale_ratio = (double)params.depth_w / (double)params.input_w;
    params.geoParam.h_scale_ratio = (double)params.depth_h / (double)params.input_h;

    ReadCameraIntrinsic(node, params.intrinsic);
    ReadGeometryParams(node, params.geoParam);
}
