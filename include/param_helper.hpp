#pragma once

#include <rclcpp/rclcpp.hpp>

typedef struct CameraIntrinsic {
    double fx;
    double fy;
    double cx;
    double cy;
} CameraIntrinsic;

typedef struct SharedParams {
    int input_w;
    int input_h;
    int depth_w;
    int depth_h;
    CameraIntrinsic intrinsic;
} SharedParams;

void ReadSharedParams(rclcpp::Node* node, SharedParams& intrinsic);

void ReadCameraIntrinsic(rclcpp::Node* node, CameraIntrinsic& intrinsic);