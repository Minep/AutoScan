#pragma once

#include <rclcpp/rclcpp.hpp>

typedef struct CameraIntrinsic {
    double fx;
    double fy;
    double cx;
    double cy;
} CameraIntrinsic;

typedef struct GeometryParams {
    double resolution_recon;
    double resolution_proc;
} GeometryParams;

typedef struct SharedParams {
    int input_w;
    int input_h;
    int depth_w;
    int depth_h;
    CameraIntrinsic intrinsic;
    GeometryParams geoParam;
} SharedParams;

void ReadSharedParams(rclcpp::Node* node, SharedParams& intrinsic);