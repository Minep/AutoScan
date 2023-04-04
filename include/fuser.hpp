#pragma once

#include <open3d/Open3D.h>
#include <opencv2/opencv.hpp>

#include "param_helper.hpp"
#include "auto_scanner/msg/posed_rgbd.hpp"

class Fuser {
private:
    open3d::pipelines::integration::ScalableTSDFVolume* tsdf;
    open3d::camera::PinholeCameraIntrinsic* camK;
    open3d::geometry::KDTreeSearchParamHybrid* kdSearch;
    open3d::geometry::PointCloud gpcd;

    std::shared_ptr<open3d::geometry::TriangleMesh> saved_geometry = nullptr;
    std::shared_ptr<open3d::visualization::Visualizer> vis = nullptr;

    open3d::geometry::Image depth_o3d;
    open3d::geometry::Image rgb_o3d;
    
    int depth_width = 0, depth_height = 0;
    int max_fusion_depth = 0;

    double voxel_size = 0;
    bool first_frame = false;
    bool enable_vis = false;
public:
    Fuser(const SharedParams& param, double resoluion, int max_depth, bool enable_vis);
    void Release();
    void FuseRGBDMessage(const auto_scanner::msg::PosedRGBD& rgbd_msg);
    void FuseFrame(cv::Mat& rgb, cv::Mat& depth, Eigen::Matrix4d& extrinsic);
    std::shared_ptr<open3d::geometry::PointCloud> GetPointCloud();
    std::shared_ptr<open3d::geometry::TriangleMesh> GetMesh();

    void UpdateVisLoop();
    void UpdateVisGeometry();

protected:
    Eigen::Matrix4d LocalFinetuning(open3d::geometry::RGBDImage& rgbd, Eigen::Matrix4d& extrinsic_prior);
};