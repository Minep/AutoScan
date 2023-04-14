#pragma once

#include <open3d/Open3D.h>
#include <opencv2/opencv.hpp>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

#include "geometry_msgs/msg/pose.hpp"

#include "param_helper.hpp"
#include "auto_scanner/msg/posed_rgbd.hpp"

class Fuser {
private:
    std::shared_ptr<open3d::pipelines::integration::ScalableTSDFVolume> tsdf;
    std::shared_ptr<open3d::camera::PinholeCameraIntrinsic> camK;
    std::shared_ptr<open3d::camera::PinholeCameraParameters> camParam;
    std::shared_ptr<open3d::geometry::KDTreeSearchParamHybrid> kdSearch;

    std::shared_ptr<open3d::geometry::TriangleMesh> saved_geometry = nullptr;
    std::shared_ptr<open3d::geometry::PointCloud> saved_occupancy = nullptr;
    std::shared_ptr<open3d::visualization::Visualizer> vis = nullptr;

    std::shared_ptr<octomap::OcTree> ocMapping;
    std::shared_ptr<octomap::Pointcloud> ocPcd;

#ifdef ICP_PAIR_WISE
    std::shared_ptr<open3d::geometry::PointCloud> prev_pcd = nullptr;
#else
    open3d::geometry::PointCloud gpcd;
#endif
    open3d::geometry::Image depth_o3d;
    open3d::geometry::Image rgb_o3d;
    
    Eigen::Matrix4d LH2RH;

    GeometryParams geoParam;
    
    int depth_width = 0, depth_height = 0;
    int max_fusion_depth = 0;

    double voxel_size = 0;
    bool first_frame = false;
    bool enable_vis = false;
public:
    Fuser(const SharedParams& param, int max_depth, bool enable_vis);
    void Release();
    
    void
    FuseRGBDMessage(const auto_scanner::msg::PosedRGBD& rgbd_msg);

    void
    FuseFrame(cv::Mat& rgb, cv::Mat& depth, Eigen::Matrix4d& extrinsic);
    
    std::shared_ptr<open3d::geometry::PointCloud> GetPointCloud();
    std::shared_ptr<open3d::geometry::TriangleMesh> GetMesh();

    void UpdateVisLoop();
    void UpdateVisMesh();
    void UpdateVisOccupancy();
    void SetPose(const Eigen::Matrix4d& pose);

    const std::shared_ptr<octomap::OcTree> occupancy() const {
        return this->ocMapping;
    }

protected:
    Eigen::Matrix4d LocalFinetuning(
        const std::shared_ptr<open3d::geometry::PointCloud> pcd, 
        Eigen::Matrix4d& extrinsic_prior
    );

};