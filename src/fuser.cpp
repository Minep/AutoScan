#include "fuser.hpp"
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/pose.hpp"
#include <Eigen/Eigen>

Fuser::Fuser(const SharedParams& param, double resoluion = 0.02, int max_depth = 256, bool enable_vis = true) {
    this->depth_height = param.depth_h;
    this->depth_width = param.depth_w;
    this->max_fusion_depth = max_depth;

    const CameraIntrinsic& K = param.intrinsic;
    
    this->camK = new open3d::camera::PinholeCameraIntrinsic(
        param.depth_w, param.depth_h,
        K.fx, K.fy,
        K.cx, K.cy
    );

    this->voxel_size = resoluion * 100;
    this->tsdf = new open3d::pipelines::integration::ScalableTSDFVolume(
        resoluion, 2 * resoluion, 
        open3d::pipelines::integration::TSDFVolumeColorType::RGB8
    );

    this->kdSearch = new open3d::geometry::KDTreeSearchParamHybrid(this->voxel_size, 20);
    this->gpcd = open3d::geometry::PointCloud();

    this->first_frame = true;
    this->enable_vis = enable_vis;

    if (enable_vis) {
        this->vis = std::make_shared<open3d::visualization::Visualizer>();
        this->vis->CreateVisualizerWindow("Reconstruction");
    }

    this->depth_o3d.Prepare(this->depth_width, this->depth_height, 1, 1);
    this->rgb_o3d.Prepare(this->depth_width, this->depth_height, 3, 1);
}

void Fuser::Release() {
    delete this->camK;
    delete this->tsdf;
    delete this->kdSearch;

    if (this->enable_vis) {
        this->vis->DestroyVisualizerWindow();
    }
}

void Fuser::FuseRGBDMessage(const auto_scanner::msg::PosedRGBD& rgbd_msg) {
    cv_bridge::CvImagePtr depth = cv_bridge::toCvCopy(rgbd_msg.depth);
    cv_bridge::CvImagePtr rgb = cv_bridge::toCvCopy(rgbd_msg.rgb);
    geometry_msgs::msg::Pose pose = rgbd_msg.cam_pose;

    auto qr = pose.orientation;
    auto pos = pose.position;
    
    Eigen::Quaterniond q(qr.w, qr.x, qr.y, qr.z);
    Eigen::Matrix4d T;
    T.setIdentity();
    T.block<3,3>(0,0) = q.toRotationMatrix();
    T.block<3,1>(0,3) = Eigen::Vector3d(pos.x, pos.y, pos.z);

    this->FuseFrame(rgb->image, depth->image, T);
}

void Fuser::FuseFrame(cv::Mat& rgb, cv::Mat& depth, Eigen::Matrix4d& extrinsic) {
    memcpy(this->depth_o3d.data_.data(), depth.data, this->depth_o3d.data_.size());
    memcpy(this->rgb_o3d.data_.data(), rgb.data, this->rgb_o3d.data_.size());
    
    auto rgbd = open3d::geometry::RGBDImage::CreateFromColorAndDepth(
        this->rgb_o3d, this->depth_o3d, 1.0, this->max_fusion_depth, false
    );

    auto fine_tuned = this->LocalFinetuning(*rgbd, extrinsic);
    
    this->tsdf->Integrate(*rgbd, *this->camK, fine_tuned);
}

Eigen::Matrix4d Fuser::LocalFinetuning(open3d::geometry::RGBDImage& rgbd, Eigen::Matrix4d& extrinsic_prior) {
    auto pcd = open3d::geometry::PointCloud::CreateFromRGBDImage(rgbd, *this->camK);
    auto pcd_down = pcd->VoxelDownSample(this->voxel_size / 2.0);
    pcd_down->EstimateNormals(*this->kdSearch);

    auto pcd_transformed = pcd_down->Transform(extrinsic_prior);

    if (this->first_frame) {
        this->gpcd += pcd_transformed;
        this->gpcd.EstimateNormals(*this->kdSearch);
        this->first_frame = false;
        return extrinsic_prior;
    }

    auto result = open3d::pipelines::registration::RegistrationICP(
        pcd_transformed, this->gpcd,
        0.01, extrinsic_prior, 
        open3d::pipelines::registration::TransformationEstimationPointToPlane()
    );

    auto T = result.transformation_;

    this->gpcd += pcd_down->Transform(T);
    this->gpcd.EstimateNormals(*this->kdSearch);

    return T;
}

void Fuser::UpdateVisLoop() {
    if (!this->enable_vis) {
        return;
    }

    this->vis->PollEvents();
    this->vis->UpdateRender();
}

void Fuser::UpdateVisGeometry() {
    if (!this->enable_vis) {
        return;
    }

    this->vis->RemoveGeometry(this->saved_geometry);

    this->saved_geometry = this->GetMesh();
    this->vis->AddGeometry(this->saved_geometry);
}

std::shared_ptr<open3d::geometry::PointCloud> Fuser::GetPointCloud() {
    return this->tsdf->ExtractPointCloud();
}

std::shared_ptr<open3d::geometry::TriangleMesh> Fuser::GetMesh() {
    return this->tsdf->ExtractTriangleMesh();
}