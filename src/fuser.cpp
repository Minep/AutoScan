// #define ICP_PAIR_WISE
#define ENABLE_ICP_FINETUNE

#include "fuser.hpp"
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/pose.hpp"
#include <Eigen/Eigen>

Fuser::Fuser(const SharedParams& param, int max_depth = 256, bool enable_vis = true) {
    this->depth_height = param.depth_h;
    this->depth_width = param.depth_w;
    this->max_fusion_depth = max_depth;

    this->geoParam = param.geoParam;

    const CameraIntrinsic& K = param.intrinsic;
    
    this->camK = std::make_shared<open3d::camera::PinholeCameraIntrinsic>(
        param.depth_w, param.depth_h,
        K.fx * this->geoParam.w_scale_ratio, K.fy * this->geoParam.h_scale_ratio,
        K.cx * this->geoParam.w_scale_ratio, K.cy * this->geoParam.h_scale_ratio
    );
    this->camParam = std::make_shared<open3d::camera::PinholeCameraParameters>();

    double resoluion = this->geoParam.resolution_recon;
    this->voxel_size = resoluion * 100;

    this->tsdf = std::make_shared<open3d::pipelines::integration::ScalableTSDFVolume>(
        resoluion, 2 * resoluion, 
        open3d::pipelines::integration::TSDFVolumeColorType::RGB8
    );

    this->saved_occupancy = std::make_shared<open3d::geometry::PointCloud>();

    this->ocMapping = std::make_shared<octomap::OcTree>(this->geoParam.resolution_proc);
    this->ocPcd = std::make_shared<octomap::Pointcloud>();

    this->kdSearch = std::make_shared<open3d::geometry::KDTreeSearchParamHybrid>(this->voxel_size, 20);

#ifndef ICP_PAIR_WISE
    this->gpcd = open3d::geometry::PointCloud();
#endif

    this->first_frame = true;
    this->enable_vis = enable_vis;
    
    this->LH2RH.setIdentity();
    this->LH2RH.block<3,3>(0,0) = open3d::geometry::Geometry3D::GetRotationMatrixFromXYZ(
        Eigen::Vector3d(0, 0, 3.1415)
    );

    if (enable_vis) {
        this->vis = std::make_shared<open3d::visualization::Visualizer>();
        this->vis->CreateVisualizerWindow("Reconstruction");
        this->vis->GetViewControl().ConvertToPinholeCameraParameters(*this->camParam);
        open3d::visualization::RenderOption& render_opts = this->vis->GetRenderOption();
        render_opts.show_coordinate_frame_ = true;
    }

    this->depth_o3d.Prepare(this->depth_width, this->depth_height, 1, 1);
    this->rgb_o3d.Prepare(this->depth_width, this->depth_height, 3, 1);
}

void Fuser::SetPose(const Eigen::Matrix4d& pose) {
    this->camParam->extrinsic_ = pose * this->LH2RH;
}

void Fuser::Release() {
    if (this->enable_vis) {
        this->vis->DestroyVisualizerWindow();
    }
}

void
Fuser::FuseRGBDMessage(const auto_scanner::msg::PosedRGBD& rgbd_msg) {
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

    rgb.reset();
    depth.reset();
}

void
Fuser::FuseFrame(cv::Mat& rgb, cv::Mat& depth, Eigen::Matrix4d& extrinsic) {
    memcpy(this->depth_o3d.data_.data(), depth.data, this->depth_o3d.data_.size());
    memcpy(this->rgb_o3d.data_.data(), rgb.data, this->rgb_o3d.data_.size());
    
    auto rgbd = open3d::geometry::RGBDImage::CreateFromColorAndDepth(
        this->rgb_o3d, this->depth_o3d, 1.0, this->max_fusion_depth, false
    );

    auto pcd = open3d::geometry::PointCloud::CreateFromRGBDImage(*rgbd, *this->camK);
    auto pcd_downsampled = pcd->VoxelDownSample(this->geoParam.resolution_proc);
    
#ifdef ENABLE_ICP_FINETUNE
    Eigen::Matrix4d fine_tuned = this->LocalFinetuning(pcd, extrinsic);
#else
    Eigen::Matrix4d& fine_tuned = extrinsic;
#endif

    auto o3d_frame = fine_tuned * this->LH2RH;

    this->tsdf->Integrate(*rgbd, *this->camK, o3d_frame);

    octomap::point3d origin(
        extrinsic.coeffRef(0, 3),
        extrinsic.coeffRef(1, 3),
        extrinsic.coeffRef(2, 3)
    );

    this->ocPcd->clear();

    auto pcd_pts = pcd_downsampled->points_;
    for(auto it = pcd_pts.begin(); it != pcd_pts.end(); it++) {
        auto pt = *it;
        this->ocPcd->push_back(pt.x(), pt.y(), pt.z());
    }

    this->ocMapping->insertPointCloud(*this->ocPcd, origin, this->max_fusion_depth);

    pcd.reset();
    rgbd.reset();
    pcd_downsampled.reset();

    // this->camParam->extrinsic_ = extrinsic * this->LH2RH;
}

Eigen::Matrix4d 
Fuser::LocalFinetuning(const std::shared_ptr<open3d::geometry::PointCloud> pcd, Eigen::Matrix4d& extrinsic_prior) {
    auto pcd_down = pcd->VoxelDownSample(this->geoParam.resolution_proc);
    pcd_down->EstimateNormals(*this->kdSearch);
    pcd_down->Transform(extrinsic_prior);

    if (this->first_frame) {
#ifndef ICP_PAIR_WISE
        this->gpcd += *pcd_down;
        this->gpcd.EstimateNormals(*this->kdSearch);
#else
        this->prev_pcd = pcd_down;
#endif
        this->first_frame = false;
        return extrinsic_prior;
    }

    auto result = open3d::pipelines::registration::RegistrationICP(
        *pcd_down, 
#ifdef ICP_PAIR_WISE
        *this->prev_pcd,
#else
        this->gpcd,
#endif
        0.01, extrinsic_prior, 
        open3d::pipelines::registration::TransformationEstimationPointToPlane()
    );

    auto T = result.transformation_;

    pcd_down->Transform(T * extrinsic_prior.inverse());

#ifdef ICP_PAIR_WISE
    this->prev_pcd.reset();
    this->prev_pcd = pcd_down;
#else
    this->gpcd += *pcd_down;
    this->gpcd.EstimateNormals(*this->kdSearch);
#endif

    pcd_down.reset();

    return T;
}

void Fuser::UpdateVisLoop() {
    if (!this->enable_vis) {
        return;
    }

    this->vis->PollEvents();
    this->vis->UpdateRender();
}

void Fuser::UpdateVisMesh() {
    if (!this->enable_vis) {
        return;
    }

    this->vis->RemoveGeometry(this->saved_geometry);
    this->saved_geometry.reset();
    this->saved_geometry = this->tsdf->ExtractTriangleMesh();
    this->vis->AddGeometry(this->saved_geometry);

    open3d::visualization::ViewControl& ctr = this->vis->GetViewControl();
    ctr.ConvertFromPinholeCameraParameters(*this->camParam, true);
}

void Fuser::UpdateVisOccupancy() {
    if (!this->enable_vis) {
        return;
    }

    this->vis->RemoveGeometry(this->saved_geometry);
    this->saved_geometry->Clear();

    unsigned int depth = this->ocMapping->getTreeDepth();

    for (auto it=this->ocMapping->begin(); it != this->ocMapping->end(); it++) {
        const double prob = it->getOccupancy() / 2;
        const auto coord = it.getCoordinate();
        this->saved_occupancy->points_.push_back(
            Eigen::Vector3d(coord.x(),
            coord.y(),
            coord.z()));
        this->saved_occupancy->colors_.push_back(
            Eigen::Vector3d(
                prob, prob, prob
            ));
    }

    this->vis->AddGeometry(this->saved_geometry);

    open3d::visualization::ViewControl& ctr = this->vis->GetViewControl();
    ctr.ConvertFromPinholeCameraParameters(*this->camParam, true);
}

std::shared_ptr<open3d::geometry::PointCloud> 
Fuser::GetPointCloud() {
    return this->tsdf->ExtractPointCloud();
}

std::shared_ptr<open3d::geometry::TriangleMesh> 
Fuser::GetMesh() {
    return this->tsdf->ExtractTriangleMesh();
}