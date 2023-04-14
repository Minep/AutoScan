#include "o3d_octomap/occupancy.hpp"

#include <chrono>
#include <float.h>

#define PI 3.1416

OccupancyMethod::OccupancyMethod() : HealthReportingNode("occupancy") {

    ReadSharedParams(this, this->params);

    const CameraIntrinsic& K = this->params.intrinsic;

    RCLCPP_WARN_STREAM(this->get_logger(), "depth: " << this->params.depth_w << "x" << this->params.depth_h);
    RCLCPP_WARN(this->get_logger(), "cam: (%.2f,%.2f,%.2f,%.2f)", K.cx, K.cy, K.fx, K.fy);

    this->fuser = std::make_shared<Fuser>(this->params, 300, true);
    this->octree = std::make_shared<octomap::OcTree>(this->params.geoParam.resolution_proc);
    this->octoPcd = std::make_shared<octomap::Pointcloud>();

    this->pose_subscription = this->create_subscription<geometry_msgs::msg::Pose>(
        "/fuser/cam_pose", 5, std::bind(&OccupancyMethod::PoseUpdated, this, std::placeholders::_1));

    this->rgbd_subscription = this->create_subscription<auto_scanner::msg::PosedRGBD>("/occupancy/rgbd", 20, 
        std::bind(&OccupancyMethod::ProcessRGBDMessage, this, std::placeholders::_1));

    this->viz_loop = this->create_wall_timer(std::chrono::milliseconds(40), std::bind(&Fuser::UpdateVisLoop, this->fuser.get()));

    std::random_device rdev;
    this->rng = std::mt19937(rdev());
    this->dist_phi = std::uniform_int_distribution<std::mt19937::result_type>(0, 90);
    this->dist_theta = std::uniform_int_distribution<std::mt19937::result_type>(0, 359);

    this->inform_ready();
}

void OccupancyMethod::PoseUpdated(const geometry_msgs::msg::Pose& pose) {
    auto qr = pose.orientation;
    auto pos = pose.position;
    
    Eigen::Quaterniond q(qr.w, qr.x, qr.y, qr.z);
    Eigen::Matrix4d T;
    T.setIdentity();
    T.block<3,3>(0,0) = q.toRotationMatrix();
    T.block<3,1>(0,3) = Eigen::Vector3d(pos.x, pos.y, pos.z);

    this->fuser->SetPose(T);
}

void OccupancyMethod::ProcessRGBDMessage(const auto_scanner::msg::PosedRGBD& rgbd) const {
    try
    {
        this->fuser->FuseRGBDMessage(rgbd);
        this->fuser->UpdateVisMesh();
    }
    catch(const std::exception& e)
    {
        RCLCPP_FATAL(this->get_logger(), "%s", e.what());
    }
}

void 
OccupancyMethod::LocateUnknownRegion(octomap::point3d& result, octomap::point3d sensor_origin) {
    auto occupancy = this->fuser->occupancy();

    octomap::OcTreeKey key;
    octomap::OcTreeNode* node;
    // TODO: sample rays from the hemisphere around user and do ray casting, to see whether it is
    //       unknown area. The optimal ray will be elected by distance from user.

    octomap::point3d dir;
    octomap::point3d end;
    octomap::point3d best;
    double d = DBL_MAX;

    for (int i = 0; i < 360; i++) {
        double theta = i / 180 * PI;
        double phi = (double)dist_phi(this->rng) / 180 * PI;
        double ct = std::cos(theta), st = std::sin(theta);
        double cp = std::cos(phi), sp = std::sin(phi);
        dir.x() = st * sp;
        dir.y() = -ct * sp;
        dir.z() = cp;

        if(occupancy->castRay(sensor_origin, dir, end)) {
            continue;
        }

        double d_ = end.distance(sensor_origin);
        if (d_ < d) {
            d = d_;
            best = end;
        }
    }

    result = best;
}

void OccupancyMethod::Release() {
    this->fuser->Release();
}

int main(int argc, const char* argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr<OccupancyMethod> occ = std::make_shared<OccupancyMethod>();

    exe.add_node(occ->get_node_base_interface());

    exe.spin();

    occ->Release();

    rclcpp::shutdown();

    return 0;
}