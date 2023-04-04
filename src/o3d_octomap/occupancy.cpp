#include "o3d_octomap/occupancy.hpp"

#include <chrono>

OccupancyMethod::OccupancyMethod() : rclcpp::Node("occupancy") {

    ReadSharedParams(this, this->params);

    const CameraIntrinsic& K = this->params.intrinsic;

    RCLCPP_WARN_STREAM(this->get_logger(), "depth: " << this->params.depth_w << "x" << this->params.depth_h);
    RCLCPP_WARN(this->get_logger(), "cam: (%.2f,%.2f,%.2f,%.2f)", K.cx, K.cy, K.fx, K.fy);

    this->fuser = std::make_shared<Fuser>(this->params, 0.02, 256, true);

    this->rgbd_subscription = this->create_subscription<auto_scanner::msg::PosedRGBD>("/occupancy/rgbd", 20, 
        std::bind(&OccupancyMethod::ProcessRGBDMessage, this, std::placeholders::_1));

    this->viz_loop = this->create_wall_timer(std::chrono::milliseconds(40), std::bind(&Fuser::UpdateVisLoop, this->fuser.get()));
}

void OccupancyMethod::ProcessRGBDMessage(const auto_scanner::msg::PosedRGBD& rgbd) const {
    this->fuser->FuseRGBDMessage(rgbd);
    this->fuser->UpdateVisGeometry();
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