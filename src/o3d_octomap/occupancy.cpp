#include "o3d_octomap/occupancy.hpp"

#include <chrono>

OccupancyMethod::OccupancyMethod() : rclcpp::Node("occupancy") {

    ReadSharedParams(this, this->params);

    const CameraIntrinsic& K = this->params.intrinsic;

    RCLCPP_WARN_STREAM(this->get_logger(), "depth: " << this->params.depth_w << "x" << this->params.depth_h);
    RCLCPP_WARN(this->get_logger(), "cam: (%.2f,%.2f,%.2f,%.2f)", K.cx, K.cy, K.fx, K.fy);

    this->fuser = std::make_shared<Fuser>(this->params, 300, true);
    this->octree = std::make_shared<octomap::OcTree>(this->params.geoParam.resolution_proc);
    this->octoPcd = std::make_shared<octomap::Pointcloud>();

    this->rgbd_subscription = this->create_subscription<auto_scanner::msg::PosedRGBD>("/occupancy/rgbd", 20, 
        std::bind(&OccupancyMethod::ProcessRGBDMessage, this, std::placeholders::_1));

    this->viz_loop = this->create_wall_timer(std::chrono::milliseconds(40), std::bind(&Fuser::UpdateVisLoop, this->fuser.get()));
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
    octomap::point3d split_center = sensor_origin;
    octomap::point3d vert_max = occupancy->getBBXMax();
    octomap::point3d vert_min = occupancy->getBBXMin();
    int depth = 1, max_depth = occupancy->getTreeDepth();
    int dim_idx = 0;
    bool found = false;

    octomap::OcTreeKey key;
    octomap::OcTreeNode* node;
    while (!found || depth <= max_depth)
    {
        for (int i = 0; i < 3; i++) {
            int dim = i % 3;
            float old_val = split_center(dim);
            double prob_occ_r = 0, prob_occ_l = 0;
            // right hand side
            split_center(dim) = (vert_max(dim) + old_val) / 2;
            if (!occupancy->coordToKeyChecked(split_center, depth, key) ||
                !(node = occupancy->search(key, depth))) {
                // found unknow area
                found = true;
                break;
            }
            prob_occ_r = node->getMeanChildLogOdds();

            // search left
            split_center(dim) = (vert_min(dim) - old_val) / 2;
            if (!occupancy->coordToKeyChecked(split_center, depth, key) ||
                !(node = occupancy->search(key, depth))) {
                // found unknow area
                found = true;
                break;
            }
            prob_occ_l = node->getMeanChildLogOdds();

            if (prob_occ_l <= prob_occ_r) {
                vert_max(dim) = old_val;
            }
            else {
                vert_min(dim) = old_val;
                split_center(dim) = (vert_max(dim) + old_val) / 2;
            }
        }

        depth++;
    }
    
    result(0) = key.k[0];
    result(1) = key.k[1];
    result(2) = key.k[2];
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