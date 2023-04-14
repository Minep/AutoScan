#ifndef E6514295_88C5_43FF_83A5_B4FE6222B6DC
#define E6514295_88C5_43FF_83A5_B4FE6222B6DC

#include <rclcpp/rclcpp.hpp>
#include <octomap/octomap.h>

#include "fuser.hpp"
#include "param_helper.hpp"
#include "health_node.hpp"
#include "auto_scanner/msg/posed_rgbd.hpp"
#include "geometry_msgs/msg/pose.hpp"

#include <random>

class OccupancyMethod : public HealthReportingNode {
private:
    std::shared_ptr<Fuser> fuser;
    std::shared_ptr<octomap::OcTree> octree;
    std::shared_ptr<octomap::Pointcloud> octoPcd;
    
    SharedParams params;

    rclcpp::TimerBase::SharedPtr viz_loop;
    rclcpp::Subscription<auto_scanner::msg::PosedRGBD>::SharedPtr rgbd_subscription;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscription;

    std::mt19937 rng;
    std::uniform_int_distribution<std::mt19937::result_type> dist_phi;
    std::uniform_int_distribution<std::mt19937::result_type> dist_theta;

public:
    OccupancyMethod();

    void Release();

    void ProcessRGBDMessage(const auto_scanner::msg::PosedRGBD& rgbd) const;

protected:
    void LocateUnknownRegion(octomap::point3d& result, octomap::point3d sensor_origin);

    void PoseUpdated(const geometry_msgs::msg::Pose& pose);
};


#endif /* E6514295_88C5_43FF_83A5_B4FE6222B6DC */
