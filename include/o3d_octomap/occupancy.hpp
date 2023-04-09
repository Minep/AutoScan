#ifndef E6514295_88C5_43FF_83A5_B4FE6222B6DC
#define E6514295_88C5_43FF_83A5_B4FE6222B6DC

#include <rclcpp/rclcpp.hpp>
#include <octomap/octomap.h>

#include "fuser.hpp"
#include "param_helper.hpp"
#include "auto_scanner/msg/posed_rgbd.hpp"

class OccupancyMethod : public rclcpp::Node {
private:
    std::shared_ptr<Fuser> fuser;
    std::shared_ptr<octomap::OcTree> octree;
    std::shared_ptr<octomap::Pointcloud> octoPcd;
    
    SharedParams params;

    rclcpp::TimerBase::SharedPtr viz_loop;
    rclcpp::Subscription<auto_scanner::msg::PosedRGBD>::SharedPtr rgbd_subscription;

public:
    OccupancyMethod();

    void Release();

    void ProcessRGBDMessage(const auto_scanner::msg::PosedRGBD& rgbd) const;

protected:
    void LocateUnknownRegion(octomap::point3d& result, octomap::point3d sensor_origin);
};


#endif /* E6514295_88C5_43FF_83A5_B4FE6222B6DC */
