#ifndef CC9D8C15_409B_453E_8E8E_858F8CF97D84
#define CC9D8C15_409B_453E_8E8E_858F8CF97D84

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "System.h"

#include "auto_scanner/msg/posed_image.hpp"

#include <opencv2/core.hpp>
#include <cv_bridge/cv_bridge.h>

class OrbSlamROS : public rclcpp::Node {
private:
    ORB_SLAM3::System* SLAM;
    rclcpp::Publisher<auto_scanner::msg::PosedImage>::SharedPtr poseImgPub;
    rclcpp::Publisher<auto_scanner::msg::PosedImage>::SharedPtr poseImgPubLow;
    rclcpp::TimerBase::SharedPtr timer = nullptr;
    cv::Mat currentFrame;
    cv::Mat rgbConverted;
    float imgScale;

    void decalre_parameters();

    void setup_topics();

    void cv2_feed_spin(const sensor_msgs::msg::Image& image);
public:
    explicit OrbSlamROS();

    void CleanUp();
};

#endif /* CC9D8C15_409B_453E_8E8E_858F8CF97D84 */
