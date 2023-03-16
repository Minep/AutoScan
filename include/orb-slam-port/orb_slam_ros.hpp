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

typedef rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn LCCallbackReturn;

class OrbSlamROS : public rclcpp::Node {
private:
    ORB_SLAM3::System* SLAM;
    cv::VideoCapture* videoCapture;
    int fps;
    int fps_low;
    int fps_scale_ratio;
    int frame_interval;
    rclcpp::Publisher<auto_scanner::msg::PosedImage>::SharedPtr poseImgPub;
    rclcpp::Publisher<auto_scanner::msg::PosedImage>::SharedPtr poseImgPubLow;
    rclcpp::TimerBase::SharedPtr timer;
    cv::Mat currentFrame;
    float imgScale;
    long frame_id = 0;
    int counter = 0;

    void decalre_parameters();

    void setup_gst_capture();

    void setup_topics();

    void cv2_feed_spin();

    void setup_file_capture(const std::string& file);

    void init_opencv_source();

public:
    explicit OrbSlamROS();

    void CleanUp();
};

#endif /* CC9D8C15_409B_453E_8E8E_858F8CF97D84 */
