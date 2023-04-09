#ifndef BD59E706_77E4_4402_99E6_F9067D62B828
#define BD59E706_77E4_4402_99E6_F9067D62B828

#include <rclcpp/rclcpp.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>

#include "param_helper.hpp"
#include "auto_scanner/srv/capture_state.hpp"

class GstreamingNode : public rclcpp::Node {
private:
    std::shared_ptr<cv::VideoCapture> capture;
    SharedParams params;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rawImgStream;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr captureStateChanged;
    rclcpp::Service<auto_scanner::srv::CaptureState>::SharedPtr captureStateChange;
    rclcpp::TimerBase::SharedPtr readStream;

    cv::Mat currentFrame;

    bool started = false;
    bool single_step = false;
    uint64_t frame_id = 0;
    uint64_t timestamp = 0;
    uint64_t increament = 0;

public:
    explicit GstreamingNode();
    
protected:
    void ReadVideoStream();
    void OnCaptureStateToggled(
        const std::shared_ptr<auto_scanner::srv::CaptureState_Request> req,
        const std::shared_ptr<auto_scanner::srv::CaptureState_Response> resp
    );
};

#endif /* BD59E706_77E4_4402_99E6_F9067D62B828 */
