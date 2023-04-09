#include <chrono>
#include <cv_bridge/cv_bridge.h>

#include "gstreaming/gstreaming.hpp"

GstreamingNode::GstreamingNode() : rclcpp::Node("gstreaming") {
    this->declare_parameter("video_dev", "/dev/video0");
    this->declare_parameter("video_file", "");
    this->declare_parameter("fps_rescale", 20);
    this->declare_parameter("default_on", true);
    this->declare_parameter("single_step", false);

    ReadSharedParams(this, this->params);

    this->started = this->get_parameter("default_on").as_bool();
    this->single_step = this->get_parameter("single_step").as_bool();
    
    char buf[1024];
    int n = 0;
    auto video_dev = this->get_parameter("video_dev").as_string();
    auto video_file = this->get_parameter("video_file").as_string();
    int fps_rescaled = this->get_parameter("fps_rescale").as_int();
    if (video_file.empty()) {
        // initialize v4lsrc;
        auto g_str = "v4l2src device=%s ! "
        "decodebin ! videoconvert ! videorate ! videoscale ! "
        "video/x-raw, framerate=%d/1, width=%d, height=%d ! "
        "appsink drop=1";
        n = snprintf(buf, 1024, g_str, fps_rescaled, this->params.input_w, this->params.input_h);
    }
    else {
        auto g_str = "filesrc location=%s ! "
        "decodebin ! videoconvert ! videorate ! videoscale ! "
        "video/x-raw, framerate=%d/1, width=%d, height=%d ! "
        "appsink drop=1";
        n = snprintf(buf, 1024, g_str, fps_rescaled, this->params.input_w, this->params.input_h);
    }

    this->increament = 1'000'000'000ul / fps_rescaled;

    std::string gstr = std::string(buf, n);
    this->capture = std::make_shared<cv::VideoCapture>(gstr, cv::CAP_GSTREAMER);

    RCLCPP_INFO(this->get_logger(), "Gstreamer config: %s", gstr);

    if (!this->capture->isOpened()) {
        RCLCPP_FATAL(this->get_logger(), "fail to open gstreamer for capturing");
        rclcpp::shutdown();
        return;
    }

    this->captureStateChanged = this->create_publisher<std_msgs::msg::Bool>("state_changed", 5);
    this->rawImgStream = this->create_publisher<sensor_msgs::msg::Image>("rgb", 5);
    this->readStream = this->create_wall_timer(
        std::chrono::milliseconds(1000 / fps_rescaled),
        std::bind(&GstreamingNode::ReadVideoStream, this)
    );
    this->captureStateChange = this->create_service<auto_scanner::srv::CaptureState>(
        "capturing", std::bind(
            &GstreamingNode::OnCaptureStateToggled, 
            this, 
            std::placeholders::_1, std::placeholders::_2)
    );
}

inline void 
notify_state_changed(const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher, bool state) {
    std_msgs::msg::Bool msg;
    msg.data = state;
    publisher->publish(msg);
}

void 
GstreamingNode::OnCaptureStateToggled(
    const std::shared_ptr<auto_scanner::srv::CaptureState_Request> req,
    const std::shared_ptr<auto_scanner::srv::CaptureState_Response> resp
) {
    this->started = req->desire_state;
    resp->ack_state = this->started;
    
    std_msgs::msg::Bool msg;
    msg.data = this->started;
    this->captureStateChanged->publish(msg);
    RCLCPP_INFO(this->get_logger(), "%s capturing.", this->started ? "start" : "stop");
}

void 
GstreamingNode::ReadVideoStream() {
    if (!this->started) {
        return;
    }

    if (!this->capture->read(currentFrame)) {
        RCLCPP_WARN(this->get_logger(), "End of stream");
        goto reset_state;
    }

    {
        std_msgs::msg::Header header;
        header.stamp = rclcpp::Time(this->timestamp);
        header.frame_id = this->frame_id;

        cv::cvtColor(this->currentFrame, this->currentFrame, cv::COLOR_BGR2RGB);
        auto cvMsg = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, this->currentFrame).toImageMsg();
        this->rawImgStream->publish(*cvMsg);
        
        this->timestamp += this->increament;
        this->frame_id++;
    }

    if (this->single_step) {
reset_state:
        this->started = false;
        notify_state_changed(this->captureStateChanged, this->started);
    }
}

int main(int argc, const char* argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr<GstreamingNode> gstnode = std::make_shared<GstreamingNode>();

    exe.add_node(gstnode->get_node_base_interface());

    exe.spin();

    rclcpp::shutdown();

    return 0;
}