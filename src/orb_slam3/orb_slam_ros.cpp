#include "orb-slam-port/orb_slam_ros.hpp"

#include <opencv2/core/core.hpp>
#include "Tracking.h"
#include "param_helper.hpp"

typedef ORB_SLAM3::Tracking::eTrackingState TrackingState;

OrbSlamROS::OrbSlamROS() : HealthReportingNode("orb_slam3") {
    this->decalre_parameters();

    SharedParams param;
    ReadSharedParams(this, param);

    std::string voc = this->get_parameter("orb_voc").as_string();
    std::string cam_cfg = this->get_parameter("cam_cfg").as_string();

    RCLCPP_INFO_STREAM(this->get_logger(), "vocabulary: " << voc);
    RCLCPP_INFO_STREAM(this->get_logger(), "camera config: " << cam_cfg);

    this->SLAM = new ORB_SLAM3::System(voc, cam_cfg, ORB_SLAM3::System::MONOCULAR, false);
    this->imgScale = this->SLAM->GetImageScale();

    ORB_SLAM3::Settings* setting = this->SLAM->GetSetting();
    cv::Size new_sz = setting->newImSize();
    this->scale_h = param.input_h / new_sz.height;
    this->scale_w = param.input_w / new_sz.width;

    this->setup_topics();
    this->inform_ready();
}

void 
OrbSlamROS::decalre_parameters() {
    this->declare_parameter("orb_voc", "");
    this->declare_parameter("cam_cfg", "");
}

void 
OrbSlamROS::setup_topics() {
    this->rgbIn = this->create_subscription<sensor_msgs::msg::Image>("/orb_slam3/rgb_in", 5, std::bind(&OrbSlamROS::cv2_feed_spin, this, std::placeholders::_1));
    this->rgbIn = this->create_subscription<sensor_msgs::msg::Image>("/orb_slam3/rgb_in", 5, std::bind(&OrbSlamROS::cv2_feed_spin, this, std::placeholders::_1));
    this->poseImgPub = this->create_publisher<auto_scanner::msg::PosedImage>("/orb_slam3/posed_image", 5);
    this->pose = this->create_publisher<geometry_msgs::msg::Pose>("/orb_slam3/pose", 5);
}

void 
OrbSlamROS::cv2_feed_spin(const sensor_msgs::msg::Image& image) {
    cv_bridge::CvImagePtr rgb = cv_bridge::toCvCopy(image);
    cv::Mat frame = rgb->image;
    double inv_imgscale = 1 / this->imgScale;

    if (this->imgScale != 1.f) {
        int w = frame.cols * this->imgScale;
        int h = frame.rows * this->imgScale;
        cv::resize(frame, frame, cv::Size(w, h));
    }

    auto ts = image.header.stamp;
    double sec = (double)ts.nanosec * 1e-9 + (double)ts.sec;
    auto pose = this->SLAM->TrackMonocular(frame, sec);
    
    if (this->SLAM->GetTrackingState() != TrackingState::OK) {
        RCLCPP_WARN(this->get_logger(), "Tracking Failed");
        return;
    }

    auto T = pose.translation();

    auto quat = pose.unit_quaternion();
    auto rosPose = geometry_msgs::msg::Pose();

    rosPose.position.x = T.x();
    rosPose.position.y = T.y();
    rosPose.position.z = T.z();
    rosPose.orientation.x = quat.x();
    rosPose.orientation.y = quat.y();
    rosPose.orientation.z = quat.z();
    rosPose.orientation.w = quat.w();

    auto_scanner::msg::PosedImage piMessage = auto_scanner::msg::PosedImage();
    piMessage.image = image;
    piMessage.cam_pose = rosPose;
    
    auto keypoints = this->SLAM->GetTrackedKeyPointsUn();
    geometry_msgs::msg::Point pt;
    int i = 0;
    for (auto it = keypoints.begin(); it != keypoints.end(); it++, i++) {
        auto kp = it.base();
        pt.x = kp->pt.x * this->scale_w * inv_imgscale;
        pt.y = kp->pt.y * this->scale_h * inv_imgscale;
        piMessage.tracked_points.push_back(pt);
    }

    this->pose->publish(rosPose);
    this->poseImgPub->publish(piMessage);
}

void
OrbSlamROS::CleanUp() {
    this->SLAM->Shutdown();
}

int main(int argc, const char* argv[]) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    
    rclcpp::init(argc, argv);

    rclcpp::executors::SingleThreadedExecutor exe;

    std::shared_ptr<OrbSlamROS> orbslam = std::make_shared<OrbSlamROS>();

    exe.add_node(orbslam->get_node_base_interface());

    exe.spin();

    orbslam->CleanUp();

    rclcpp::shutdown();

    return 0;
}