#include "orb-slam-port/orb_slam_ros.hpp"

#include <opencv2/core/core.hpp>
#include "Tracking.h"

typedef ORB_SLAM3::Tracking::eTrackingState TrackingState;

OrbSlamROS::OrbSlamROS() : rclcpp::Node("orb_slam3") {
    this->decalre_parameters();
    this->setup_topics();

    std::string voc = this->get_parameter("orb_voc").as_string();
    std::string cam_cfg = this->get_parameter("cam_cfg").as_string();

    RCLCPP_INFO_STREAM(this->get_logger(), "vocabulary: " << voc);
    RCLCPP_INFO_STREAM(this->get_logger(), "camera config: " << cam_cfg);

    this->SLAM = new ORB_SLAM3::System(voc, cam_cfg, ORB_SLAM3::System::MONOCULAR, true);
    this->imgScale = this->SLAM->GetImageScale();

    this->create_subscription<sensor_msgs::msg::Image>("rgbd_in", 5, std::bind(&OrbSlamROS::cv2_feed_spin, this, std::placeholders::_1));

}

void 
OrbSlamROS::decalre_parameters() {
    this->declare_parameter("orb_voc", "");
    this->declare_parameter("cam_cfg", "");
}

void 
OrbSlamROS::setup_topics() {
    this->poseImgPub = this->create_publisher<auto_scanner::msg::PosedImage>("/orb_slam3/posed_image", 5);
    this->poseImgPubLow = this->create_publisher<auto_scanner::msg::PosedImage>("/orb_slam3/pose_image_lfreq", 5);
}

void 
OrbSlamROS::cv2_feed_spin(const sensor_msgs::msg::Image& image) {
    cv_bridge::CvImagePtr rgb = cv_bridge::toCvCopy(image);
    cv::Mat frame = rgb->image;

    if (this->imgScale != 1.f) {
        int w = frame.cols * this->imgScale;
        int h = frame.rows * this->imgScale;
        cv::resize(frame, frame, cv::Size(w, h));
    }

    double sec = image.header.stamp.nanosec * 1e-9;
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