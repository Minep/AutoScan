#include "orb-slam-port/orb_slam_ros.hpp"

#include <opencv2/core/core.hpp>

OrbSlamROS::OrbSlamROS() : rclcpp::Node("orb_slam3") {
    this->decalre_parameters();
    RCLCPP_INFO_STREAM(this->get_logger(), "Created");

    std::string voc = this->get_parameter("orb_voc").as_string();
    std::string cam_cfg = this->get_parameter("cam_cfg").as_string();
    this->fps = this->get_parameter("fps").as_int();
    this->frame_interval = 1 / this->fps;
    this->accumulatedT = Eigen::Matrix4f::Identity();

    RCLCPP_INFO_STREAM(this->get_logger(), "vocabulary: " << voc);
    RCLCPP_INFO_STREAM(this->get_logger(), "camera config: " << cam_cfg);

    this->SLAM = new ORB_SLAM3::System(voc, cam_cfg, ORB_SLAM3::System::MONOCULAR, true);
    this->imgScale = this->SLAM->GetImageScale();

    this->setup_topics();

    this->init_opencv_source();

    int64_t interval = 1000 / this->fps;

    this->timer = this->create_wall_timer(
        std::chrono::milliseconds(interval)
        , std::bind(&OrbSlamROS::cv2_feed_spin, this));
}

void 
OrbSlamROS::decalre_parameters() {
    this->declare_parameter("vid_dev", "/dev/video0");
    this->declare_parameter("orb_voc", "");
    this->declare_parameter("cam_cfg", "");
    this->declare_parameter("fps", 10);
    this->declare_parameter("rescale_w", 720);
    this->declare_parameter("rescale_h", 480);
    this->declare_parameter("video_file", "");
}

void 
OrbSlamROS::setup_gst_capture() {
    char buffer[256];
    std::string vdev = this->get_parameter("vid_dev").as_string();
    std::string gstfmt = 
        "v4l2src device=%s ! "
        "videoconvert ! videorate ! videoscale ! "
        "video/x-raw, framerate=%d/1, width=%d, height=%d ! "
        "appsink drop=1";
    
    int rescale_w = this->get_parameter("rescale_w").as_int();
    int rescale_h = this->get_parameter("rescale_h").as_int();

    int n = std::snprintf(buffer, sizeof(buffer), 
        gstfmt.c_str(), vdev.c_str(), this->fps,
        rescale_w, rescale_h);
    
    std::string gststr = std::string(buffer, n);
    this->videoCapture = new cv::VideoCapture(gststr, cv::CAP_GSTREAMER);

    RCLCPP_INFO_STREAM(this->get_logger(), "Gst: " << gststr);
}

void
OrbSlamROS::setup_file_capture(const std::string& file) {
    char buffer[256];
    std::string gstfmt = 
        "filesrc location=%s ! "
        "videorate ! "
        "video/x-raw, framerate=%d/1 ! "
        "appsink drop=1";
    int n = std::snprintf(buffer, sizeof(buffer), 
        file.c_str(), this->fps);
    
    std::string gststr = std::string(buffer, n);
    this->videoCapture = new cv::VideoCapture(gststr, cv::CAP_GSTREAMER);

    RCLCPP_INFO_STREAM(this->get_logger(), "Gst: " << gststr);
}

void
OrbSlamROS::init_opencv_source() {
    std::string file = this->get_parameter("video_file").as_string();
    if (file.empty()) {
        this->setup_gst_capture();
    }
    else {
        this->setup_file_capture(file);
    }

    if (!this->videoCapture->isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Error opening capture.");
    }
}

void 
OrbSlamROS::setup_topics() {
    this->poseImgPub = this->create_publisher<auto_scanner::msg::PosedImage>("pose_image", 5);
}

void 
OrbSlamROS::cv2_feed_spin() {
    if (!this->videoCapture->isOpened()) {
        RCLCPP_ERROR_STREAM(this->get_logger(), "Gst closed.");
        return;
    }

    if (!this->videoCapture->read(this->currentFrame)) {
        RCLCPP_WARN(this->get_logger(), "Empty frame!");
        return;
    }
    
    this->frame_id++;

    if (this->imgScale != 1.f) {
        int w = this->currentFrame.cols * this->imgScale;
        int h = this->currentFrame.rows * this->imgScale;
        cv::resize(this->currentFrame, this->currentFrame, cv::Size(w, h));
    }

    std_msgs::msg::Header header;
    header.frame_id = this->frame_id;
    header.stamp = this->now();

    auto pose = this->SLAM->TrackMonocular(this->currentFrame, header.stamp.sec);
    // this->accumulatedT = pose.matrix() * this->accumulatedT;
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

    cv_bridge::CvImage cvImg(header, sensor_msgs::image_encodings::RGB8, this->currentFrame);

    auto_scanner::msg::PosedImage piMessage = auto_scanner::msg::PosedImage();
    cvImg.toImageMsg(piMessage.image);
    piMessage.cam_pose = rosPose;

    this->poseImgPub->publish(piMessage);
}

void
OrbSlamROS::CleanUp() {
    this->SLAM->Shutdown();
    this->videoCapture->release();
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