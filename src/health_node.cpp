#include "health_node.hpp"
#include <chrono>

HealthReportingNode::HealthReportingNode(const std::string& name, const rclcpp::NodeOptions& options)
    : rclcpp::Node(name, options) {

    this->health_report = this->create_client<auto_scanner::srv::NodeReady>("/control_panel/inform_ready");
    this->req = std::make_shared<auto_scanner::srv::NodeReady_Request>();
    this->req->node_name = std::string(this->get_name());

    this->health_report->wait_for_service(std::chrono::seconds(30));
    if (!this->health_report->service_is_ready()) {
        RCLCPP_ERROR(this->get_logger(), "Health monitor is down!");
    }
}

void HealthReportingNode::inform_ready() {
    auto future = 
        this->health_report->async_send_request(
            this->req,
            std::bind(&HealthReportingNode::health_service_callback, this, std::placeholders::_1));
}

void 
HealthReportingNode::health_service_callback(rclcpp::Client<auto_scanner::srv::NodeReady>::SharedFuture future) {
    auto status = future.wait_for(std::chrono::seconds(1));
    if (status == std::future_status::ready) {
        auto result = future.get();
        if (result->ack_name != this->req->node_name) {
            RCLCPP_FATAL(this->get_logger(), "I am unexpected node.");
        }
    }
    else {
        RCLCPP_FATAL(this->get_logger(), "Heath checker disappeared.");
    }
}