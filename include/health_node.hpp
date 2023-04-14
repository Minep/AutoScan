#ifndef BE942AC5_275B_401B_BA3E_6A55777620C9
#define BE942AC5_275B_401B_BA3E_6A55777620C9

#include "rclcpp/rclcpp.hpp"
#include "auto_scanner/srv/node_ready.hpp"

class HealthReportingNode : public rclcpp::Node {
private:
    rclcpp::Client<auto_scanner::srv::NodeReady>::SharedPtr health_report;
    auto_scanner::srv::NodeReady_Request::SharedPtr req;

    void health_service_callback(rclcpp::Client<auto_scanner::srv::NodeReady>::SharedFuture future);

public:
    explicit HealthReportingNode(const std::string& name, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

protected:
    void inform_ready();
};

#endif /* BE942AC5_275B_401B_BA3E_6A55777620C9 */
