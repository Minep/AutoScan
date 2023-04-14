import rclpy
from rclpy.node import Node

from auto_scanner.srv import NodeReady

class HeathReportingNode(Node):
    def __init__(self, name, *args, **kwargs):
        super().__init__(node_name=name, *args, **kwargs)

        self._heath_report = self.create_client(NodeReady, "/control_panel/inform_ready")

        self._heath_report.wait_for_service(30)
        if not self._heath_report.service_is_ready():
            self.get_logger().error("Health monitor is down")

    def _inform_ready(self):
        req = NodeReady.Request()
        req.node_name = self.get_name()
        future = self._heath_report.call_async(req)

        rclpy.spin_until_future_complete(self, future)
        result: NodeReady.Response = future.result()
        
        if (result.ack_name != req.node_name):
            self.get_logger().fatal("I am the unexpected node!")