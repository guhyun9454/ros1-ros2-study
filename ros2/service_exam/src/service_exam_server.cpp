#include "rclcpp/rclcpp.hpp"
#include "service_exam/srv/my_service.hpp"

#include <memory>

void server_callback(const std::shared_ptr<service_exam::srv::MyService::Request> request,
  std::shared_ptr<service_exam::srv::MyService::Response> response)
{
  response->result = request->a + request->b;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
                request->a, request->b);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->result);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("service_exam_server");

  rclcpp::Service<service_exam::srv::MyService>::SharedPtr service =
    node->create_service<service_exam::srv::MyService>("my_service", &server_callback);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to calculate two ints.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}

