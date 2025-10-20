#include "rclcpp/rclcpp.hpp"
#include "service_exam2/srv/my_service.hpp"

#include <memory>

#define PLUS 1
#define MINUS 2
#define MULTIPLICATION 3
#define DIVISION 4

int opt=1;

void server_callback(const std::shared_ptr<service_exam2::srv::MyService::Request> request,
  std::shared_ptr<service_exam2::srv::MyService::Response> response)
{
  if (opt == 1) {
    response->result = request->a + request->b;
  } else if (opt == 2) {
    response->result = request->a - request->b;
  } else if (opt == 3) {
    response->result = request->a * request->b;
  } else if (opt == 4) {
    response->result = request->a / request->b;
  } else {
    RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Invalid operation code: %d. Setting result to 0.", opt);
    response->result = 0;
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %lld b: %lld", (long long)request->a, (long long)request->b);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%lld]", (long long)response->result);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("service_exam_server2");
  node-> declare_parameter("calculation_method",PLUS);

  rclcpp::Service<service_exam2::srv::MyService>::SharedPtr service =
    node->create_service<service_exam2::srv::MyService>("my_service", &server_callback);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to calculate two ints.");

  rclcpp::WallRate r(10);
  while(rclcpp::ok()){
    node->get_parameter("calculation_method",opt);
    rclcpp::spin_some(node);
    r.sleep();
  }
  rclcpp::shutdown();
}

