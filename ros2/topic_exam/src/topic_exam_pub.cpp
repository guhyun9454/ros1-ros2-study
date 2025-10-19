#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class PublisherNode : public rclcpp::Node
{
public:
    PublisherNode()
    : Node("publisher_node"), count_(0)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("my_ros2_topic", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&PublisherNode::timer_callback, this));
    }

    void timer_callback()
    {
        message_.data = "My name is GuHyeon" + std::to_string(count_++);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message_.data.c_str());
        publisher_->publish(message_);
    }

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    std_msgs::msg::String message_;
    size_t count_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}