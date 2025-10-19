#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class SubscriberNode : public rclcpp::Node
{
public:
    SubscriberNode()
    : Node("subscriber_node")
    {
        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "my_ros2_topic",
            10,
            std::bind(&SubscriberNode::callback, this, _1)
        );
    }

    void callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SubscriberNode>());
    rclcpp::shutdown();
    return 0;
}