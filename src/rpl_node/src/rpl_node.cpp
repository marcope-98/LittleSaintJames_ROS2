#include <functional>
#include <memory>
#include <string>
//
#include "rclcpp/rclcpp.hpp"

#include "rpl/interface.hpp"
#include "std_msgs/msg/float32.hpp"

using std::placeholders::_1;

class RPLNode : public rclcpp::Node
{
public:
  RPLNode() : Node("rpl_node")
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    pub_     = this->create_publisher<std_msgs::msg::Float32>("output_topic", qos);
    sub_     = create_subscription<std_msgs::msg::Float32>(
        "input_topic", 10, std::bind(&RPLNode::topic_callback, this, _1));
  }
  void topic_callback(const std_msgs::msg::Float32 &msg) const
  {
    rpl::interface::planPath();
  }

private:
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr    pub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RPLNode>();
  RCLCPP_INFO(node->get_logger(), "RPL node started.");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}