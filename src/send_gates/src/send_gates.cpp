// stl
#include <chrono>
#include <cstdint>
#include <functional>
#include <string>
#include <unistd.h>
#include <vector>
// rclcpp and gazebo
#include "rclcpp/rclcpp.hpp"
// messages
#include "gazebo_msgs/msg/link_states.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/header.hpp"

using std::placeholders::_1;

class GatesPublisher : public rclcpp::Node
{
public:
  GatesPublisher() : Node("send_gates")
  {
    subscriber_ = this->create_subscription<gazebo_msgs::msg::LinkStates>(
        "link_states", 10, std::bind(&GatesPublisher::topic_callback, this, _1));
    auto qos   = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("gate_position", qos);
  }

private:
  void topic_callback(const gazebo_msgs::msg::LinkStates::SharedPtr msg) const
  {
    std::string gates_pattern("mindstorm_map::gate");

    std_msgs::msg::Header                 hh;
    geometry_msgs::msg::Pose              pose;
    std::vector<geometry_msgs::msg::Pose> pose_array;
    geometry_msgs::msg::PoseArray         pose_array_msg;

    hh.stamp    = this->now();
    hh.frame_id = "map";

    pose.position.z    = 0;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 0;

    for (std::size_t i = 0; i < msg->name.size(); ++i)
    {
      if (msg->name[i].find(gates_pattern) != std::string::npos)
      {
        pose.position.x = msg->pose[i].position.x;
        pose.position.y = msg->pose[i].position.y;
        pose_array.emplace_back(pose);
      }
    }
    pose_array_msg.header = hh;
    pose_array_msg.poses  = pose_array;

    while (1)
    {
      publisher_->publish(pose_array_msg);
      usleep(1000000);
    }
  }

  rclcpp::Subscription<gazebo_msgs::msg::LinkStates>::SharedPtr subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr   publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GatesPublisher>());
  rclcpp::shutdown();
  return 0;
}