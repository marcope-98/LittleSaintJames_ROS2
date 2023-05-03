// stl
#include <cstdint>
#include <functional>
#include <string>
#include <vector>
// rclcpp and gazebo
#include "rclcpp/rclcpp.hpp"
// messages
#include "gazebo_msgs/msg/link_states.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "obstacles_msgs/msg/obstacle_array_msg.hpp"
#include "obstacles_msgs/msg/obstacle_msg.hpp"

using std::placeholders::_1;

// TODO: prolly this is wise to make it a service so we dont keep listening to the topic
class ObstaclePublisher : public rclcpp::Node
{
public:
  ObstaclePublisher() : Node("obstacle_publisher")
  {
    auto qos    = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    publisher_  = this->create_publisher<obstacles_msgs::msg::ObstacleArrayMsg>("obstacles", qos);
    subscriber_ = this->create_subscription<gazebo_msgs::msg::LinkStates>(
        "link_states", 10, std::bind(&ObstaclePublisher::topic_callback, this, _1));
  }

private:
  void topic_callback(const gazebo_msgs::msg::LinkStates::SharedPtr msg) const
  {
    std::string obs_pattern("mindstorm_map::obstacle_");
    std::string link_pattern("::link");

    geometry_msgs::msg::Point32           point;
    geometry_msgs::msg::Polygon           pol;
    obstacles_msgs::msg::ObstacleMsg      obs_msg;
    obstacles_msgs::msg::ObstacleArrayMsg obs_arr_msg;

    std::vector<geometry_msgs::msg::Point32>      points;
    std::vector<obstacles_msgs::msg::ObstacleMsg> obstacles;

    point.z = 0;

    // WARN: this is really hard to read.
    for (std::size_t i = 0; i < msg->name.size(); ++i)
    {
      if (msg->name[i].find(obs_pattern) != std::string::npos)
      {
        if (msg->name[i].find(link_pattern) != std::string::npos)
        {
          if (!points.empty())
          {
            pol.points      = points;
            obs_msg.polygon = pol;
            obstacles.emplace_back(obs_msg);
            points.clear();
          }
        }
        else
        {
          point.x = msg->pose[i].position.x;
          point.y = msg->pose[i].position.y;
          points.emplace_back(point);
        }
      }
    }

    if (!points.empty())
    {
      pol.points      = points;
      obs_msg.polygon = pol;
      obstacles.emplace_back(obs_msg);
      points.clear();
    }

    obs_arr_msg.obstacles = obstacles;

    while (1)
    {
      publisher_->publish(obs_arr_msg);
      usleep(1000000);
    }
  }

  rclcpp::Publisher<obstacles_msgs::msg::ObstacleArrayMsg>::SharedPtr publisher_;
  rclcpp::Subscription<gazebo_msgs::msg::LinkStates>::SharedPtr       subscriber_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstaclePublisher>());
  rclcpp::shutdown();
  return 0;
}