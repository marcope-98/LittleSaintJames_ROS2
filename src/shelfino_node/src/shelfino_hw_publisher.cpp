#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "std_srvs/srv/set_bool.hpp"

#include "hardwareparameters.h"
#include "hardwareglobalinterface.h"
#include "json.hpp"

using namespace std::chrono_literals;
using namespace nlohmann;

/// @brief Variable for definition of the shelfino ID number
const auto R_ID_DEFAULT = 2;

/// @brief Pointer to a Hardware Parameters instance which contains all the information on the IP addresses and ports of the specific shelfino robot
std::unique_ptr<HardwareParameters> hp;

/**
 * @brief Shelfino ROS2 node to bridge the sensor data from ZMQ to ROS2.
 * 
 * ROS 2 interface for the mobile robot Shelfino of the Department of Information Engineering and Computer Science of the University of Trento.
 */
class ShelfinoHWNode : public rclcpp::Node
{
  public:
    /**
     * @brief Construct a new Shelfino HW Node object.
     * 
     * Create all the ROS2 tranform broadcasters, topic publishers and subscribers.
     */
    ShelfinoHWNode()
    : Node("shelfino_hw_publisher")
    {
      auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);

      // Load shelfino paths to communicate with ZMQ
      hp = std::make_unique<HardwareParameters>(R_ID_DEFAULT);
      HardwareGlobalInterface::initialize(hp.get());
      // ROS2 transform broadcaster
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      // Creation of ROS2 publishers
      lidar_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", qos);
      t265_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("t265", qos);
      odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", qos);
      encoders_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", qos);
      // Selectiong the callbacks for the publishers
      lidar_timer_ = this->create_wall_timer(100ms, std::bind(&ShelfinoHWNode::lidar_callback, this));
      t265_timer_ = this->create_wall_timer(100ms, std::bind(&ShelfinoHWNode::t265_callback, this));
      odom_timer_ = this->create_wall_timer(100ms, std::bind(&ShelfinoHWNode::odom_callback, this));
      encoders_timer_ = this->create_wall_timer(100ms, std::bind(&ShelfinoHWNode::enc_callback, this));
      // Creation of the CMD_VEL subscriber to move the shelfino
      cmd_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, std::bind(&ShelfinoHWNode::handle_shelfino_cmd_vel, this, std::placeholders::_1));
      // Retrieve node namespace to use as prefix of transforms
      ns = this->get_namespace();
      ns.erase(0,1);
      
      service_ = this->create_service<std_srvs::srv::SetBool>("power", std::bind(&ShelfinoHWNode::handle_power_srv, this, std::placeholders::_1, std::placeholders::_2));
    }

  private:
    /**
     * @brief Method callback that retrieves the lidar scan data from ZMQ and publishes to ROS
     * 
     */
    void lidar_callback()
    {
      RobotStatus::LidarData lidarData; 
      HardwareGlobalInterface::getInstance().getFrontLidarData(lidarData);

      sensor_msgs::msg::LaserScan msg;
      msg.header.stamp = this->get_clock()->now();
      
      msg.header.frame_id = "base_laser";
      msg.angle_increment = 0.00872664625;
      msg.angle_min = msg.angle_increment;
      msg.angle_max = 6.27445866092 + msg.angle_min;
      //msg.time_increment = (1./10.) / lidarData.datum.size();
      //msg.scan_time = 1./10.;
      msg.range_min = 0.05;
      msg.range_max = 10;

      std::vector<float> data;
      for(int i=0;i<lidarData.datum.size();i++){
        data.push_back(lidarData.datum.at(i).distance);
      }

      msg.ranges = data;

      lidar_publisher_->publish(msg);
    }

    /**
     * @brief Method callback that retrieves the RealSense odometry data from ZMQ and publishes to ROS
     * 
     */
    void t265_callback()
    {
      RobotStatus::OdometryData odomData; 
      HardwareGlobalInterface::getInstance().getOdomData(odomData);

      nav_msgs::msg::Odometry msg;
      msg.header.stamp = this->get_clock()->now();
      
      msg.header.frame_id = "odom";
      msg.child_frame_id = "base_link";

      msg.pose.pose.position.x = odomData.pos_x; 
      msg.pose.pose.position.y = odomData.pos_y;
      msg.pose.pose.position.z = odomData.pos_z;
      msg.pose.pose.orientation.x = odomData.orient_x;
      msg.pose.pose.orientation.y = odomData.orient_y;
      msg.pose.pose.orientation.z = odomData.orient_z;
      msg.pose.pose.orientation.w = odomData.orient_w;

      msg.twist.twist.linear.x = odomData.twist_lin_x;
      msg.twist.twist.linear.y = odomData.twist_lin_y;
      msg.twist.twist.linear.z = odomData.twist_lin_z;
      msg.twist.twist.angular.x = odomData.twist_ang_x;
      msg.twist.twist.angular.y = odomData.twist_ang_y;
      msg.twist.twist.angular.z = odomData.twist_ang_z;

      for (int i=0; i<odomData.pose_cov.size(); i++) {
        msg.pose.covariance[i] = odomData.pose_cov.at(i);
        msg.twist.covariance[i] = odomData.twist_cov.at(i);
      }

      t265_publisher_->publish(msg);
    }

    /**
     * @brief Method callback that retrieves odometry data from ZMQ and publishes to ROS
     * 
     * The odometry is calculated from the sensor fusion of encoders and RealSense data
     */
    void odom_callback()
    {
      RobotStatus::OdometryData odomData; 
      HardwareGlobalInterface::getInstance().getOdomData(odomData);

      nav_msgs::msg::Odometry msg;
      msg.header.stamp = this->get_clock()->now();

      msg.header.frame_id = "odom";
      msg.child_frame_id = "base_link";

      msg.pose.pose.position.x = odomData.pos_x; 
      msg.pose.pose.position.y = odomData.pos_y;
      msg.pose.pose.position.z = odomData.pos_z;
      msg.pose.pose.orientation.x = odomData.orient_x;
      msg.pose.pose.orientation.y = odomData.orient_y;
      msg.pose.pose.orientation.z = odomData.orient_z;
      msg.pose.pose.orientation.w = odomData.orient_w;

      msg.twist.twist.linear.x = odomData.twist_lin_x;
      msg.twist.twist.linear.y = odomData.twist_lin_y;
      msg.twist.twist.linear.z = odomData.twist_lin_z;
      msg.twist.twist.angular.x = odomData.twist_ang_x;
      msg.twist.twist.angular.y = odomData.twist_ang_y;
      msg.twist.twist.angular.z = odomData.twist_ang_z;

      for (int i=0; i<odomData.pose_cov.size(); i++) {
        msg.pose.covariance[i] = odomData.pose_cov.at(i);
        msg.twist.covariance[i] = odomData.twist_cov.at(i);
      }

      odom_publisher_->publish(msg);
      handle_shelfino_pose(msg);
    }

    /**
     * @brief Method callback that retrieves the encoders data from ZMQ and publishes to ROS
     * 
     */
    void enc_callback()
    {
      RobotStatus::HardwareData hwData; 
      HardwareGlobalInterface::getInstance().getHardwareData(hwData);

      sensor_msgs::msg::JointState msg;
      msg.header.stamp = this->get_clock()->now();

      msg.header.frame_id = "";

      msg.name.push_back("wheel_left_joint");
      msg.position.push_back(hwData.leftWheel.ticks);
      msg.velocity.push_back(hwData.leftWheel.omega);
      msg.name.push_back("wheel_right_joint");
      msg.position.push_back(hwData.rightWheel.ticks);
      msg.velocity.push_back(hwData.rightWheel.omega);

      encoders_publisher_->publish(msg);
    }

    /**
     * @brief Method that handles the velocities commands from ROS to ZMQ.
     * 
     * @param msg The velocity mesasge used as control action for the robot.
     */
    void handle_shelfino_cmd_vel(const std::shared_ptr<geometry_msgs::msg::Twist> msg)
    {
      double v = 0., omega = 0.;
      v = msg->linear.x;
      omega = msg->angular.z;

      HardwareGlobalInterface::getInstance().vehicleMove(v,omega);

      return;
    }

    /**
     * @brief Method that broadcasts the transform from the "odom" frame to "base_link" frame.
     * 
     * @param msg The odometry message containing the information about the robot's location.
     */
    void handle_shelfino_pose(nav_msgs::msg::Odometry msg)
    {
      geometry_msgs::msg::TransformStamped t;

      t.header.stamp = msg.header.stamp;

      t.header.frame_id = "odom";
      t.child_frame_id = "base_link";

      t.transform.translation.x = msg.pose.pose.position.x;
      t.transform.translation.y = msg.pose.pose.position.y;
      t.transform.translation.z = 0.0;
      t.transform.rotation.x = msg.pose.pose.orientation.x;
      t.transform.rotation.y = msg.pose.pose.orientation.y;
      t.transform.rotation.z = msg.pose.pose.orientation.z;
      t.transform.rotation.w = msg.pose.pose.orientation.w;

      // Send the transformation
      tf_broadcaster_->sendTransform(t);
    }

    void handle_power_srv(std_srvs::srv::SetBool::Request::SharedPtr request,
                          std_srvs::srv::SetBool::Response::SharedPtr response)
    {
      if(request->data){
        HardwareGlobalInterface::getInstance().robotOnVelControl();
        response->message = ns+" motors powered on";
        RCLCPP_INFO(this->get_logger(), "motors powered on");
      } else {
        HardwareGlobalInterface::getInstance().robotOff();
        response->message = ns+" motors powered off";
        RCLCPP_INFO(this->get_logger(), "motors powered off");
      }
      response->success = true;
    }
    
    std::string ns;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr lidar_timer_;
    rclcpp::TimerBase::SharedPtr t265_timer_;
    rclcpp::TimerBase::SharedPtr odom_timer_;
    rclcpp::TimerBase::SharedPtr encoders_timer_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr lidar_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr t265_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr encoders_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscription_;

    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;
};

/**
 * @brief Main function that initialize the ROS node
 * 
 * @param argc Number of arguments passed from the command line
 * @param argv Array of arguments passed from the command line
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ShelfinoHWNode>());

  rclcpp::shutdown();
  return 0;
}