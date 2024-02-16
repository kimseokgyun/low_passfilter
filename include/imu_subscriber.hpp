#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/node_utils.hpp"
#include "sensor_msgs/msg/imu.hpp"

/**
 * @class OdomSubscriber
 * Wrapper for some common odometry operations. Subscribes to the topic with a mutex.
 */

namespace piot_filter_utils
{
class IMUSubscriber
{
public:
  /**
   * @brief Constructor that subscribes to an Odometry topic
   *
   * @param nh NodeHandle for creating subscriber
   * @param default_topic Name of the topic that will be loaded of the odom_topic param is not set.
   */
  explicit IMUSubscriber(
    nav2_util::LifecycleNode::SharedPtr nh,
    std::string default_topic = "/imu")
  {
    nav2_util::declare_parameter_if_not_declared(
      nh, "/imu", rclcpp::ParameterValue(default_topic));

    std::string topic;
    nh->get_parameter_or("/imu", topic, default_topic);
    imu_sub_ =
      nh->create_subscription<sensor_msgs::msg::Imu>(
      topic,
      rclcpp::SystemDefaultsQoS(),
      std::bind(&IMUSubscriber::IMUCallback, this, std::placeholders::_1));
  }

  inline sensor_msgs::msg::Imu getIMU() {return current_imu;}

protected:
  void IMUCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
 
    std::lock_guard<std::mutex> lock(status_mutex_);
    current_imu = *msg;     
  }

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  sensor_msgs::msg::Imu current_imu;
  std::mutex status_mutex_;
};
}


