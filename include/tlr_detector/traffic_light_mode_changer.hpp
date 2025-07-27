#ifndef TLR_DETECTOR__TRAFFIC_LIGHT_MODE_CHANGER_HPP_
#define TLR_DETECTOR__TRAFFIC_LIGHT_MODE_CHANGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <autoware_adapi_v1_msgs/srv/change_operation_mode.hpp>

namespace tlr_detector
{

class TrafficLightModeChanger : public rclcpp::Node
{
public:
  TrafficLightModeChanger();

private:
  void traffic_light_status_callback(const std_msgs::msg::String::SharedPtr msg);
  void call_change_operation_mode_service();

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr traffic_light_status_sub_;
  rclcpp::Client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>::SharedPtr auto_mode_client_;
};

}  // namespace tlr_detector

#endif  // TLR_DETECTOR__TRAFFIC_LIGHT_MODE_CHANGER_HPP_
