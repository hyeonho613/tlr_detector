#include "tlr_detector/traffic_light_mode_changer.hpp"

namespace tlr_detector
{

TrafficLightModeChanger::TrafficLightModeChanger()
: Node("traffic_light_mode_changer")
{
  traffic_light_status_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/traffic_light_status", 10,
    std::bind(&TrafficLightModeChanger::traffic_light_status_callback, this, std::placeholders::_1));

  auto_mode_client_ = this->create_client<autoware_adapi_v1_msgs::srv::ChangeOperationMode>(
    "/api/operation_mode/change_to_autonomous");

  RCLCPP_INFO(this->get_logger(), "TrafficLightModeChanger node has been started.");
}

void TrafficLightModeChanger::traffic_light_status_callback(const std_msgs::msg::String::SharedPtr msg)
{
  if (msg->data == "GREEN") {
    RCLCPP_INFO(this->get_logger(), "Traffic light is green. Calling ChangeOperationMode service.");
    call_change_operation_mode_service();
  }
}

void TrafficLightModeChanger::call_change_operation_mode_service()
{
  if (!auto_mode_client_->wait_for_service(std::chrono::seconds(1)))
  {
    RCLCPP_ERROR(this->get_logger(), "Service /api/operation_mode/change_to_autonomous not available.");
    return;
  }

  std::shared_ptr<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request>request = 
      std::make_shared<autoware_adapi_v1_msgs::srv::ChangeOperationMode::Request>();
    auto result_future = auto_mode_client_->async_send_request(request);

    // Add a callback to handle the service response
    rclcpp::Rate rate(10); // 10 Hz
    while (rclcpp::ok() && result_future.wait_for(rate.period()) == std::future_status::timeout) {
      rclcpp::spin_some(this->get_node_base_interface());
    }

    if (result_future.valid()) {
      auto response = result_future.get();
      if (response) {
        RCLCPP_INFO(this->get_logger(), "Service call successful. Response: %s", response->status.message.c_str());
      } else {
        RCLCPP_ERROR(this->get_logger(), "Service call failed: Empty response.");
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "Service call failed: Invalid future.");
    }
}

}  // namespace tlr_detector

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tlr_detector::TrafficLightModeChanger>());
  rclcpp::shutdown();
  return 0;
}
