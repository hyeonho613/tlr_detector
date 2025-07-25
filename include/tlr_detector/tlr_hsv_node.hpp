#ifndef TLR_DETECTOR__TLR_HSV_NODE_HPP_
#define TLR_DETECTOR__TLR_HSV_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/string.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <string>
#include <algorithm>

namespace tlr_detector
{

class TlrHsvNode : public rclcpp::Node
{
public:
  TlrHsvNode();
  ~TlrHsvNode();

private:
  // Callback for /traffic_light_image topic
  void traffic_light_image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  // Analyzes HSV image and determines traffic light color
  std::string analyze_hsv(const cv::Mat& hsv_image);

  // Sub: /traffic_light_image
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr traffic_light_image_subscription_;
  // Pub: /traffic_light_status
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr traffic_light_status_publisher_;

  // HSV color ranges for traffic light detection
  // These values are based on the autoware code and can be tuned
  cv::Scalar red_min1_, red_max1_;   // For red (hue wraps around)
  cv::Scalar red_min2_, red_max2_;   // For red (hue wraps around)
  cv::Scalar yellow_min_, yellow_max_;
  cv::Scalar green_min_, green_max_;
};

} // namespace tlr_detector

#endif // ROS2_MULTI_LAUNCHER__TLR_HSV_NODE_HPP_
