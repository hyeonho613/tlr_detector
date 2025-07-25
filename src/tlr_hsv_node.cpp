#include "tlr_detector/tlr_hsv_node.hpp"

namespace tlr_detector
{

TlrHsvNode::TlrHsvNode()
: Node("tlr_hsv_node")
{
  RCLCPP_INFO(this->get_logger(), "Starting traffic_light_hsv_analyzer_node...");

  // Subscribe to /traffic_light_image
  traffic_light_image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/traffic_light_image", 10, std::bind(&TlrHsvNode::traffic_light_image_callback, this, std::placeholders::_1));

  // Publish /traffic_light_status
  traffic_light_status_publisher_ = this->create_publisher<std_msgs::msg::String>(
    "/traffic_light_status", 10);

  // Initialize HSV color ranges (tuned from autoware example)
  // Red (hue wraps around, so two ranges are needed)
  red_min1_ = cv::Scalar(0, 100, 100);
  red_max1_ = cv::Scalar(10, 255, 255);
  red_min2_ = cv::Scalar(160, 100, 100);
  red_max2_ = cv::Scalar(180, 255, 255);

  // Yellow
  yellow_min_ = cv::Scalar(20, 100, 100);
  yellow_max_ = cv::Scalar(40, 255, 255);

  // Green
  green_min_ = cv::Scalar(50, 100, 100);
  green_max_ = cv::Scalar(80, 255, 255);

  RCLCPP_INFO(this->get_logger(), "Subscribing to /traffic_light_image and publishing to /traffic_light_status.");
}

TlrHsvNode::~TlrHsvNode()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down traffic_light_hsv_analyzer_node...");
}

void TlrHsvNode::traffic_light_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received traffic_light_image. Image size: %dx%d, encoding: %s", msg->width, msg->height, msg->encoding.c_str());

  try {
    cv_bridge::CvImagePtr cv_ptr;
    // Convert to BGR8 for consistent OpenCV processing
    cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

    if (cv_ptr->image.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty image.");
      return;
    }

    cv::Mat hsv_image;
    cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

    std::string status = analyze_hsv(hsv_image);

    std_msgs::msg::String status_msg;
    status_msg.data = status;
    traffic_light_status_publisher_->publish(status_msg);
    RCLCPP_INFO(this->get_logger(), "Published traffic light status: %s", status.c_str());

  } catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
  } catch (const cv::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
  }
}

std::string TlrHsvNode::analyze_hsv(const cv::Mat& hsv_image)
{
  // Count pixels within each color range
  cv::Mat red_mask1, red_mask2, red_mask, yellow_mask, green_mask;

  cv::inRange(hsv_image, red_min1_, red_max1_, red_mask1);
  cv::inRange(hsv_image, red_min2_, red_max2_, red_mask2);
  cv::bitwise_or(red_mask1, red_mask2, red_mask);

  cv::inRange(hsv_image, yellow_min_, yellow_max_, yellow_mask);
  cv::inRange(hsv_image, green_min_, green_max_, green_mask);

  int red_pixels = cv::countNonZero(red_mask);
  int yellow_pixels = cv::countNonZero(yellow_mask);
  int green_pixels = cv::countNonZero(green_mask);

  // Determine the dominant color
  if (red_pixels > yellow_pixels && red_pixels > green_pixels) {
    return "RED";
  } else if (yellow_pixels > red_pixels && yellow_pixels > green_pixels) {
    return "YELLOW";
  } else if (green_pixels > red_pixels && green_pixels > yellow_pixels) {
    return "GREEN";
  } else {
    return "UNKNOWN"; // Or OFF
  }
}

} // namespace tlr_detector

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tlr_detector::TlrHsvNode>());
  rclcpp::shutdown();
  return 0;
}
