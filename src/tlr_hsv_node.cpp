#include "tlr_detector/tlr_hsv_node.hpp"

namespace tlr_detector
{

TlrHsvNode::TlrHsvNode()
: Node("tlr_hsv_node")
{
  RCLCPP_DEBUG(this->get_logger(), "Starting traffic_light_hsv_analyzer_node...");

  // Declare parameters for HSV ranges
  this->declare_parameter<int>("red_h_min1", 0);
  this->declare_parameter<int>("red_s_min1", 100);
  this->declare_parameter<int>("red_v_min1", 100);
  this->declare_parameter<int>("red_h_max1", 10);
  this->declare_parameter<int>("red_s_max1", 255);
  this->declare_parameter<int>("red_v_max1", 255);

  this->declare_parameter<int>("red_h_min2", 160);
  this->declare_parameter<int>("red_s_min2", 100);
  this->declare_parameter<int>("red_v_min2", 100);
  this->declare_parameter<int>("red_h_max2", 180);
  this->declare_parameter<int>("red_s_max2", 255);
  this->declare_parameter<int>("red_v_max2", 255);

  this->declare_parameter<int>("yellow_h_min", 20);
  this->declare_parameter<int>("yellow_s_min", 100);
  this->declare_parameter<int>("yellow_v_min", 100);
  this->declare_parameter<int>("yellow_h_max", 40);
  this->declare_parameter<int>("yellow_s_max", 255);
  this->declare_parameter<int>("yellow_v_max", 255);

  this->declare_parameter<int>("green_h_min", 50);
  this->declare_parameter<int>("green_s_min", 100);
  this->declare_parameter<int>("green_v_min", 100);
  this->declare_parameter<int>("green_h_max", 80);
  this->declare_parameter<int>("green_s_max", 255);
  this->declare_parameter<int>("green_v_max", 255);

  // Get parameter values
  red_min1_ = cv::Scalar(this->get_parameter("red_h_min1").as_int(), this->get_parameter("red_s_min1").as_int(), this->get_parameter("red_v_min1").as_int());
  red_max1_ = cv::Scalar(this->get_parameter("red_h_max1").as_int(), this->get_parameter("red_s_max1").as_int(), this->get_parameter("red_v_max1").as_int());
  red_min2_ = cv::Scalar(this->get_parameter("red_h_min2").as_int(), this->get_parameter("red_s_min2").as_int(), this->get_parameter("red_v_min2").as_int());
  red_max2_ = cv::Scalar(this->get_parameter("red_h_max2").as_int(), this->get_parameter("red_s_max2").as_int(), this->get_parameter("red_v_max2").as_int());

  yellow_min_ = cv::Scalar(this->get_parameter("yellow_h_min").as_int(), this->get_parameter("yellow_s_min").as_int(), this->get_parameter("yellow_v_min").as_int());
  yellow_max_ = cv::Scalar(this->get_parameter("yellow_h_max").as_int(), this->get_parameter("yellow_s_max").as_int(), this->get_parameter("yellow_v_max").as_int());

  green_min_ = cv::Scalar(this->get_parameter("green_h_min").as_int(), this->get_parameter("green_s_min").as_int(), this->get_parameter("green_v_min").as_int());
  green_max_ = cv::Scalar(this->get_parameter("green_h_max").as_int(), this->get_parameter("green_s_max").as_int(), this->get_parameter("green_v_max").as_int());

  // Subscribe to /traffic_light_image
  traffic_light_image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/traffic_light_image", rclcpp::QoS(1).best_effort(), std::bind(&TlrHsvNode::traffic_light_image_callback, this, std::placeholders::_1));

  // Publish /traffic_light_status
  traffic_light_status_publisher_ = this->create_publisher<std_msgs::msg::String>(
    "/traffic_light_status", 10);

  RCLCPP_DEBUG(this->get_logger(), "Subscribing to /traffic_light_image and publishing to /traffic_light_status.");
}

TlrHsvNode::~TlrHsvNode()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down traffic_light_hsv_analyzer_node...");
}

void TlrHsvNode::traffic_light_image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  RCLCPP_DEBUG(this->get_logger(), "Received traffic_light_image. Image size: %dx%d, encoding: %s", msg->width, msg->height, msg->encoding.c_str());

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
    RCLCPP_DEBUG(this->get_logger(), "Published traffic light status: %s", status.c_str());

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
    RCLCPP_DEBUG(this->get_logger(), "Detected RED (pixels: %d)", red_pixels);
    return "RED";
  } else if (yellow_pixels > red_pixels && yellow_pixels > green_pixels) {
    RCLCPP_DEBUG(this->get_logger(), "Detected YELLOW (pixels: %d)", yellow_pixels);
    return "YELLOW";
  } else if (green_pixels > red_pixels && green_pixels > yellow_pixels) {
    RCLCPP_DEBUG(this->get_logger(), "Detected GREEN (pixels: %d)", green_pixels);
    return "GREEN";
  } else {
    RCLCPP_DEBUG(this->get_logger(), "Detected UNKNOWN (red: %d, yellow: %d, green: %d)", red_pixels, yellow_pixels, green_pixels);
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
