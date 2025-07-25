#include "tlr_detector/tlr_detector_node.hpp"

namespace tlr_detector
{

TlrDetectorNode::TlrDetectorNode()
: Node("tlr_detector_node")
{
  RCLCPP_INFO(this->get_logger(), "Starting image_processor_node...");

  yolo_result_subscription_ = this->create_subscription<ultralytics_ros::msg::YoloResult>(
    "/yolo_result", 10, std::bind(&TlrDetectorNode::yolo_result_callback, this, std::placeholders::_1));

  traffic_light_bbox_publisher_ = this->create_publisher<vision_msgs::msg::BoundingBox2D>(
    "/traffic_light_bbox", 10);

  camera_raw_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/image_raw", 10, std::bind(&TlrDetectorNode::camera_raw_callback, this, std::placeholders::_1));

  traffic_light_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
    "/traffic_light_image", 10);

  RCLCPP_INFO(this->get_logger(), "Subscribing to /yolo_result and /image_raw.");
}

TlrDetectorNode::~TlrDetectorNode()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down image_processor_node...");
}

void TlrDetectorNode::yolo_result_callback(const ultralytics_ros::msg::YoloResult::SharedPtr msg)
{
  bool traffic_light_found = false;
  for (const auto& detection : msg->detections.detections) {
    for (const auto& hypothesis : detection.results) {
      if (hypothesis.hypothesis.class_id == "traffic light") {
        RCLCPP_INFO(this->get_logger(), "Found traffic light! Publishing bbox.");
        traffic_light_bbox_publisher_->publish(detection.bbox);

        std::lock_guard<std::mutex> lock(bbox_mutex_);
        latest_traffic_light_bbox_ = detection.bbox;
        traffic_light_found = true;
        break; // Only need one traffic light
      }
    }
    if (traffic_light_found) {
      break;
    }
  }

  if (!traffic_light_found) {
    std::lock_guard<std::mutex> lock(bbox_mutex_);
    latest_traffic_light_bbox_ = vision_msgs::msg::BoundingBox2D(); // Reset bbox if no traffic light found
  }
}

void TlrDetectorNode::camera_raw_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  vision_msgs::msg::BoundingBox2D current_bbox;
  {
    std::lock_guard<std::mutex> lock(bbox_mutex_);
    current_bbox = latest_traffic_light_bbox_;
  }

  // Only process if a valid traffic light bbox is available
  if (current_bbox.size_x > 0 && current_bbox.size_y > 0) {
    try {
      cv_bridge::CvImagePtr cv_ptr;
      if (msg->encoding == "rgb8") {
          cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      } else {
          cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
      }

      int x = static_cast<int>(current_bbox.center.position.x - current_bbox.size_x / 2.0);
      int y = static_cast<int>(current_bbox.center.position.y - current_bbox.size_y / 2.0);
      int width = static_cast<int>(current_bbox.size_x);
      int height = static_cast<int>(current_bbox.size_y);

      int img_width = cv_ptr->image.cols;
      int img_height = cv_ptr->image.rows;

      x = std::max(0, x);
      y = std::max(0, y);

      width = std::min(width, img_width - x);
      height = std::min(height, img_height - y);

      if (width > 0 && height > 0 && x >= 0 && y >= 0 && (x + width) <= img_width && (y + height) <= img_height) {
        cv::Rect roi(x, y, width, height);
        cv::Mat cropped_image = cv_ptr->image(roi);

        cv::Mat resized_image;
        cv::Size target_size(200, 200);
        cv::resize(cropped_image, resized_image, target_size, 0, 0, cv::INTER_LINEAR);

        sensor_msgs::msg::Image::SharedPtr cropped_msg = cv_bridge::CvImage(
          msg->header, cv_ptr->encoding, resized_image).toImageMsg();
        traffic_light_image_publisher_->publish(*cropped_msg);
        RCLCPP_INFO(this->get_logger(), "Published cropped and resized traffic light image.");
      } else {
        RCLCPP_WARN(this->get_logger(), "Calculated ROI is invalid after clamping: x=%d, y=%d, width=%d, height=%d. Image size: %dx%d", x, y, width, height, img_width, img_height);
      }

    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    } catch (const cv::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "No valid traffic light bbox received yet (size_x or size_y <= 0).");
  }
}

} // namespace tlr_detector

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tlr_detector::TlrDetectorNode>());
  rclcpp::shutdown();
  return 0;
}