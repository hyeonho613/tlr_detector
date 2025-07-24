#include "ros2_multi_launcher/multi_launcher_node.hpp"

namespace ros2_multi_launcher
{

MultiLauncherNode::MultiLauncherNode()
: Node("multi_launcher_node")
{
  RCLCPP_INFO(this->get_logger(), "Starting multi_launcher_node...");

  launch_v4l2_camera();
  launch_ultralytics_ros_tracker();

  // Subscribe to YOLO results
  yolo_result_subscription_ = this->create_subscription<ultralytics_ros::msg::YoloResult>(
    "/yolo_result", 10, std::bind(&MultiLauncherNode::yolo_result_callback, this, std::placeholders::_1));

  // Publish traffic light bounding box
  traffic_light_bbox_publisher_ = this->create_publisher<vision_msgs::msg::BoundingBox2D>(
    "/traffic_light_bbox", 10);

  // Subscribe to raw camera image
  camera_raw_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/image_raw", 10, std::bind(&MultiLauncherNode::camera_raw_callback, this, std::placeholders::_1));

  // Publish cropped and resized traffic light image
  traffic_light_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
    "/traffic_light_image", 10);

  RCLCPP_INFO(this->get_logger(), "Commands launched and subscribing to /yolo_result and /image_raw. Keeping node alive...");
}

MultiLauncherNode::~MultiLauncherNode()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down multi_launcher_node...");
}

void MultiLauncherNode::run_shell_command(const std::string& command)
{
  RCLCPP_INFO(this->get_logger(), "Executing command: %s", command.c_str());
  std::string full_command = "bash -c \"" + command + "\"";
  int result = std::system(full_command.c_str());
  if (result != 0) {
    RCLCPP_ERROR(this->get_logger(), "Command failed with exit code: %d", result);
  }
}

void MultiLauncherNode::launch_v4l2_camera()
{
  std::string ros2_setup_command = "source /opt/ros/humble/setup.bash && source /home/sws/yolo_ws/install/setup.bash";
  std::string v4l2_camera_command = ros2_setup_command + " && ros2 run v4l2_camera v4l2_camera_node";
  std::thread(&MultiLauncherNode::run_shell_command, this, v4l2_camera_command).detach();
}

void MultiLauncherNode::launch_ultralytics_ros_tracker()
{
  std::string ros2_setup_command = "source /opt/ros/humble/setup.bash && source /home/sws/yolo_ws/install/setup.bash";
  std::string ultralytics_ros_command = ros2_setup_command + " && ros2 launch ultralytics_ros tracker_with_cloud.launch.xml debug:=true";
  std::thread(&MultiLauncherNode::run_shell_command, this, ultralytics_ros_command).detach();
}

void MultiLauncherNode::yolo_result_callback(const ultralytics_ros::msg::YoloResult::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received YoloResult message.");
  for (const auto& detection : msg->detections.detections) {
    const auto& bbox = detection.bbox;
    RCLCPP_INFO(this->get_logger(),
                "  Bounding Box: center_x=%.2f, center_y=%.2f, size_x=%.2f, size_y=%.2f",
                bbox.center.position.x, bbox.center.position.y, bbox.size_x, bbox.size_y);

    for (const auto& hypothesis : detection.results) {
      RCLCPP_INFO(this->get_logger(),
                  "    Class ID: %s, Score: %.2f",
                  hypothesis.hypothesis.class_id.c_str(), hypothesis.hypothesis.score);

      // If traffic light detected, publish bbox and store it
      if (hypothesis.hypothesis.class_id == "traffic light") {
        RCLCPP_INFO(this->get_logger(), "    Found traffic light! Publishing bbox.");
        traffic_light_bbox_publisher_->publish(bbox);

        std::lock_guard<std::mutex> lock(bbox_mutex_);
        latest_traffic_light_bbox_ = bbox;
      }
    }
  }
}

void MultiLauncherNode::camera_raw_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received camera_raw image. Image size: %dx%d, encoding: %s", msg->width, msg->height, msg->encoding.c_str());

  vision_msgs::msg::BoundingBox2D current_bbox;
  {
    std::lock_guard<std::mutex> lock(bbox_mutex_);
    current_bbox = latest_traffic_light_bbox_;
  }

  // Process image only if a valid traffic light bbox is available
  if (current_bbox.size_x > 0 && current_bbox.size_y > 0) {
    try {
      cv_bridge::CvImagePtr cv_ptr;
      if (msg->encoding == "rgb8") {
          cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
      } else {
          cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
      }

      // Calculate ROI from bbox
      int x = static_cast<int>(current_bbox.center.position.x - current_bbox.size_x / 2.0);
      int y = static_cast<int>(current_bbox.center.position.y - current_bbox.size_y / 2.0);
      int width = static_cast<int>(current_bbox.size_x);
      int height = static_cast<int>(current_bbox.size_y);

      RCLCPP_INFO(this->get_logger(), "  Calculated ROI (before clamp): x=%d, y=%d, width=%d, height=%d", x, y, width, height);

      int img_width = cv_ptr->image.cols;
      int img_height = cv_ptr->image.rows;

      // Clamp ROI to image bounds
      x = std::max(0, x);
      y = std::max(0, y);
      width = std::min(width, img_width - x);
      height = std::min(height, img_height - y);

      RCLCPP_INFO(this->get_logger(), "  Clamped ROI (after clamp): x=%d, y=%d, width=%d, height=%d", x, y, width, height);

      // If ROI is valid, crop and resize image
      if (width > 0 && height > 0 && x >= 0 && y >= 0 && (x + width) <= img_width && (y + height) <= img_height) {
        cv::Rect roi(x, y, width, height);
        cv::Mat cropped_image = cv_ptr->image(roi);

        cv::Mat resized_image;
        cv::Size target_size(200, 200);
        cv::resize(cropped_image, resized_image, target_size, 0, 0, cv::INTER_LINEAR);

        sensor_msgs::msg::Image::SharedPtr cropped_msg = cv_bridge::CvImage(
          msg->header, cv_ptr->encoding, resized_image).toImageMsg();
        traffic_light_image_publisher_->publish(*cropped_msg);
        RCLCPP_INFO(this->get_logger(), "Published cropped and resized traffic light image of size %dx%d.", resized_image.cols, resized_image.rows);
      } else {
        RCLCPP_WARN(this->get_logger(), "Calculated ROI is invalid after clamping: x=%d, y=%d, width=%d, height=%d. Image size: %dx%d", x, y, width, height, img_width, img_height);
      }

    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    } catch (const cv::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "No valid traffic light bbox received yet (size_x or size_y <= 0). Current bbox: center_x=%.2f, center_y=%.2f, size_x=%.2f, size_y=%.2f",
                current_bbox.center.position.x, current_bbox.center.position.y, current_bbox.size_x, current_bbox.size_y);
  }
}

} // namespace ros2_multi_launcher

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ros2_multi_launcher::MultiLauncherNode>());
  rclcpp::shutdown();
  return 0;
}
