#ifndef TLR_DETECTOR__TLR_DETECTOR_NODE_HPP_
#define TLR_DETECTOR__TLR_DETECTOR_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <ultralytics_ros/msg/yolo_result.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/bounding_box2_d.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <string>
#include <mutex>

namespace tlr_detector
{

class TlrDetectorNode : public rclcpp::Node
{
public:
  TlrDetectorNode();
  ~TlrDetectorNode();

private:
  // Callback for /yolo_result topic
  void yolo_result_callback(const ultralytics_ros::msg::YoloResult::SharedPtr msg);
  // Callback for /image_raw topic
  void camera_raw_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  // Sub: /yolo_result
  rclcpp::Subscription<ultralytics_ros::msg::YoloResult>::SharedPtr yolo_result_subscription_;
  // Sub: /image_raw
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_raw_subscription_;

  // Pub: /traffic_light_bbox
  rclcpp::Publisher<vision_msgs::msg::BoundingBox2D>::SharedPtr traffic_light_bbox_publisher_;
  // Pub: /traffic_light_image
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr traffic_light_image_publisher_;

  // Stores the latest traffic light bounding box
  vision_msgs::msg::BoundingBox2D latest_traffic_light_bbox_;
  // Mutex for protecting latest_traffic_light_bbox_
  std::mutex bbox_mutex_;
};

} // namespace tlr_detector

#endif // ROS2_MULTI_LAUNCHER__TLR_DETECTOR_NODE_HPP_
