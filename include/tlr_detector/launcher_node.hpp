#ifndef TLR_DETECTOR__LAUNCHER_NODE_HPP_
#define TLR_DETECTOR__LAUNCHER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <string>
#include <thread>
#include <cstdlib>

namespace tlr_detector
{

class LauncherNode : public rclcpp::Node
{
public:
  LauncherNode();
  ~LauncherNode();

private:
  void run_shell_command(const std::string& command);
  void launch_v4l2_camera();
  void launch_ultralytics_ros_tracker();
};

} // namespace tlr_detector

#endif // ROS2_MULTI_LAUNCHER__LAUNCHER_NODE_HPP_
