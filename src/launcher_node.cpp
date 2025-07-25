#include "tlr_detector/launcher_node.hpp"

namespace tlr_detector
{

LauncherNode::LauncherNode()
: Node("launcher_node")
{
  RCLCPP_INFO(this->get_logger(), "Starting external_launcher_node...");

  launch_v4l2_camera();
  launch_ultralytics_ros_tracker();

  RCLCPP_INFO(this->get_logger(), "External commands launched.");
}

LauncherNode::~LauncherNode()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down external_launcher_node...");
}

void LauncherNode::run_shell_command(const std::string& command)
{
  RCLCPP_INFO(this->get_logger(), "Executing command: %s", command.c_str());
  std::string full_command = "bash -c \"" + command + "\"";
  int result = std::system(full_command.c_str());
  if (result != 0) {
    RCLCPP_ERROR(this->get_logger(), "Command failed with exit code: %d", result);
  }
}

void LauncherNode::launch_v4l2_camera()
{
  std::string ros2_setup_command = "source /opt/ros/humble/setup.bash && source /home/sws/yolo_ws/install/setup.bash";
  std::string v4l2_camera_command = ros2_setup_command + " && ros2 run v4l2_camera v4l2_camera_node";
  std::thread(&LauncherNode::run_shell_command, this, v4l2_camera_command).detach();
}

void LauncherNode::launch_ultralytics_ros_tracker()
{
  std::string ros2_setup_command = "source /opt/ros/humble/setup.bash && source /home/sws/yolo_ws/install/setup.bash";
  std::string ultralytics_ros_command = ros2_setup_command + " && ros2 launch ultralytics_ros tracker.launch.xml debug:=true"; // tracker_with_cloud.launch.xml
  std::thread(&LauncherNode::run_shell_command, this, ultralytics_ros_command).detach();
}

} // namespace tlr_detector

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tlr_detector::LauncherNode>());
  rclcpp::shutdown();
  return 0;
}