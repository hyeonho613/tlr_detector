cmake_minimum_required(VERSION 3.8)
project(ros2_multi_launcher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(ultralytics_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Add the include directory
include_directories(
  include
)

# Add the executable
add_executable(multi_launcher_node
  src/multi_launcher_node.cpp
)

# Link against necessary libraries
ament_target_dependencies(multi_launcher_node
  rclcpp
  vision_msgs
  ultralytics_ros
  cv_bridge
)

target_link_libraries(multi_launcher_node
  ${OpenCV_LIBS}
)

# Install the executable
install(TARGETS
  multi_launcher_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
