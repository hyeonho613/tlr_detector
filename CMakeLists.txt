cmake_minimum_required(VERSION 3.8)
project(tlr_detector)

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

# Add external_launcher_node executable
add_executable(launcher_node
  src/launcher_node.cpp
)
ament_target_dependencies(launcher_node rclcpp)
install(TARGETS
  launcher_node
  DESTINATION lib/${PROJECT_NAME}
)

# Add image_processor_node executable
add_executable(tlr_detector_node
  src/tlr_detector_node.cpp
)
ament_target_dependencies(tlr_detector_node
  rclcpp
  vision_msgs
  ultralytics_ros
  cv_bridge
)
target_link_libraries(tlr_detector_node
  ${OpenCV_LIBS}
)
install(TARGETS
  tlr_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

# Add traffic_light_hsv_analyzer_node executable
add_executable(tlr_hsv_node
  src/tlr_hsv_node.cpp
)
ament_target_dependencies(tlr_hsv_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
)
target_link_libraries(tlr_hsv_node
  ${OpenCV_LIBS}
)
install(TARGETS
  tlr_hsv_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()