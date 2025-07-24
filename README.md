# Traffic Light Detection with V4L2 and Ultralytics ROS

This project is a ROS-based system that uses the V4L2 package to activate a camera, detects objects with the `ultralytics_ros` package, and identifies traffic lights using the `TLR_detector` package.

## Overview
The system operates in the following sequence:
1. **V4L2 Package**: Activates the camera to capture real-time video.
2. **ultralytics_ros Package**: Utilizes a YOLO-based model to detect objects in the camera feed.
3. **TLR_detector Package**: Detects traffic lights from the processed video feed.

## Usage
1. Ensure all required packages (`v4l2`, `ultralytics_ros`, `TLR_detector`) are installed in your ROS workspace.
2. Launch the V4L2 camera node to start capturing video:
   ```bash
   roslaunch v4l2_camera camera.launch
