# Traffic Light Detection with V4L2 and Ultralytics ROS (ROS 2)

This project is a ROS 2-based system that uses the V4L2 package to activate a camera, detects objects with the `ultralytics_ros` package, and identifies traffic lights using the `TLR_detector` package.

## Overview
The system operates in the following sequence:
1. **V4L2 Package**: Activates the camera to capture real-time video.
2. **ultralytics_ros Package**: Utilizes a YOLO-based model to detect objects in the camera feed.
3. **TLR_detector Package**: Detects traffic lights from the processed video feed.
