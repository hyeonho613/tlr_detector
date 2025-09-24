  tlr_detector 실행 요약

  1. 필요한 ROS2 패키지:

   * tlr_detector (현재 작업 중인 패키지)
   * ultralytics_ros (YOLO 객체 감지)
   * v4l2_camera (카메라 이미지 발행)

  2. 외부 라이브러리:

   * numpy (버전 < 2.0, cv_bridge 호환성)
   * cv_bridge (ROS-OpenCV 이미지 변환)
   * OpenCV (이미지 처리)

  3. 디버그 및 확인:

   * 자세한 로그 확인:

   $     ros2 launch tlr_detector tlr.launch.py --ros-args --log-level DEBUG
   * 최종 신호등 인식 결과 확인:
   $     ros2 topic echo /traffic_light_status
   * 신호등만 잘라낸 이미지 시각화:
   $     ros2 run image_view image_view --ros-args -r image:=/traffic_light_image
