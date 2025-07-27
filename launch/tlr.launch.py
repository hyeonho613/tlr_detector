from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tlr_detector',
            executable='launcher_node',
            name='launcher_node',
            output='screen',
            emulate_tty=True, # Required for output to be visible
        ),
        Node(
            package='tlr_detector',
            executable='tlr_detector_node',
            name='tlr_detector_node',
            output='screen',
            emulate_tty=True, # Required for output to be visible
        ),
        Node(
            package='tlr_detector',
            executable='tlr_hsv_node',
            name='tlr_hsv_node',
            output='screen',
            emulate_tty=True, # Required for output to be visible
        ),
        Node(
            package='tlr_detector',
            executable='traffic_light_mode_changer',
            name='traffic_light_mode_changer',
            output='screen',
            emulate_tty=True, # Required for output to be visible
        ),
    ])