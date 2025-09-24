from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_share_directory = get_package_share_directory('tlr_detector')
    tlr_hsv_params_file = os.path.join(package_share_directory, 'config', 'tlr_hsv_params.yaml')

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
            remappings=[
                ('image_raw', '/sensing/camera/front/image_raw')
            ]
        ),
        Node(
            package='tlr_detector',
            executable='tlr_hsv_node',
            name='tlr_hsv_node',
            output='screen',
            emulate_tty=True, # Required for output to be visible
            parameters=[tlr_hsv_params_file]
        ),
        Node(
            package='tlr_detector',
            executable='traffic_light_mode_changer',
            name='traffic_light_mode_changer',
            output='screen',
            emulate_tty=True, # Required for output to be visible
        ),
    ])