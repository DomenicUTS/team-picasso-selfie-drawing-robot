"""
ROS2 launch file for the selfie perception pipeline.

Launches:
  1. image_loader_node  — loads selfie from ~/perception/input/
  2. perception_node    — bg removal → HED edges → stroke extraction
  3. visualization_node — renders preview of the strokes

Topics:
  /raw_image             (sensor_msgs/Image)   loader → perception
  /drawing_strokes       (std_msgs/String)     perception → visualization, RS2, GUI
  /drawing_preview_image (sensor_msgs/Image)   visualization → GUI
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='selfie_perception',
            executable='image_loader_node',
            name='image_loader_node',
            output='screen',
            parameters=[{
                'image_dir': '~/perception/input/',
                'max_publishes': 5,
            }],
        ),
        Node(
            package='selfie_perception',
            executable='perception_node',
            name='perception_node',
            output='screen',
            parameters=[{
                'output_dir': '~/perception/output/',
                'canvas_px_w': 400,
                'canvas_px_h': 300,
                'gaussian_sigma': 3.0,
                'canny_low': 20,
                'canny_high': 60,
                'simplification_epsilon': 1.5,
                'min_contour_points': 5,
                'min_stroke_length': 15,
            }],
        ),
        Node(
            package='selfie_perception',
            executable='visualization_node',
            name='visualization_node',
            output='screen',
            parameters=[{
                'output_dir': '~/perception/output/',
                'canvas_px_w': 400,
                'canvas_px_h': 300,
                'display': False,
            }],
        ),
    ])
