"""
ROS2 Image Loader Node — watches a directory and publishes selfie images.

Publishes to /raw_image (sensor_msgs/Image) for the perception pipeline.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import glob


class ImageLoaderNode(Node):

    def __init__(self):
        super().__init__('image_loader_node')

        default_input = os.path.join(
            os.path.expanduser('~'), 'perception', 'input')
        self.declare_parameter('image_dir', default_input)
        self.declare_parameter('max_publishes', 5)

        self.image_dir = os.path.expanduser(
            self.get_parameter('image_dir').value)
        self.max_publishes = self.get_parameter('max_publishes').value

        self.publisher_ = self.create_publisher(Image, 'raw_image', 10)
        self.bridge = CvBridge()
        self.publish_count = 0

        os.makedirs(self.image_dir, exist_ok=True)
        self.timer = self.create_timer(1.0, self.check_for_image)
        self.get_logger().info(
            f"Image Loader Node started — watching: {self.image_dir}")

    def check_for_image(self):
        if self.publish_count >= self.max_publishes:
            return

        extensions = ['*.jpg', '*.jpeg', '*.png', '*.bmp']
        image_files = []
        for ext in extensions:
            image_files.extend(glob.glob(os.path.join(self.image_dir, ext)))

        if not image_files:
            self.get_logger().info(
                f"No images found in {self.image_dir}, waiting...",
                throttle_duration_sec=5.0)
            return

        image_path = max(image_files, key=os.path.getmtime)
        frame = cv2.imread(image_path)
        if frame is None:
            self.get_logger().error(f"Failed to read image: {image_path}")
            return

        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        self.publisher_.publish(ros_image)
        self.publish_count += 1

        if self.publish_count == 1:
            self.get_logger().info(f"Publishing image: {image_path}")
        if self.publish_count >= self.max_publishes:
            self.get_logger().info(
                "Done publishing. Pipeline should have received the image.")


def main(args=None):
    rclpy.init(args=args)
    node = ImageLoaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
