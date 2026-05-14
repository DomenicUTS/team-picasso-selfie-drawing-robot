"""
ROS2 Visualization Node — renders a preview of the drawing strokes.

Subscribes to /drawing_strokes (std_msgs/String JSON), renders a preview
image, saves it to disk, and publishes to /drawing_preview_image.
"""

import json
import os

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class VisualizationNode(Node):

    def __init__(self):
        super().__init__('visualization_node')

        default_output = os.path.join(
            os.path.expanduser('~'), 'perception', 'output')
        self.declare_parameter('output_dir', default_output)
        self.declare_parameter('canvas_px_w', 400)
        self.declare_parameter('canvas_px_h', 300)
        self.declare_parameter('display', True)

        self.output_dir = os.path.expanduser(
            self.get_parameter('output_dir').value)
        self.canvas_w = self.get_parameter('canvas_px_w').value
        self.canvas_h = self.get_parameter('canvas_px_h').value
        self.display = self.get_parameter('display').value

        os.makedirs(self.output_dir, exist_ok=True)
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            String, 'drawing_strokes', self.vis_callback, 10)
        self.preview_pub = self.create_publisher(
            Image, 'drawing_preview_image', 10)
        self.raw_sub = self.create_subscription(
            Image, 'raw_image', self._on_new_raw_image, 10)

        self.get_logger().info(
            f"Visualization Node started — output: {self.output_dir}")
        self.done = False

    def _on_new_raw_image(self, msg):
        if self.done:
            self.get_logger().info(
                "New raw_image received — resetting visualization")
            self.done = False

    def vis_callback(self, msg):
        if self.done:
            return
        self.done = True

        strokes = json.loads(msg.data)
        if not strokes:
            self.get_logger().warn("No strokes received — nothing to draw")
            return

        canvas = np.ones((self.canvas_h, self.canvas_w, 3),
                         dtype=np.uint8) * 255
        total_pts = 0
        for stroke in strokes:
            pts = np.array(stroke, dtype=np.int32)
            if len(pts) < 2:
                continue
            for j in range(len(pts) - 1):
                p1 = tuple(pts[j])
                p2 = tuple(pts[j + 1])
                cv2.line(canvas, p1, p2, (0, 0, 0), 1, cv2.LINE_AA)
            cv2.circle(canvas, tuple(pts[0]), 2, (0, 160, 0), -1)
            total_pts += len(stroke)

        info = f"{len(strokes)} strokes, {total_pts} pts"
        cv2.putText(canvas, info, (10, self.canvas_h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (128, 128, 128), 1)

        preview_path = os.path.join(self.output_dir, 'drawing_preview.png')
        cv2.imwrite(preview_path, canvas)
        self.get_logger().info(f"Saved drawing preview → {preview_path}")

        preview_msg = self.bridge.cv2_to_imgmsg(canvas, 'bgr8')
        self.preview_pub.publish(preview_msg)
        self.get_logger().info(
            'Published preview image to /drawing_preview_image')

        if self.display:
            cv2.imshow("Drawing Preview (what robot will draw)", canvas)
            cv2.waitKey(0)
            cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
