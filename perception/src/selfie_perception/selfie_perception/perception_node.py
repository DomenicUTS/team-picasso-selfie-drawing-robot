"""
ROS2 Perception Node — subscribes to /raw_image, runs the full pipeline,
publishes stroke JSON to /drawing_strokes.

Pipeline stages:
  1. Background removal  (rembg / u2net_human_seg)
  2. Canny edge detection (Gaussian σ=3 pre-blur)
  3. Stroke extraction   (contour → JSON)
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

from selfie_perception.background_removal import remove_background
from selfie_perception.edge_detection import detect_edges
from selfie_perception.stroke_extraction import (
    extract_strokes, save_strokes, strokes_to_json,
    CANVAS_PX_W, CANVAS_PX_H)


class PerceptionNode(Node):

    def __init__(self):
        super().__init__('perception_node')

        default_output = os.path.join(
            os.path.expanduser('~'), 'perception', 'output')

        self.declare_parameter('output_dir', default_output)
        self.declare_parameter('canvas_px_w', CANVAS_PX_W)
        self.declare_parameter('canvas_px_h', CANVAS_PX_H)
        self.declare_parameter('gaussian_sigma', 3.0)
        self.declare_parameter('canny_low', 20)
        self.declare_parameter('canny_high', 60)
        self.declare_parameter('simplification_epsilon', 3.0)
        self.declare_parameter('min_contour_points', 5)
        self.declare_parameter('min_stroke_length', 11)

        self.output_dir = os.path.expanduser(
            self.get_parameter('output_dir').value)
        self.canvas_w = self.get_parameter('canvas_px_w').value
        self.canvas_h = self.get_parameter('canvas_px_h').value
        self.gaussian_sigma = self.get_parameter('gaussian_sigma').value
        self.canny_low = self.get_parameter('canny_low').value
        self.canny_high = self.get_parameter('canny_high').value
        self.epsilon = self.get_parameter('simplification_epsilon').value
        self.min_points = self.get_parameter('min_contour_points').value
        self.min_stroke_length = self.get_parameter('min_stroke_length').value

        os.makedirs(self.output_dir, exist_ok=True)

        self.bridge = CvBridge()

        # Subscribe to raw images
        self.sub_image = self.create_subscription(
            Image, 'raw_image', self._on_image, 10)

        # Publish stroke JSON for RS2 motion planning and GUI
        self.pub_strokes = self.create_publisher(
            String, 'drawing_strokes', 10)

        # Publish preview image for GUI
        self.pub_preview = self.create_publisher(
            Image, 'drawing_preview_image', 10)

        self._processing = False
        self.get_logger().info("Perception Node started")

    def _on_image(self, msg):
        if self._processing:
            return
        self._processing = True

        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.get_logger().info(
                f"Received image ({image.shape[1]}x{image.shape[0]})")

            # Stage 1: Background removal
            self.get_logger().info("Stage 1: Removing background...")
            nobg = remove_background(image)

            # Stage 2: Canny edge detection (σ=3)
            self.get_logger().info(
                f"Stage 2: Canny edge detection (sigma={self.gaussian_sigma})...")
            edges = detect_edges(
                nobg, sigma=self.gaussian_sigma,
                low_threshold=self.canny_low,
                high_threshold=self.canny_high)

            # Stage 3: Stroke extraction
            self.get_logger().info("Stage 3: Extracting strokes...")
            strokes = extract_strokes(
                edges, canvas_w=self.canvas_w, canvas_h=self.canvas_h,
                epsilon=self.epsilon, min_points=self.min_points,
                min_stroke_length=self.min_stroke_length)

            total_pts = sum(len(s) for s in strokes)
            self.get_logger().info(
                f"Extracted {len(strokes)} strokes, {total_pts} points")

            # Publish strokes as JSON on /drawing_strokes
            stroke_json = strokes_to_json(strokes)
            stroke_msg = String()
            stroke_msg.data = stroke_json
            self.pub_strokes.publish(stroke_msg)
            self.get_logger().info("Published strokes to /drawing_strokes")

            # Save to disk for file-based RS2 consumption
            out_path = os.path.join(self.output_dir, 'perception_strokes.json')
            save_strokes(strokes, out_path)
            self.get_logger().info(f"Saved strokes to {out_path}")

            # Generate and publish preview
            preview = self._draw_preview(strokes)
            preview_path = os.path.join(self.output_dir, 'drawing_preview.png')
            cv2.imwrite(preview_path, preview)
            preview_msg = self.bridge.cv2_to_imgmsg(preview, 'bgr8')
            self.pub_preview.publish(preview_msg)
            self.get_logger().info("Published preview to /drawing_preview_image")
        finally:
            self._processing = False

    def _draw_preview(self, strokes):
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
        return canvas


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
