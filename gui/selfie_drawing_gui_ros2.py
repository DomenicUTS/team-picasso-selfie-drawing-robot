"""
ROS 2 integrated version of the Selfie Drawing Robot GUI.

Adds a thin rclpy bridge so the PySide6 GUI can:
  * publish the captured photo on  /raw_image   (sensor_msgs/Image)
  * subscribe to /drawing_strokes  (std_msgs/String)   – stroke preview JSON
  * subscribe to /drawing_status   (std_msgs/String)   – pipeline state
  * publish     /gui/start_drawing (std_msgs/String)   – command to start
  * publish     /gui/stop_drawing  (std_msgs/String)   – command to stop/pause

The perception pipeline picks up /raw_image as usual, and the motion node
picks up /drawing_strokes.  This file imports the original GUI and extends it.

Usage:
    source ~/perception/install/setup.bash
    source ~/RS2/ros2_ws/install/setup.bash
    python3 ~/gui/selfie_drawing_gui_ros2.py
"""

import sys
import os
import json
import threading

import cv2
import numpy as np

from PySide6.QtCore import Qt, QTimer, Signal, QObject
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import QApplication, QMessageBox

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from std_msgs.msg import String

try:
    from cv_bridge import CvBridge
except ImportError:
    CvBridge = None  # fallback below

# Import the original GUI
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from selfie_drawing_gui_starter import MainWindow, AppState  # noqa: E402


# ── Minimal numpy-only CvBridge fallback ──────────────────────────
class _FallbackBridge:
    """Encode/decode sensor_msgs/Image without the C++ cv_bridge."""
    def cv2_to_imgmsg(self, cv_image, encoding="bgr8"):
        msg = Image()
        msg.height, msg.width = cv_image.shape[:2]
        msg.encoding = encoding
        msg.step = cv_image.strides[0]
        msg.data = cv_image.tobytes()
        return msg

    def imgmsg_to_cv2(self, msg, desired_encoding="bgr8"):
        channels = 3 if "bgr" in msg.encoding or "rgb" in msg.encoding else 1
        dtype = np.uint8
        img = np.frombuffer(msg.data, dtype=dtype).reshape(
            msg.height, msg.width, channels)
        if msg.encoding == "rgb8" and desired_encoding == "bgr8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img


# ── Qt signal bridge (thread-safe ROS → Qt) ──────────────────────
class _RosSignals(QObject):
    drawing_strokes_received = Signal(str)
    drawing_status_received = Signal(str)
    drawing_preview_received = Signal(object)  # numpy array


# ── ROS 2 node that lives inside the GUI process ─────────────────
class GuiRosNode(Node):
    def __init__(self, signals: _RosSignals):
        super().__init__('selfie_gui_node')
        self.signals = signals
        self.bridge = CvBridge() if CvBridge is not None else _FallbackBridge()

        # Publishers
        self.image_pub = self.create_publisher(Image, 'raw_image', 10)
        self.cmd_pub = self.create_publisher(String, 'gui/command', 10)
        self.colour_pub = self.create_publisher(String, 'gui/marker_colour', 10)

        # Subscribers
        self.create_subscription(
            String, 'drawing_strokes', self._on_strokes, 10)
        self.create_subscription(
            String, 'drawing_status', self._on_status, 10)
        self.create_subscription(
            Image, 'drawing_preview_image', self._on_preview_image, 10)

        self.get_logger().info('[GUI Node] ROS2 bridge started')

    def publish_image(self, cv_frame):
        msg = self.bridge.cv2_to_imgmsg(cv_frame, encoding='bgr8')
        self.image_pub.publish(msg)
        self.get_logger().info('[GUI Node] Published captured image to /raw_image')

    def publish_command(self, command: str):
        msg = String()
        msg.data = command
        self.cmd_pub.publish(msg)
        self.get_logger().info(f'[GUI Node] Published command: {command}')

    def publish_colour(self, colour: str):
        msg = String()
        msg.data = colour
        # Latch-style: motion node may subscribe slightly later, but as long
        # as the colour is published before START, the standard QoS keeps
        # the most recent value available.
        self.colour_pub.publish(msg)
        self.get_logger().info(f'[GUI Node] Published marker colour: {colour}')

    def _on_strokes(self, msg: String):
        self.signals.drawing_strokes_received.emit(msg.data)

    def _on_status(self, msg: String):
        self.signals.drawing_status_received.emit(msg.data)

    def _on_preview_image(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.signals.drawing_preview_received.emit(frame)


# ── Extended MainWindow ──────────────────────────────────────────
class RosMainWindow(MainWindow):
    """Subclass the starter GUI to wire buttons to ROS 2."""

    def __init__(self, ros_node: GuiRosNode, signals: _RosSignals):
        self.ros_node = ros_node
        self.ros_signals = signals
        self._strokes_json = None
        super().__init__()
        self.setWindowTitle('Selfie Drawing Robot — ROS 2 Integrated')
        # Connect ROS signals → Qt slots
        self.ros_signals.drawing_strokes_received.connect(self._on_ros_strokes)
        self.ros_signals.drawing_status_received.connect(self._on_ros_status)
        self.ros_signals.drawing_preview_received.connect(self._on_ros_preview)

    # ── Override: send captured image to perception via ROS ──
    def process_photo(self):
        if self.captured_frame is None:
            return

        self.state = AppState.PROCESSING
        self.refresh_buttons()
        self.set_status('Sending photo to perception pipeline...')
        self.log(
            f'Publishing captured image to /raw_image | '
            f'subjects={self.subject_combo.currentText()} | '
            f'style={self.colour_combo.currentText()}'
        )

        # Publish to /raw_image — perception picks it up
        self.ros_node.publish_image(self.captured_frame)

    # ── Override: send real start command ──
    def start_drawing(self):
        if self.state != AppState.PREVIEW_READY:
            return
        self.state = AppState.DRAWING
        self.progress_value = 0
        self.progress_bar.setValue(0)
        self.progress_label.setText('Progress: 0%')
        # Encode the chosen marker colour into the START command itself.
        # We tried publishing colour on a separate topic first, but with
        # the multi-threaded executor on the motion side the START
        # callback could fire before the colour callback finished
        # processing — so the pipeline started with the default colour.
        # Combining them makes the colour selection atomic.
        colour = self.colour_combo.currentText()
        # Publish on the dedicated topic too (for status/debug consumers
        # that don't parse the START command).
        self.ros_node.publish_colour(colour)
        self.set_status(f'Drawing started in {colour} — waiting for robot...')
        self.log(f'Selected colour: {colour}. Sent START:{colour} command to motion node.')
        self.ros_node.publish_command(f'START:{colour}')
        self.refresh_buttons()

    # ── Override: real pause ──
    def pause_drawing(self):
        if self.state != AppState.DRAWING:
            return
        self.state = AppState.PAUSED
        self.set_status('Drawing paused.')
        self.log('Sent PAUSE command.')
        self.ros_node.publish_command('PAUSE')
        self.refresh_buttons()

    # ── Override: real resume ──
    def resume_drawing(self):
        if self.state != AppState.PAUSED:
            return
        self.state = AppState.DRAWING
        self.set_status('Drawing resumed.')
        self.log('Sent RESUME command.')
        self.ros_node.publish_command('RESUME')
        self.refresh_buttons()

    # ── Override: real stop ──
    def stop_drawing(self):
        if self.state not in {AppState.DRAWING, AppState.PAUSED}:
            return
        self.state = AppState.ESTOPPED
        self.set_status('Drawing stopped.')
        self.log('Sent STOP command.')
        self.ros_node.publish_command('STOP')
        self.refresh_buttons()

    # ── ROS callbacks (run on Qt thread via signals) ──
    def _on_ros_strokes(self, json_str: str):
        """Perception published strokes — store for reference only.
        
        Display comes from /drawing_preview_image, which is the
        fully rendered preview with text info from perception_node.
        """
        self._strokes_json = json_str
        try:
            strokes = json.loads(json_str)
            self.log(f'Received {len(strokes)} strokes from perception '
                     f'({sum(len(s) for s in strokes)} pts)')
        except json.JSONDecodeError as e:
            self.log(f'Invalid strokes JSON: {e}')

    def _on_ros_status(self, status: str):
        """Motion node published a status update."""
        self.log(f'[Motion] {status}')
        status_upper = status.upper()
        if 'COMPLETE' in status_upper:
            self.state = AppState.FINISHED
            self.progress_bar.setValue(100)
            self.progress_label.setText('Progress: 100%')
            self.set_status('Drawing complete!')
            self.refresh_buttons()
        elif 'ERROR' in status_upper:
            self.state = AppState.ERROR
            self.set_status(f'Error: {status}')
            self.refresh_buttons()
        elif 'EXECUTING' in status_upper:
            self.state = AppState.DRAWING
            self.set_status('Robot is drawing...')
            self.refresh_buttons()

    def _on_ros_preview(self, frame):
        """Display the fully rendered preview from perception node.
        
        This is the authoritative preview with:
        - Stroke lines (black)
        - Stroke start markers (green circles)
        - Stroke/point count text
        """
        self.preview_frame = frame
        self.show_on_label(self.preview_label, frame)
        self.state = AppState.PREVIEW_READY
        self.set_status('Preview ready. Start drawing when ready.')
        self.refresh_buttons()


# ── Entry point ──────────────────────────────────────────────────
def main():
    rclpy.init()
    signals = _RosSignals()
    ros_node = GuiRosNode(signals)

    # Spin rclpy in a background thread so PySide6 event loop is unblocked
    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    app = QApplication(sys.argv)
    window = RosMainWindow(ros_node, signals)
    window.show()

    exit_code = app.exec()

    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
