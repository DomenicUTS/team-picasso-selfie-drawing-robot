import sys
from enum import Enum, auto

import cv2
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QProgressBar,
    QSizePolicy,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)


class AppState(Enum):
    DISCONNECTED = auto()
    LIVE_PREVIEW = auto()
    PHOTO_CAPTURED = auto()
    PROCESSING = auto()
    PREVIEW_READY = auto()
    DRAWING = auto()
    PAUSED = auto()
    FINISHED = auto()
    ERROR = auto()
    ESTOPPED = auto()


class CameraHandler:
    def __init__(self, camera_index: int = 0):
        self.camera_index = camera_index
        self.cap = None

    def start(self) -> bool:
        self.cap = cv2.VideoCapture(self.camera_index)
        return self.cap.isOpened()

    def get_frame(self):
        if self.cap is None:
            return None
        ok, frame = self.cap.read()
        if not ok:
            return None
        return frame

    def release(self):
        if self.cap is not None:
            self.cap.release()
            self.cap = None


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Selfie Drawing Robot GUI Starter")
        self.resize(1280, 780)

        self.state = AppState.DISCONNECTED
        self.camera = CameraHandler()
        self.current_frame = None
        self.captured_frame = None
        self.preview_frame = None
        self.progress_value = 0

        self.camera_timer = QTimer(self)
        self.camera_timer.timeout.connect(self.update_camera_feed)

        self.draw_timer = QTimer(self)
        self.draw_timer.timeout.connect(self.update_fake_drawing_progress)

        self.build_ui()
        self.connect_signals()
        self.start_camera()

    def build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)

        root_layout = QVBoxLayout(central)

        title = QLabel("Picasso - Selfie Drawing UR3")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 22px; font-weight: 700;")

        self.status_label = QLabel("Status: Initialising...")
        self.status_label.setStyleSheet("font-size: 14px;")

        top_layout = QHBoxLayout()
        top_layout.addWidget(title, 1)
        top_layout.addWidget(self.status_label, 1)

        images_layout = QHBoxLayout()

        self.camera_group = QGroupBox("Camera / Captured Photo")
        self.preview_group = QGroupBox("Processed Preview")

        self.camera_label = QLabel("No camera feed")
        self.camera_label.setAlignment(Qt.AlignCenter)
        self.camera_label.setMinimumSize(560, 420)
        self.camera_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.camera_label.setStyleSheet("border: 1px solid #888; background: #111; color: #ddd;")

        self.preview_label = QLabel("No preview yet")
        self.preview_label.setAlignment(Qt.AlignCenter)
        self.preview_label.setMinimumSize(560, 420)
        self.preview_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.preview_label.setStyleSheet("border: 1px solid #888; background: #111; color: #ddd;")

        camera_layout = QVBoxLayout()
        camera_layout.addWidget(self.camera_label)
        self.camera_group.setLayout(camera_layout)

        preview_layout = QVBoxLayout()
        preview_layout.addWidget(self.preview_label)
        self.preview_group.setLayout(preview_layout)

        images_layout.addWidget(self.camera_group)
        images_layout.addWidget(self.preview_group)

        controls_group = QGroupBox("Controls")
        controls_layout = QGridLayout()

        self.subject_combo = QComboBox()
        self.subject_combo.addItems(["1", "2"])

        self.colour_combo = QComboBox()
        self.colour_combo.addItems(["red", "blue", "green", "black"])

        self.capture_button = QPushButton("Capture")
        self.retake_button = QPushButton("Retake")
        self.process_button = QPushButton("Process")
        self.start_button = QPushButton("Start Drawing")
        self.pause_button = QPushButton("Pause")
        self.resume_button = QPushButton("Resume")
        self.stop_button = QPushButton("Stop")
        self.reset_button = QPushButton("Reset")

        controls_layout.addWidget(QLabel("Subjects"), 0, 0)
        controls_layout.addWidget(self.subject_combo, 0, 1)
        controls_layout.addWidget(QLabel("Style"), 0, 2)
        controls_layout.addWidget(self.colour_combo, 0, 3)

        controls_layout.addWidget(self.capture_button, 1, 0)
        controls_layout.addWidget(self.retake_button, 1, 1)
        controls_layout.addWidget(self.process_button, 1, 2)
        controls_layout.addWidget(self.start_button, 1, 3)
        controls_layout.addWidget(self.pause_button, 1, 4)
        controls_layout.addWidget(self.resume_button, 1, 5)
        controls_layout.addWidget(self.stop_button, 1, 6)
        controls_layout.addWidget(self.reset_button, 1, 7)

        controls_group.setLayout(controls_layout)

        bottom_layout = QHBoxLayout()

        progress_group = QGroupBox("Execution")
        progress_layout = QVBoxLayout()
        self.progress_bar = QProgressBar()
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        self.progress_label = QLabel("Progress: 0%")
        progress_layout.addWidget(self.progress_bar)
        progress_layout.addWidget(self.progress_label)
        progress_group.setLayout(progress_layout)

        log_group = QGroupBox("System Log")
        log_layout = QVBoxLayout()
        self.log_box = QTextEdit()
        self.log_box.setReadOnly(True)
        log_layout.addWidget(self.log_box)
        log_group.setLayout(log_layout)

        bottom_layout.addWidget(progress_group, 1)
        bottom_layout.addWidget(log_group, 2)

        root_layout.addLayout(top_layout)
        root_layout.addLayout(images_layout)
        root_layout.addWidget(controls_group)
        root_layout.addLayout(bottom_layout)

    def connect_signals(self):
        self.capture_button.clicked.connect(self.capture_photo)
        self.retake_button.clicked.connect(self.retake_photo)
        self.process_button.clicked.connect(self.process_photo)
        self.start_button.clicked.connect(self.start_drawing)
        self.pause_button.clicked.connect(self.pause_drawing)
        self.resume_button.clicked.connect(self.resume_drawing)
        self.stop_button.clicked.connect(self.stop_drawing)
        self.reset_button.clicked.connect(self.reset_system)

    def start_camera(self):
        if self.camera.start():
            self.state = AppState.LIVE_PREVIEW
            self.camera_timer.start(30)
            self.set_status("Camera connected. Live preview running.")
            self.log("Camera connected successfully.")
        else:
            self.state = AppState.ERROR
            self.set_status("Could not connect to webcam.")
            self.log("Camera connection failed.")
            QMessageBox.warning(self, "Camera Error", "Could not open the webcam.")
        self.refresh_buttons()

    def update_camera_feed(self):
        frame = self.camera.get_frame()
        if frame is None:
            return
        self.current_frame = frame
        if self.state == AppState.LIVE_PREVIEW:
            self.show_on_label(self.camera_label, frame)

    def capture_photo(self):
        if self.current_frame is None:
            return
        self.captured_frame = self.current_frame.copy()
        self.state = AppState.PHOTO_CAPTURED
        self.show_on_label(self.camera_label, self.captured_frame)
        self.set_status("Photo captured. Retake or process.")
        self.log("Photo captured.")
        self.refresh_buttons()

    def retake_photo(self):
        self.captured_frame = None
        self.preview_frame = None
        self.preview_label.setText("No preview yet")
        self.state = AppState.LIVE_PREVIEW
        self.set_status("Retake enabled. Live preview resumed.")
        self.log("Retake selected.")
        self.refresh_buttons()

    def process_photo(self):
        if self.captured_frame is None:
            return

        self.state = AppState.PROCESSING
        self.refresh_buttons()
        self.set_status("Processing image...")
        self.log(
            f"Processing started | subjects={self.subject_combo.currentText()} | "
            f"style={self.colour_combo.currentText()}"
        )

        frame = self.captured_frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        threshold_1, threshold_2 = 60, 120

        edges = cv2.Canny(gray, threshold_1, threshold_2)
        edges = cv2.GaussianBlur(edges, (3, 3), 0)

        preview = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)

        self.preview_frame = preview
        self.show_on_label(self.preview_label, self.preview_frame)
        self.state = AppState.PREVIEW_READY
        self.set_status("Preview ready. Start drawing when ready.")
        self.log("Preview generated successfully.")
        self.refresh_buttons()

        # TODO: Replace local preview generation with a ROS2 request to the perception node.
        # Expected flow:
        # 1. Send captured image and UI settings
        # 2. Receive processed preview/vector result
        # 3. Display it here and enable Start Drawing

    def start_drawing(self):
        if self.state != AppState.PREVIEW_READY:
            return

        self.state = AppState.DRAWING
        self.progress_value = 0
        self.progress_bar.setValue(0)
        self.progress_label.setText("Progress: 0%")
        self.draw_timer.start(120)
        self.set_status("Drawing started.")
        self.log("Fake drawing run started.")
        self.refresh_buttons()

        # TODO: Replace this fake run with a ROS2 command to the motion planning node.
        # Expected flow:
        # 1. Send execute/start command
        # 2. Subscribe to progress/status
        # 3. Update progress bar from real feedback

    def update_fake_drawing_progress(self):
        if self.state != AppState.DRAWING:
            return

        self.progress_value += 2
        if self.progress_value > 100:
            self.progress_value = 100

        self.progress_bar.setValue(self.progress_value)
        self.progress_label.setText(f"Progress: {self.progress_value}%")

        if self.progress_value >= 100:
            self.draw_timer.stop()
            self.state = AppState.FINISHED
            self.set_status("Drawing complete.")
            self.log("Fake drawing run finished.")
            self.refresh_buttons()

    def pause_drawing(self):
        if self.state != AppState.DRAWING:
            return
        self.draw_timer.stop()
        self.state = AppState.PAUSED
        self.set_status("Drawing paused.")
        self.log("Pause requested.")
        self.refresh_buttons()

        # TODO: Send pause command to motion node

    def resume_drawing(self):
        if self.state != AppState.PAUSED:
            return
        self.draw_timer.start(120)
        self.state = AppState.DRAWING
        self.set_status("Drawing resumed.")
        self.log("Resume requested.")
        self.refresh_buttons()

        # TODO: Send resume command to motion node

    def stop_drawing(self):
        if self.state not in {AppState.DRAWING, AppState.PAUSED}:
            return
        self.draw_timer.stop()
        self.state = AppState.ESTOPPED
        self.set_status("Drawing stopped.")
        self.log("Stop requested.")
        self.refresh_buttons()

        # TODO: Send stop/cancel command to motion node

    def reset_system(self):
        self.draw_timer.stop()
        self.progress_value = 0
        self.progress_bar.setValue(0)
        self.progress_label.setText("Progress: 0%")
        self.captured_frame = None
        self.preview_frame = None
        self.preview_label.setText("No preview yet")
        self.state = AppState.LIVE_PREVIEW if self.current_frame is not None else AppState.ERROR
        self.set_status("System reset. Ready for next capture.")
        self.log("System reset.")
        self.refresh_buttons()

    def refresh_buttons(self):
        self.capture_button.setEnabled(self.state in {AppState.LIVE_PREVIEW, AppState.FINISHED})
        self.retake_button.setEnabled(self.state in {AppState.PHOTO_CAPTURED, AppState.PREVIEW_READY})
        self.process_button.setEnabled(self.state == AppState.PHOTO_CAPTURED)
        self.start_button.setEnabled(self.state == AppState.PREVIEW_READY)
        self.pause_button.setEnabled(self.state == AppState.DRAWING)
        self.resume_button.setEnabled(self.state == AppState.PAUSED)
        self.stop_button.setEnabled(self.state in {AppState.DRAWING, AppState.PAUSED})
        self.reset_button.setEnabled(self.state != AppState.DISCONNECTED)

    def show_on_label(self, label: QLabel, frame):
        pixmap = self.cv_to_qpixmap(frame)
        scaled = pixmap.scaled(label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation)
        label.setPixmap(scaled)

    @staticmethod
    def cv_to_qpixmap(frame) -> QPixmap:
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        height, width, channels = rgb.shape
        bytes_per_line = channels * width
        image = QImage(rgb.data, width, height, bytes_per_line, QImage.Format_RGB888)
        return QPixmap.fromImage(image)

    def set_status(self, text: str):
        self.status_label.setText(f"Status: {text}")

    def log(self, text: str):
        self.log_box.append(text)

    def closeEvent(self, event):
        self.camera_timer.stop()
        self.draw_timer.stop()
        self.camera.release()
        super().closeEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec())
