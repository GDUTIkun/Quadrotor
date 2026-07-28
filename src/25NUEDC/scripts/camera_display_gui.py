#!/usr/bin/env python3
"""Full-screen HDMI viewer for a ROS 2 CompressedImage topic."""

from __future__ import annotations

import signal
import sys
import time

import rclpy
from rclpy.signals import SignalHandlerOptions
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor, QImage, QKeyEvent, QPainter, QPixmap
from PyQt5.QtWidgets import QApplication, QLabel, QMainWindow, QVBoxLayout, QWidget


class CameraDisplayNode(Node):
    def __init__(self, window: "CameraDisplayWindow") -> None:
        super().__init__("camera_display_gui")
        self.declare_parameter("image_topic", "/camera/image_raw/compressed")
        self.image_topic = str(self.get_parameter("image_topic").value)
        self.window = window
        self.subscription = self.create_subscription(
            CompressedImage,
            self.image_topic,
            self.on_image,
            qos_profile_sensor_data,
        )
        self.get_logger().info(f"自动订阅图像话题: {self.image_topic}")

    def on_image(self, message: CompressedImage) -> None:
        image = QImage.fromData(bytes(message.data))
        if image.isNull():
            self.get_logger().warning("收到无法解码的压缩图像")
            return
        self.window.set_image(image)


class CameraLabel(QLabel):
    def __init__(self) -> None:
        super().__init__("等待机载图像…")
        self._image = QImage()
        self.setAlignment(Qt.AlignCenter)
        self.setMinimumSize(320, 240)
        self.setStyleSheet("background: black; color: white; font-size: 28px;")

    def set_camera_image(self, image: QImage) -> None:
        self._image = image
        self.update_pixmap()

    def resizeEvent(self, event) -> None:  # noqa: N802
        super().resizeEvent(event)
        self.update_pixmap()

    def update_pixmap(self) -> None:
        if self._image.isNull():
            return
        target_size = self.size()
        scaled = self._image.scaled(
            target_size,
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation,
        )
        canvas = QPixmap(target_size)
        canvas.fill(QColor("black"))
        painter = QPainter(canvas)
        x = (target_size.width() - scaled.width()) // 2
        y = (target_size.height() - scaled.height()) // 2
        painter.drawImage(x, y, scaled)
        painter.end()
        self.setPixmap(canvas)


class CameraDisplayWindow(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.setWindowTitle("无人机实时图像")
        self.camera_label = CameraLabel()
        self.status_label = QLabel("正在等待 /camera/image_raw/compressed")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setFixedHeight(42)
        self.status_label.setStyleSheet(
            "background: #17212b; color: #e8f1f8; font-size: 18px; padding: 6px;"
        )

        central = QWidget()
        layout = QVBoxLayout(central)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)
        layout.addWidget(self.camera_label, 1)
        layout.addWidget(self.status_label)
        self.setCentralWidget(central)

        self.frame_count = 0
        self.fps_frame_count = 0
        self.fps_started_at = time.monotonic()
        self.current_fps = 0.0

    def set_image(self, image: QImage) -> None:
        self.camera_label.set_camera_image(image)
        self.frame_count += 1
        self.fps_frame_count += 1
        now = time.monotonic()
        elapsed = now - self.fps_started_at
        if elapsed >= 2.0:
            self.current_fps = self.fps_frame_count / elapsed
            self.fps_frame_count = 0
            self.fps_started_at = now
        self.status_label.setText(
            f"在线  |  {image.width()}×{image.height()}  |  "
            f"{self.current_fps:.1f} FPS  |  已接收 {self.frame_count} 帧"
        )

    def keyPressEvent(self, event: QKeyEvent) -> None:  # noqa: N802
        if event.key() in (Qt.Key_Escape, Qt.Key_Q):
            self.close()
            return
        if event.key() == Qt.Key_F11:
            self.showNormal() if self.isFullScreen() else self.showFullScreen()
            return
        super().keyPressEvent(event)


def main() -> int:
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    app = QApplication(sys.argv[:1])
    signal.signal(signal.SIGINT, lambda _signum, _frame: app.quit())
    signal.signal(signal.SIGTERM, lambda _signum, _frame: app.quit())
    window = CameraDisplayWindow()
    node = CameraDisplayNode(window)

    spin_timer = QTimer()
    spin_timer.setInterval(5)
    def spin_ros_once() -> None:
        if not rclpy.ok():
            spin_timer.stop()
            app.quit()
            return
        try:
            rclpy.spin_once(node, timeout_sec=0.0)
        except Exception:
            if not rclpy.ok():
                spin_timer.stop()
                app.quit()
                return
            raise

    spin_timer.timeout.connect(spin_ros_once)
    spin_timer.start()

    app.aboutToQuit.connect(spin_timer.stop)
    window.showFullScreen()
    result = app.exec()

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return result


if __name__ == "__main__":
    raise SystemExit(main())
