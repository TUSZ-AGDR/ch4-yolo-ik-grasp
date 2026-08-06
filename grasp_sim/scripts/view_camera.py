"""Minimal camera viewer — no Qt needed, uses OpenCV."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np


class CameraViewer(Node):
    def __init__(self):
        super().__init__("camera_viewer")
        self.sub = self.create_subscription(Image, "/camera/color/image_raw", self._cb, 10)

    def _cb(self, msg):
        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        bgr = cv2.cvtColor(raw, cv2.COLOR_RGB2BGR)
        cv2.imshow("Camera (YOLO input)", bgr)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = CameraViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
