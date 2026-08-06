#!/usr/bin/env python3
"""
yolo_vision.detector_node — YOLO 检测 + 3D 定位 ROS2 节点
=========================================================
  Sub: /camera/color/image_raw   (sensor_msgs/Image)
  Sub: /camera/depth/image_raw   (sensor_msgs/Image)
  Sub: /camera/color/camera_info (sensor_msgs/CameraInfo)
  Pub: /detections     (vision_msgs/Detection2DArray, 含 3D 相机光轴坐标)
  Pub: /detections_image (sensor_msgs/Image, 标注可视化)

模型从包内 models/best.pt 加载，支持 ros2 run 参数覆盖。
手动图像转换（不依赖 cv_bridge）。
"""

import math
import os
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from ament_index_python.packages import get_package_share_directory


# --- Encoding maps (avoid cv_bridge numpy conflict) ---
_ENCODING_TO_DTYPE = {
    "rgb8":     (np.uint8, 3),
    "bgr8":     (np.uint8, 3),
    "bgra8":    (np.uint8, 4),
    "rgba8":    (np.uint8, 4),
    "mono8":    (np.uint8, 1),
    "16UC1":    (np.uint16, 1),
    "32FC1":    (np.float32, 1),
}


def _msg_to_numpy(msg: Image) -> np.ndarray:
    """Convert ROS Image to numpy array (replaces cv_bridge.imgmsg_to_cv2)."""
    base = msg.encoding.split(";")[0]
    dtype, ch = _ENCODING_TO_DTYPE.get(base, (np.uint8, 3))
    arr = np.frombuffer(msg.data, dtype=dtype)
    if ch > 1:
        return arr.reshape(msg.height, msg.width, ch)
    return arr.reshape(msg.height, msg.width)


def _numpy_to_msg(arr: np.ndarray, encoding: str, header=None) -> Image:
    """Convert numpy array to ROS Image (replaces cv_bridge.cv2_to_imgmsg)."""
    msg = Image()
    msg.height = arr.shape[0]
    msg.width = arr.shape[1]
    msg.encoding = encoding
    msg.step = msg.width * arr.shape[2] if arr.ndim == 3 else msg.width * arr.dtype.itemsize
    msg.data = arr.tobytes()
    if header is not None:
        msg.header = header
    return msg


class DetectorNode(Node):

    def __init__(self):
        super().__init__('yolo_detector')

        # ---- 参数 ----
        self.declare_parameter('model', 'best.pt')
        self.declare_parameter('confidence', 0.5)
        self.declare_parameter('device', 'cuda:0')

        model_name = self.get_parameter('model').value
        self.conf = self.get_parameter('confidence').value
        device = self.get_parameter('device').value

        # 模型路径：绝对路径 > 包内 models/ > 自动下载
        if os.path.isabs(model_name) and os.path.exists(model_name):
            model_path = model_name
        else:
            share_dir = get_package_share_directory('yolo_vision')
            model_path = os.path.join(share_dir, 'models', model_name)
            if not os.path.exists(model_path):
                self.get_logger().warn(f'Model not found: {model_path}, using yolov8n.pt')
                model_path = 'yolov8n.pt'

        self.get_logger().info(f'Loading model: {model_path} on {device}')
        from ultralytics import YOLO
        self.model = YOLO(model_path)
        self.model.to(device)

        # ---- 状态 ----
        self._latest_depth: Optional[np.ndarray] = None
        self._rgb_w: int = 0
        self._rgb_h: int = 0
        self._fx: float = 0.0
        self._fy: float = 0.0
        self._cx: float = 0.0
        self._cy: float = 0.0
        self._k_calibrated: bool = False

        # ---- 发布者 ----
        qos = QoSProfile(depth=10)
        self.det_pub = self.create_publisher(Detection2DArray, '/detections', qos)
        self.img_pub = self.create_publisher(Image, '/detections_image', qos)

        # ---- 订阅者（BEST_EFFORT 兼容 Gazebo） ----
        cam_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.rgb_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self._rgb_callback, cam_qos,
        )
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self._depth_callback, cam_qos,
        )
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self._info_callback, cam_qos,
        )

        self.get_logger().info('YOLO Detector ready (RGB + depth → 3D).')

    # ── Subscriber callbacks ─────────────────────────────────────

    def _depth_callback(self, msg: Image):
        self._latest_depth = _msg_to_numpy(msg)

    def _info_callback(self, msg: CameraInfo):
        if not self._k_calibrated and msg.k[0] > 0 and self._rgb_w > 0:
            cx = msg.k[2]
            if abs(cx - self._rgb_w / 2.0) < self._rgb_w * 0.1:
                self._fx = msg.k[0]
                self._fy = msg.k[4]
                self._cx = msg.k[2]
                self._cy = msg.k[5]
                self._k_calibrated = True
                self.get_logger().info(
                    f'K from camera_info: fx={self._fx:.1f} fy={self._fy:.1f} '
                    f'cx={self._cx:.1f} cy={self._cy:.1f}'
                )

    def _rgb_callback(self, msg: Image):
        self._rgb_w = msg.width
        self._rgb_h = msg.height

        # Fallback intrinsics from HFOV if camera_info hasn't arrived
        if not self._k_calibrated and self._rgb_w > 0:
            hfov = 1.047  # ~60° horizontal FOV
            fx = (self._rgb_w / 2.0) / math.tan(hfov / 2.0)
            self._fx = fx
            self._fy = fx
            self._cx = self._rgb_w / 2.0
            self._cy = self._rgb_h / 2.0
            self.get_logger().info(
                f'K from HFOV: fx={fx:.1f} cx={self._cx:.1f} cy={self._cy:.1f} '
                f'({self._rgb_w}x{self._rgb_h})'
            )

        try:
            rgb = _msg_to_numpy(msg)
            if rgb.shape[2] == 3 and msg.encoding.startswith('bgr'):
                pass  # already BGR
        except Exception as e:
            self.get_logger().error(f'Image conversion: {e}')
            return

        results = self.model(rgb, conf=self.conf, verbose=False)
        if not results or results[0].boxes is None:
            return

        r = results[0]
        boxes = r.boxes.xyxy.cpu().numpy()
        confs = r.boxes.conf.cpu().numpy()
        clss  = r.boxes.cls.cpu().numpy()

        det_msg = Detection2DArray()
        det_msg.header.stamp = msg.header.stamp
        det_msg.header.frame_id = msg.header.frame_id or 'camera_link'

        for box, conf, cls_id in zip(boxes, confs, clss):
            x1, y1, x2, y2 = box
            w_img = float(msg.width) if msg.width > 0 else 640.0
            h_img = float(msg.height) if msg.height > 0 else 480.0

            cx_px = (x1 + x2) / 2.0
            cy_px = (y1 + y2) / 2.0

            d = Detection2D()
            d.header = det_msg.header
            d.bbox.center.position.x = float(cx_px / w_img)
            d.bbox.center.position.y = float(cy_px / h_img)
            d.bbox.size_x = float((x2 - x1) / w_img)
            d.bbox.size_y = float((y2 - y1) / h_img)

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(int(cls_id))
            hyp.hypothesis.score = float(conf)

            # 3D projection — lookup depth at bbox centre, project to camera frame
            pc = self._pixel_to_camera(cx_px, cy_px)
            if pc is not None:
                hyp.pose.pose.position.x = float(pc[0])
                hyp.pose.pose.position.y = float(pc[1])
                hyp.pose.pose.position.z = float(pc[2])

            d.results.append(hyp)
            det_msg.detections.append(d)

        self.det_pub.publish(det_msg)

        # 标注图像
        annotated = r.plot()
        anno_msg = _numpy_to_msg(annotated, 'bgr8', header=msg.header)
        self.img_pub.publish(anno_msg)

    # ── Pinhole projection ───────────────────────────────────────

    def _pixel_to_camera(self, u: float, v: float) -> Optional[tuple]:
        """Project pixel (u,v) + depth → camera optical-frame 3D (xc, yc, zc)."""
        if self._latest_depth is None:
            return None

        dh, dw = self._latest_depth.shape[:2]

        # Scale pixel coords if depth image resolution differs from RGB
        u_d = u * dw / self._rgb_w if self._rgb_w > 0 else u
        v_d = v * dh / self._rgb_h if self._rgb_h > 0 else v

        ui, vi = int(round(u_d)), int(round(v_d))
        if ui < 0 or ui >= dw or vi < 0 or vi >= dh:
            return None

        zc = float(self._latest_depth[vi, ui])
        if np.isnan(zc) or zc <= 0.01:
            return None

        xc = (u - self._cx) * zc / self._fx
        yc = (v - self._cy) * zc / self._fy
        return (xc, yc, zc)


def main():
    rclpy.init()
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
