#!/usr/bin/env python3

import sys
import time
from typing import Optional, Tuple

import cv2
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image

from mindx.sdk import base

from det_utils import (
    resize_image,
    img2input,
    std_output,
    nms,
    cod_trf,
    draw,
)


class Yolov8CamDetector(Node):
    def __init__(
        self,
        topic_name: str,
        model_path: str,
        data_yaml_path: str,
        device_id: int = 0,
        target_size: Tuple[int, int] = (640, 640),
        conf_thres: float = 0.25,
        iou_thres: float = 0.45,
        max_fps: float = 20.0,
    ) -> None:
        super().__init__("yolov8_cam_detector")

        self.window_name = f"YOLOv8: {topic_name} (press q to quit)"
        self.topic_name = topic_name
        self.target_w, self.target_h = target_size[0], target_size[1]
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.latest_frame_bgr: Optional[np.ndarray] = None
        self.last_infer_time = 0.0
        self.min_interval = 1.0 / max(1e-3, max_fps)

        # Load classes
        with open(data_yaml_path, 'r', encoding='utf-8') as f:
            yaml_data = yaml.safe_load(f)
        self.class_list = yaml_data.get('names', [])
        self.get_logger().info(f"Loaded {len(self.class_list)} classes from {data_yaml_path}")

        # Init MindX and load model
        base.mx_init()
        self.model = base.model(modelPath=model_path, deviceId=device_id)
        self.get_logger().info(f"YOLOv8 model loaded: {model_path}")

        # ROS2 subscription (BEST_EFFORT suits camera streams)
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.subscription = self.create_subscription(Image, topic_name, self._on_image, qos)

        # Periodic timer for inference/display
        self.timer = self.create_timer(0.0, self._on_timer)
        self.get_logger().info(f"Subscribing to: {topic_name}")

    def _on_image(self, msg: Image) -> None:
        width = msg.width
        height = msg.height
        encoding = (msg.encoding or "").lower()
        buf = np.frombuffer(msg.data, dtype=np.uint8)

        try:
            if encoding in ("bgr8",):
                frame_bgr = buf.reshape((height, width, 3))
            elif encoding in ("rgb8",):
                frame_rgb = buf.reshape((height, width, 3))
                frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
            elif encoding in ("mono8", "8uc1"):
                frame_gray = buf.reshape((height, width))
                frame_bgr = cv2.cvtColor(frame_gray, cv2.COLOR_GRAY2BGR)
            elif encoding in ("yuyv", "yuv422", "yuv422_yuy2", "yuy2"):
                frame_yuyv = buf.reshape((height, width, 2))
                frame_bgr = cv2.cvtColor(frame_yuyv, cv2.COLOR_YUV2BGR_YUY2)
            else:
                expected = height * width * 3
                if buf.size == expected:
                    frame_bgr = buf.reshape((height, width, 3))
                else:
                    return
            self.latest_frame_bgr = frame_bgr
        except Exception:
            # Ignore malformed frames
            pass

    def _infer_once(self, frame_bgr: np.ndarray) -> np.ndarray:
        # Preprocess with letterbox
        img_after, scale, dh, dw = resize_image(frame_bgr, (self.target_h, self.target_w), True)
        letterbox_info = (scale, dw, dh)

        # Build tensor
        data = img2input(img_after)

        # Inference
        output = self.model.infer([data])[0]
        output.to_host()
        output_np = np.array(output)

        # Postprocess
        pred = std_output(output_np)
        result = nms(pred, self.conf_thres, self.iou_thres)
        result = cod_trf(result, frame_bgr, img_after, letterbox_info)
        vis = draw(result, frame_bgr.copy(), self.class_list)
        return vis

    def _on_timer(self) -> None:
        # Throttle inference to desired FPS
        now = time.time()
        if (now - self.last_infer_time) < self.min_interval:
            # Still handle UI events
            self._poll_key()
            return
        if self.latest_frame_bgr is None:
            self._poll_key()
            return

        frame = self.latest_frame_bgr
        try:
            vis = self._infer_once(frame)
        except Exception as exc:
            self.get_logger().warn(f"Inference failed: {exc}")
            self._poll_key()
            return

        cv2.imshow(self.window_name, vis)
        self.last_infer_time = now
        self._poll_key()

    def _poll_key(self) -> None:
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info("'q' pressed, shutting down...")
            rclpy.shutdown()

    def destroy_node(self) -> None:  # type: ignore[override]
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        try:
            base.mx_deinit()
        except Exception:
            pass
        super().destroy_node()


def main(argv=None) -> None:
    argv = argv if argv is not None else sys.argv
    topic = "/camera/color/image_raw"
    model_path = "/home/HwHiAiUser/work/yolov8.om"
    data_yaml = "/home/HwHiAiUser/work/data.yaml"

    # Optional CLI overrides: topic model data_yaml
    if len(argv) > 1:
        topic = argv[1]
    if len(argv) > 2:
        model_path = argv[2]
    if len(argv) > 3:
        data_yaml = argv[3]

    rclpy.init(args=argv)
    node = Yolov8CamDetector(
        topic_name=topic,
        model_path=model_path,
        data_yaml_path=data_yaml,
        device_id=0,
        target_size=(640, 640),
        conf_thres=0.25,
        iou_thres=0.45,
        max_fps=20.0,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()


