import json
import time
from typing import List, Dict

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

try:
    from ultralytics import YOLO
except Exception:  # pragma: no cover - import error path
    YOLO = None


class YoloDetector(Node):
    def __init__(self) -> None:
        super().__init__("yolo_detector")

        self.declare_parameter("image_topic", "/image_raw")
        self.declare_parameter("annotated_topic", "/detections/image")
        self.declare_parameter("detections_topic", "/detections")
        self.declare_parameter("model", "yolov8n.pt")
        self.declare_parameter("device", "cpu")
        self.declare_parameter("conf", 0.25)
        self.declare_parameter("max_fps", 10.0)
        self.declare_parameter("allowed_classes", "")
        self.declare_parameter("blocked_classes", "")
        self.declare_parameter("min_area_ratio", 0.0)
        self.declare_parameter("max_area_ratio", 1.0)

        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        annotated_topic = self.get_parameter("annotated_topic").get_parameter_value().string_value
        detections_topic = self.get_parameter("detections_topic").get_parameter_value().string_value

        model_path = self.get_parameter("model").get_parameter_value().string_value
        device = self.get_parameter("device").get_parameter_value().string_value
        self.conf = self.get_parameter("conf").get_parameter_value().double_value
        self.max_fps = self.get_parameter("max_fps").get_parameter_value().double_value
        self.allowed_classes = self._parse_class_list(
            self.get_parameter("allowed_classes").get_parameter_value().string_value
        )
        self.blocked_classes = self._parse_class_list(
            self.get_parameter("blocked_classes").get_parameter_value().string_value
        )
        self.min_area_ratio = self.get_parameter("min_area_ratio").get_parameter_value().double_value
        self.max_area_ratio = self.get_parameter("max_area_ratio").get_parameter_value().double_value

        if YOLO is None:
            self.get_logger().error(
                "ultralytics is not installed. Install it with: pip install ultralytics"
            )
            raise RuntimeError("Missing ultralytics")

        self.get_logger().info(f"Loading YOLO model: {model_path} on {device}")
        self.model = YOLO(model_path)
        self.device = device

        self.bridge = CvBridge()
        self.last_infer = 0.0

        self.image_sub = self.create_subscription(
            Image, image_topic, self.image_cb, qos_profile_sensor_data
        )
        self.annotated_pub = self.create_publisher(Image, annotated_topic, 10)
        self.detections_pub = self.create_publisher(String, detections_topic, 10)

        self.get_logger().info(
            f"Subscribed to {image_topic}. Publishing {annotated_topic} and {detections_topic}."
        )

    def image_cb(self, msg: Image) -> None:
        now = time.time()
        if self.max_fps > 0 and (now - self.last_infer) < (1.0 / self.max_fps):
            return
        self.last_infer = now

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        detections = []
        annotated = frame

        results = self.model.predict(
            source=frame, conf=self.conf, device=self.device, verbose=False
        )

        if results and len(results) > 0:
            r = results[0]
            detections = self._to_detections(r, frame.shape[1], frame.shape[0])
            if detections:
                annotated = r.plot()

        # Always publish, even if detections is empty.
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
        annotated_msg.header = msg.header
        self.annotated_pub.publish(annotated_msg)

        out = String()
        out.data = json.dumps(detections)
        self.detections_pub.publish(out)

    def _to_detections(self, result, width: int, height: int) -> List[Dict]:
        dets = []
        names = result.names or {}
        if result.boxes is None:
            return dets

        for box in result.boxes:
            cls_id = int(box.cls.item()) if box.cls is not None else -1
            conf = float(box.conf.item()) if box.conf is not None else 0.0
            xyxy = box.xyxy[0].tolist() if box.xyxy is not None else [0, 0, 0, 0]
            x1, y1, x2, y2 = [float(v) for v in xyxy]
            w = max(0.0, x2 - x1)
            h = max(0.0, y2 - y1)
            area_ratio = (w * h) / max(1.0, float(width * height))

            class_name = names.get(cls_id, "unknown")

            if self.allowed_classes and not self._class_in_filter(
                cls_id, class_name, self.allowed_classes
            ):
                continue
            if self.blocked_classes and self._class_in_filter(
                cls_id, class_name, self.blocked_classes
            ):
                continue
            if self.min_area_ratio > 0.0 and area_ratio < self.min_area_ratio:
                continue
            if self.max_area_ratio > 0.0 and area_ratio > self.max_area_ratio:
                continue

            dets.append(
                {
                    "class_id": cls_id,
                    "class_name": class_name,
                    "confidence": conf,
                    "bbox_xyxy": [x1, y1, x2, y2],
                    "area_ratio": area_ratio,
                }
            )
        return dets

    @staticmethod
    def _parse_class_list(value: str) -> List[str]:
        if not value:
            return []
        return [item.strip() for item in value.split(",") if item.strip()]

    @staticmethod
    def _class_in_filter(class_id: int, class_name: str, items: List[str]) -> bool:
        for item in items:
            if item.isdigit() and class_id == int(item):
                return True
            if class_name.lower() == item.lower():
                return True
        return False


def main() -> None:
    rclpy.init()
    node = YoloDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
