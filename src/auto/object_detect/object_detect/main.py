import os
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node, SetParametersResult
import rclpy
from sensor_msgs.msg import Image, RegionOfInterest
from rclpy.parameter import Parameter
from auto_msgs.msg import Detection
import numpy as np
import onnxruntime as ort
import cv2

DETECT_FREQ = 1
CONF_THRESHOLD = 0.1
CLASS_NAMES = ["bottle", "mallet", "hammer"]


class ObjectDetect(Node):
    def __init__(self):
        super().__init__('object_detect', namespace="auto")

        self._model_name = self.declare_parameter("model_name", "test").value
        self.active = self.declare_parameter("active", True).value
        self.target_class = self.declare_parameter("target_class", "mallet").value

        self._img_header = None
        self._img = None
        self._session = self._load_model()

        self.add_on_set_parameters_callback(self.param_cb)

        self.create_subscription(Image, "/vision/main/image_rect_color", self.img_cb, 1)
        self._detect_pub = self.create_publisher(Detection, "object/detect", 10)
        self.create_timer(1.0 / DETECT_FREQ, self.detect_cb)

        self.get_logger().info("Ready")

    def param_cb(self, params: list[Parameter]):
        for param in params:
            if param.name == "active":
                self.active = param.value
                return SetParametersResult(successful=True)
            elif param.name == "target_class":
                # Only valid classes are "mallet", "bottle", and "hammer"
                if param.value in ["mallet", "bottle", "hammer"]:
                    self.target_class = param.value
                    return SetParametersResult(successful=True)
                else:
                    return SetParametersResult(successful=False, reason="Invalid target class.")
            elif param.name == "model_name":
                self._model_name = param.value
                self._session = self._load_model()
                return SetParametersResult(successful=True)

    def img_cb(self, msg: Image):
        # Convert ROS Image message to numpy array
        self._img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        self._img_header = msg.header

    def detect_cb(self):
        if not self.active or self._img is None or self._session is None:
            return

        in_img = self._img
        # # Letterbox to 640x640: scale to fit, pad the rest (preserves aspect ratio)
        h, w, _ = in_img.shape
        scale = 640 / max(h, w)
        new_h, new_w = int(h * scale), int(w * scale)
        pad_top = (640 - new_h) // 2
        pad_bottom = 640 - new_h - pad_top
        pad_left = (640 - new_w) // 2
        pad_right = 640 - new_w - pad_left
        in_img = cv2.resize(in_img, (new_w, new_h), interpolation=cv2.INTER_AREA)
        in_img = cv2.copyMakeBorder(in_img, pad_top, pad_bottom, pad_left, pad_right, cv2.BORDER_CONSTANT, value=0)

        in_img = in_img.astype(np.float32) / 255.0  # Normalize to [0, 1]
        in_img = np.expand_dims(in_img.transpose(2, 0, 1), axis=0)  # Convert HWC to NCHW

        out = self._session.run(["output0"], {"images": in_img})
        detections = out[0][0]  # remove batch dim -> [300, 6]
        detections = detections[detections[:, 4] >= CONF_THRESHOLD]  # filter by confidence
        target_id = CLASS_NAMES.index(self.target_class)
        detections = detections[detections[:, 5].astype(int) == target_id]  # filter by class

        detection_msg = Detection()
        detection_msg.header = self._img_header

        # # Find the detection with the highest confidence
        if len(detections) > 0:
            best_det = detections[detections[:, 4].argmax()]
            x1, y1, x2, y2, conf, cls = best_det.tolist()
            detection_msg.ids.append(int(cls))
            detection_msg.confs.append(float(conf))
            # TODO: Add basic pose estimation
            detection_msg.rois.append(RegionOfInterest(
                x_offset=int(x1 / scale) - (pad_left + pad_right),
                y_offset=int(y1 / scale) - (pad_top + pad_bottom),
                width=int((x2 - x1) / scale),
                height=int((y2 - y1) / scale)
            ))

        self._detect_pub.publish(detection_msg)
        self._img = None

    def _load_model(self):
        models_dir = get_package_share_directory('object_detect') + "/models/"
        model_path = models_dir + self._model_name + ".onnx"

        if not os.path.exists(model_path):
            self.get_logger().error(f"Model file {model_path} does not exist.")
            return None

        try:
            if ort.get_device() == "CPU":
                self.get_logger().warning("CPU being used.")
                providers = ["CPUExecutionProvider"]
            else:
                providers = ["CUDAExecutionProvider", "CPUExecutionProvider"]

            return ort.InferenceSession(model_path, providers=providers)
        except Exception as e:
            self.get_logger().error(f"Failed to load model {self._model_name}: {e}")
            return None


def main(args=None):
    rclpy.init(args=args)

    node = ObjectDetect()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
