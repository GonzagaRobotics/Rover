import os
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node, SetParametersResult
import rclpy
from sensor_msgs.msg import Image
from rclpy.parameter import Parameter
import numpy as np
import torch

DETECT_FREQ = 0.2
CONF_THRESHOLD = 0.2
CLASS_NAMES = ["bottle", "mallet", "hammer"]


class ObjectDetect(Node):
    def __init__(self):
        super().__init__('object_detect')

        self._model_name = self.declare_parameter("model_name", "test").value
        self.active = self.declare_parameter("active", True).value
        self.target_class = self.declare_parameter("target_class", "mallet").value

        self._img = None
        # Set when the model is loaded
        self._device = None 
        self._model = None
        self._load_model()

        self.add_on_set_parameters_callback(self.param_cb)

        self.create_subscription(Image, "/vision/main/image_rect_color", self.img_cb, 1)

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
                return SetParametersResult(successful=False, reason="Model cannot be changed at runtime.")

    def img_cb(self, msg: Image):
        if not self.active:
            return

        # Convert ROS Image message to numpy array
        self._img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)

    def detect_cb(self):
        if not self.active or self._model is None:
            return

        if self._img is None:
            return

        self.get_logger().info("Running detection...")

        with torch.no_grad():
            input_tensor = torch.from_numpy(self._img.astype(np.float32) / 255.0).to(self._device).to(torch.float16)
            # Add batch dimension and convert HWC to CHW format
            input_tensor = input_tensor.permute(2, 0, 1).unsqueeze(0)
            # Letterbox to 640x640: scale to fit, pad the rest (preserves aspect ratio)
            _, _, h, w = input_tensor.shape
            scale = 640 / max(h, w)
            new_h, new_w = int(h * scale), int(w * scale)
            input_tensor = torch.nn.functional.interpolate(
                input_tensor.float(), size=(new_h, new_w), mode='bilinear', align_corners=False
            ).to(torch.float16)
            pad_top = (640 - new_h) // 2
            pad_bottom = 640 - new_h - pad_top
            pad_left = (640 - new_w) // 2
            pad_right = 640 - new_w - pad_left
            input_tensor = torch.nn.functional.pad(input_tensor, (pad_left, pad_right, pad_top, pad_bottom))
            res = self._model(input_tensor)
        # np.savetxt("/tmp/det.txt", res[0].cpu().numpy())
        detections = res[0]  # remove batch dim -> [300, 6]
        detections = detections[detections[:, 4] >= CONF_THRESHOLD]  # filter by confidence
        target_id = CLASS_NAMES.index(self.target_class)
        detections = detections[detections[:, 5].int() == target_id]  # filter by class

        for det in detections:
            x1, y1, x2, y2, conf, cls = det.tolist()
            self.get_logger().info(
                f"Detected {CLASS_NAMES[int(cls)]} conf={conf:.2f} box=[{x1:.0f},{y1:.0f},{x2:.0f},{y2:.0f}]"
            )

        self._img = None

    def _load_model(self):
        models_dir = get_package_share_directory('object_detect') + "/models/"
        model_path = models_dir + self._model_name + ".torchscript"

        if not os.path.exists(model_path):
            self.get_logger().error(f"Model file {model_path} does not exist.")
            return None

        try:
            if not torch.cuda.is_available():
                self.get_logger().warning("CUDA not available, using CPU.")

            device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
            model = torch.jit.load(model_path, map_location=device)
            model.eval()
            self._device = device
            self._model = model
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
