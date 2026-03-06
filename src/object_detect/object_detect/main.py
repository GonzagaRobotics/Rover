import os
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node, SetParametersResult
import rclpy
from sensor_msgs.msg import Image
from rclpy.parameter import Parameter
import numpy as np
import onnxruntime as ort

DETECT_FREQ = 5


class ObjectDetect(Node):
    def __init__(self):
        super().__init__('object_detect')

        self._sess = None
        self._img = None

        self._model_name = self.declare_parameter("model_name", "test").value
        self.active = self.declare_parameter("active", False).value
        self.target_class = self.declare_parameter("target_class", "mallet").value

        self.add_on_set_parameters_callback(self.param_cb)

        self.create_subscription(Image, "/vision/image/clean", self.img_cb, 1)

        self.create_timer(1.0 / DETECT_FREQ, self.detect_cb)

        self.get_logger().info("Ready")

    def param_cb(self, params: list[Parameter]):
        for param in params:
            if param.name == "active":
                if param.value:
                    self._sess = self._load_model()

                    if self._sess is None:
                        return SetParametersResult(successful=False, reason="Failed to load model.")

                    self.active = True
                else:
                    self.active = False
                    self._sess = None
            elif param.name == "target_class":
                # Only valid classes are "mallet", "bottle", and "hammer"
                if param.value in ["mallet", "bottle", "hammer"]:
                    self.target_class = param.value
                else:
                    return SetParametersResult(successful=False, reason="Invalid target class.")
            elif param.name == "model_name":
                return SetParametersResult(successful=False, reason="Model name cannot be changed at runtime.")

        return SetParametersResult(successful=True)

    def img_cb(self, msg: Image):
        if not self.active or self._sess is None:
            return

        # Convert ROS Image message to numpy array
        self._img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)

    def detect_cb(self):
        if not self.active or self._sess is None:
            return

        if self._img is None:
            self.get_logger().warning("No image received yet.")
            return

        input_name = self._sess.get_inputs()[0].name
        # TODO: Use async inference?
        res = self._sess.run(None, {input_name: self._img.astype(np.float16)})
        print(res)

    def _load_model(self):
        models_dir = get_package_share_directory('object_detect') + f"/models/"
        model_path = models_dir + self._model_name + ".onnx"

        if not os.path.exists(model_path):
            self.get_logger().error(f"Model file {model_path} does not exist.")
            return None

        try:
            sess = ort.InferenceSession(model_path, providers=['CUDAExecutionProvider'])

            return sess
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
