import os

import torch
import tensorrt as trt
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter


class ObjectDetect(Node):
    def __init__(self):
        super().__init__('object_detect')

        model_name = self.declare_parameter("model_name", "test").value
        self.active = self.declare_parameter("active", False).value
        self.target_class = self.declare_parameter("target_class", "mallet").value

        self._trt_logger = trt.Logger(trt.Logger.WARNING)
        self._engine = self._load_engine(model_name)
        self._context = None

        self.add_on_set_parameters_callback(self.param_cb)

        self.get_logger().info("Ready")

    def param_cb(self, params: list[Parameter]):
        for param in params:
            if param.name == "active":
                if param.value:
                    self.get_logger().info("Activating object detection.")
                    self._context = self._create_context()
                    self.active = True
                else:
                    self.get_logger().info("Deactivating object detection.")
                    self.active = False
                    self._context = None

        return rclpy.parameter.ParameterEventDescriptors()

    def _create_context(self):
        if self._engine is None:
            raise RuntimeError("Engine not loaded.")

        context = self._engine.create_execution_context()
        if context is None:
            raise RuntimeError("Failed to create execution context.")

    def _load_engine(self, name: str):
        self.get_logger().info(f"Loading engine for model: {name}")

        # If the engine file doesn't exist, build it
        models_dir = get_package_share_directory('object_detect') + f"/models/"

        if not os.path.exists(models_dir + name + ".engine"):
            self.get_logger().warn(f"Engine file {name}.engine not found. Building engine...")
            engine = self._build_engine(name, models_dir)
        else:
            with open(models_dir + name + ".engine", "rb") as f:
                engine_data = f.read()
                runtime = trt.Runtime(self._trt_logger)
                engine = runtime.deserialize_cuda_engine(engine_data)

        return engine

    def _build_engine(self, name: str, models_dir: str) -> trt.ICudaEngine:
        builder = trt.Builder(self._trt_logger)
        network = builder.create_network()
        parser = trt.OnnxParser(network, self._trt_logger)

        success = parser.parse_from_file(models_dir + name + ".onnx")

        if parser.num_errors > 0:
            for i in range(parser.num_errors):
                self.get_logger().error(f"ONNX parsing error: {parser.get_error(i)}")

        if not success:
            raise RuntimeError("Failed to parse ONNX model.")

        config = builder.create_builder_config()
        config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, 1 << 24)  # 16 MiB

        engine = builder.build_serialized_network(network, config)

        with open(models_dir + name + ".engine", "wb") as f:
            f.write(engine)

        return engine


def main(args=None):
    rclpy.init(args=args)

    node = ObjectDetect()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
