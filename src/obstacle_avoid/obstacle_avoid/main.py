import time
import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py.point_cloud2 import read_points_numpy, create_cloud
from nav_msgs.msg import OccupancyGrid

class ObstacleAvoid(Node):
    def __init__(self):
        super().__init__("obstacle_avoid")

        self.cost_pub = self.create_publisher(OccupancyGrid, "/costmap", 1)
        self.clean_pub = self.create_publisher(PointCloud2, "/clean_pcl", 1)
        self.create_subscription(PointCloud2, "/camera/camera/depth/color/points", self.pcl_cb, 1)

        self.get_logger().info("Ready")

    def pcl_cb(self, msg: PointCloud2):
        start_t = time.perf_counter()
        points = read_points_numpy(msg, ["x", "y", "z"], skip_nans=True)

        # ROS2 X becomes Open3D Z, ROS2 Y becomes Open3D -X, ROS2 Z becomes Open3D Y

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        pcd = pcd.voxel_down_sample(voxel_size=0.05)

        pcd.rotate(pcd.get_rotation_matrix_from_xyz((0, 0, np.pi)), center=(0, 0, 0))

        # Remove points that are too far away
        aabb = o3d.geometry.AxisAlignedBoundingBox(min_bound=(-5, -3, 0), max_bound=(5, 3, 4))
        pcd = pcd.crop(aabb)


        clean_pcl = self._to_pointcloud2(pcd, msg.header.frame_id)
        self.clean_pub.publish(clean_pcl)

        # # Create a 2D costmap from the point cloud by projecting the points onto the ground plane
        # costmap = np.zeros((40, 40), dtype=np.float32)

        # for p in pcd.points:
        #     x, y, _ = p

        #     i = int((x + 2) / 0.1)
        #     j = int((y + 2) / 0.1)
        #     if 0 <= i < costmap.shape[0] and 0 <= j < costmap.shape[1]:
        #         costmap[i, j] += 1

        # # Publish the costmap
        # cost_msg = OccupancyGrid()
        # cost_msg.header.frame_id = msg.header.frame_id
        # cost_msg.header.stamp = self.get_clock().now().to_msg()
        # cost_msg.info.width = costmap.shape[1]
        # cost_msg.info.height = costmap.shape[0]
        # cost_msg.info.resolution = 0.1
        # cost_msg.info.origin.position.x = -2.0
        # cost_msg.info.origin.position.y = -2.0
        # cost_msg.data = (costmap * 100).astype(np.int8).flatten().tolist()

        # self.cost_pub.publish(cost_msg)

        total_t = time.perf_counter() - start_t
        self.get_logger().info(f"Processing time: {int(total_t * 1000)} ms")

    def _to_pointcloud2(self, pcd: o3d.geometry.PointCloud, frame_id: str) -> PointCloud2:
        header = Header()
        header.frame_id = frame_id
        header.stamp = self.get_clock().now().to_msg()

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)
        ]

        return create_cloud(header, fields, np.asarray(pcd.points))

def main(args=None):
    rclpy.init(args=args)

    node = ObstacleAvoid()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass