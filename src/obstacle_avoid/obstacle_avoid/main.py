import time
import numpy as np
import open3d as o3d
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
from nav_msgs.msg import OccupancyGrid

class ObstacleAvoid(Node):
    def __init__(self):
        super().__init__("obstacle_avoid")

        self.cost_pub = self.create_publisher(OccupancyGrid, "/costmap", 1)
        self.create_subscription(PointCloud2, "/rs/depth/color/points", self.pcl_cb, 1)

        self.get_logger().info("Ready")

    def pcl_cb(self, msg: PointCloud2):
        start_t = time.perf_counter()
        points = read_points_numpy(msg, ["x", "y", "z"], skip_nans=True)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # Remove points that are too far away
        aabb = o3d.geometry.AxisAlignedBoundingBox(min_bound=(-2, -2, -1), max_bound=(2, 2, 2))
        pcd = pcd.crop(aabb)

        pcd.voxel_down_sample(voxel_size=0.05)

        # Create a 2D costmap from the point cloud by projecting the points onto the ground plane
        costmap = np.zeros((40, 40), dtype=np.float32)

        for p in pcd.points:
            x, y, _ = p

            i = int((x + 2) / 0.1)
            j = int((y + 2) / 0.1)
            if 0 <= i < costmap.shape[0] and 0 <= j < costmap.shape[1]:
                costmap[i, j] += 1

        # Publish the costmap
        cost_msg = OccupancyGrid()
        cost_msg.header.frame_id = msg.header.frame_id
        cost_msg.header.stamp = self.get_clock().now().to_msg()
        cost_msg.info.width = costmap.shape[1]
        cost_msg.info.height = costmap.shape[0]
        cost_msg.info.resolution = 0.1
        cost_msg.info.origin.position.x = -2.0
        cost_msg.info.origin.position.y = -2.0
        cost_msg.data = (costmap * 100).astype(np.int8).flatten().tolist()

        self.cost_pub.publish(cost_msg)

        total_t = time.perf_counter() - start_t
        self.get_logger().info(f"Processing time: {int(total_t * 1000)} ms")

def main(args=None):
    rclpy.init(args=args)

    node = ObstacleAvoid()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass