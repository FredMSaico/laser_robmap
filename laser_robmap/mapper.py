import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
import open3d as o3d
import numpy as np
from laser_robmap.cloud_utils import create_cloud_msg, save_cloud

class PointCloudMap(Node):

    def __init__(self):
        super().__init__('cloud_to_map')

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.declare_parameters(
            namespace='',
            parameters=[
                ('frequency', 2),
                ('PCD', True),
                ('PLY', True),
                ('LAS', False),
                ('Voxelization', True),
                ('Voxel_size', 0.05)
            ])

        self.subscription = self.create_subscription(PointCloud2, '/cloud', self.cloud_callback, qos_profile)
        self.subscription = self.create_subscription(Odometry, '/odometry/filtered', self.odom_callback, qos_profile)
        self.map_publisher = self.create_publisher(PointCloud2, '/map', qos_profile)
        self.pcd = self.get_parameter('PCD').value
        self.ply = self.get_parameter('PLY').value
        self.las = self.get_parameter('LAS').value
        self.voxel = self.get_parameter('Voxelization').value
        self.voxel_size = self.get_parameter('Voxel_size').value

        self.last_x = 0.0
        self.last_y = 0.0
        self.last_yaw = 0.0
        self.x = None
        self.y = None
        self.yaw = None

        self.stamp = None
        self.frame_id = None
        self.accumulated_points = []

        timer_period = 1.0 / self.get_parameter('frequency').value
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if self.stamp is not None:
            map_msg = create_cloud_msg(self.accumulated_points, self.stamp, self.frame_id)
            self.map_publisher.publish(map_msg)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        xq, yq, zq, wq = msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        self.yaw = np.arctan2(2*(wq*zq + xq*yq), wq*wq - xq*xq - yq*yq + zq*zq)

    def cloud_callback(self, msg):
        P_M = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        self.stamp = msg.header.stamp
        self.frame_id = msg.header.frame_id
        if self.x is not None and self.y is not None and self.yaw is not None:
            if abs(self.x - self.last_x) > 0.01 or abs(self.yaw - self.last_yaw) > 0.01:
                self.accumulated_points.extend(P_M)
                self.last_x = self.x
                self.last_y = self.y
                self.last_yaw = self.yaw

def main(args=None):
    rclpy.init(args=args)
    cloud_to_map = PointCloudMap()
    try:
        rclpy.spin(cloud_to_map)
    except KeyboardInterrupt:
        pass
    finally:
        save_cloud(cloud_to_map.accumulated_points, cloud_to_map.pcd, cloud_to_map.ply, cloud_to_map.las, cloud_to_map.voxel, cloud_to_map.voxel_size)
        cloud_to_map.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()