import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy
import numpy as np

class PointCloudMap(Node):

    def __init__(self):
        super().__init__('point_cloud_map')

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.subscription = self.create_subscription(
            PointCloud2,
            '/cloud',
            self.cloud_callback,
            qos_profile
        )
        self.publisher = self.create_publisher(PointCloud2, '/map', qos_profile)
        self.accumulated_points = []

    def cloud_callback(self, msg):
        # Convert PointCloud2 message to a list of points
        points = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
        
        # Apply voxel grid filter to reduce points
        filtered_points = self.apply_voxel_grid_filter(points, voxel_size=0.005)  # Adjust voxel size as needed
        
        self.accumulated_points.extend(filtered_points)
        
        # Create a new PointCloud2 message with the accumulated points
        self.publish_accumulated_points(msg.header.stamp, msg.header.frame_id)

    def apply_voxel_grid_filter(self, points, voxel_size):
        voxel_dict = {}
        filtered_points = []

        for point in points:
            voxel_key = (
                int(point[0] / voxel_size),
                int(point[1] / voxel_size),
                int(point[2] / voxel_size)
            )
            if voxel_key not in voxel_dict:
                voxel_dict[voxel_key] = point
        
        filtered_points = list(voxel_dict.values())
        return filtered_points

    def publish_accumulated_points(self, stamp, frame_id):
        header = Header()
        header.stamp = stamp
        header.frame_id = frame_id

        fields = [
            point_cloud2.PointField(name='x', offset=0, datatype=point_cloud2.PointField.FLOAT32, count=1),
            point_cloud2.PointField(name='y', offset=4, datatype=point_cloud2.PointField.FLOAT32, count=1),
            point_cloud2.PointField(name='z', offset=8, datatype=point_cloud2.PointField.FLOAT32, count=1),
        ]

        point_cloud_msg = point_cloud2.create_cloud(header, fields, self.accumulated_points)
        self.publisher.publish(point_cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudMap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()