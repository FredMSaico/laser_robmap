import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
import numpy as np
import math
from std_msgs.msg import Header
from laser_robmap.scan_utils import scan_filter, scan_rotation, scan_transform
from laser_robmap.cloud_utils import create_cloud_msg

class LaserScanToPointCloud(Node):

    def __init__(self):
        super().__init__('scan_to_pointcloud')

        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.declare_parameters(
            namespace='',
            parameters=[
                ('frequency', 5),
                ('angle_min', 0),
                ('angle_max', 180),
                ('angle_res', 0.225)
            ])

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )
        
        self.cloud_publisher = self.create_publisher(PointCloud2, '/cloud', qos_profile)
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        timer_period = 1.0 / self.get_parameter('frequency').value  # 5 Hz
        self.timer = self.create_timer(timer_period, self.scan_to_cloud)
        
        self.latest_scan = None

    def scan_callback(self, msg):
        laserscan = msg
        angle_min = self.get_parameter('angle_min').value
        angle_max = self.get_parameter('angle_max').value
        angle_res = self.get_parameter('angle_res').value
        self.latest_scan = scan_filter(laserscan, angle_min, angle_max, angle_res)

    def scan_to_cloud(self):
        if self.latest_scan is None:
            return

        msg = self.latest_scan
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        ranges = msg.ranges

        P_S = []
        for i, r in enumerate(ranges):
            if r < msg.range_min or r > msg.range_max:
                continue
            angle = angle_min + i * angle_increment
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            z = 0.0
            P_S.append((x, y, z))

        try:
            TF = self.tf_buffer.lookup_transform('odom', 'base_scan', rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(f'Could not transform "base_scan" to "odom": {ex}')
            return

        P_M = scan_transform(P_S, TF)
        cloud_msg = create_cloud_msg(P_M, msg.header.stamp, 'odom')
        self.cloud_publisher.publish(cloud_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanToPointCloud()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()