import rclpy
import numpy as np
from rclpy.time import Time
from sensor_msgs.msg import LaserScan

def scan_filter(input_scan: LaserScan, lower_angle: float, upper_angle: float, angle_res: float) -> LaserScan:
    """
    Filters a LaserScan message by removing points outside the specified angle range and adjusts the angular resolution.

    :param input_scan: Input LaserScan message.
    :param lower_angle: Lower angle of the allowed range.
    :param upper_angle: Upper angle of the allowed range.
    :param angle_res: Desired angular resolution.
    :return: Filtered LaserScan message.
    """
    filtered_scan = LaserScan()
    filtered_scan.header = input_scan.header
    
    start_angle = input_scan.angle_min
    current_angle = input_scan.angle_min
    start_time = Time.from_msg(input_scan.header.stamp)
    
    filtered_ranges = []
    filtered_intensities = []
    
    if angle_res < input_scan.angle_increment:
        angle_res = input_scan.angle_increment
    
    skip_points = int(angle_res / input_scan.angle_increment)
    
    for i, range_value in enumerate(input_scan.ranges):
        if start_angle < lower_angle:
            start_angle += input_scan.angle_increment
            current_angle += input_scan.angle_increment
            start_time = start_time + rclpy.duration.Duration(seconds=input_scan.time_increment)
        else:
            if (current_angle - start_angle) % angle_res < input_scan.angle_increment:
                filtered_ranges.append(range_value)
                if input_scan.intensities:
                    filtered_intensities.append(input_scan.intensities[i])
            
            if current_angle + input_scan.angle_increment > upper_angle:
                break
            
            current_angle += input_scan.angle_increment
    
    filtered_scan.angle_min = start_angle
    filtered_scan.angle_max = current_angle
    filtered_scan.angle_increment = angle_res
    filtered_scan.time_increment = input_scan.time_increment * skip_points
    filtered_scan.scan_time = input_scan.scan_time
    filtered_scan.range_min = input_scan.range_min
    filtered_scan.range_max = input_scan.range_max
    
    filtered_scan.ranges = filtered_ranges
    filtered_scan.intensities = filtered_intensities if input_scan.intensities else []
    
    return filtered_scan

def scan_rotation(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2*(y**2 + z**2), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x**2 + y**2)]
    ])

def scan_transform(points, transform):
    """
    Transforma los puntos del marco S al marco M

    :T: Translacion
    :q: quaternos
    :R: matris de rotacion
    :P_S: Puntos en marco S
    :p_M: Puntos en marco M
    :return: Puntos transformados
    """
    if len(points) == 0:
        print("Warning: LaserScan is empty.")
        return []

    T = np.array([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z])
    q = transform.transform.rotation
    R = scan_rotation(q)

    P_S = np.array(points)
    P_M = np.dot(P_S, R.T) + T

    return list(map(tuple, P_M))