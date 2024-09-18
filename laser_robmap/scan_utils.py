import numpy as np
from sensor_msgs.msg import LaserScan

def scan_filter(laserscan, angle_min, angle_max, angle_res):
    """
    Filtra un mensaje LaserScan para limitar el rango de ángulos y cambiar la resolución.

    :param laserscan: Mensaje de tipo LaserScan
    :param angle_min: Ángulo mínimo del rango deseado en grados
    :param angle_max: Ángulo máximo del rango deseado en grados
    :param angle_res: Nueva resolución de escaneo en grados (por defecto 0.225°)
    :return: Nuevo mensaje LaserScan con los datos filtrados y la nueva resolución
    """
    angle_min_rad = np.deg2rad(angle_min)
    angle_max_rad = np.deg2rad(angle_max)
    angle_res_rad = np.deg2rad(angle_res)

    indice_min = int((angle_min_rad - laserscan.angle_min) / laserscan.angle_increment)
    indice_max = int((angle_max_rad - laserscan.angle_min) / laserscan.angle_increment)

    filtered_ranges = laserscan.ranges[indice_min:indice_max]
    filtered_intensities = laserscan.intensities[indice_min:indice_max] if laserscan.intensities else []

    new_scan = LaserScan()
    new_scan.header = laserscan.header
    new_scan.angle_min = angle_min_rad
    new_scan.angle_max = angle_max_rad
    new_scan.time_increment = laserscan.time_increment
    new_scan.scan_time = laserscan.scan_time
    new_scan.range_min = laserscan.range_min
    new_scan.range_max = laserscan.range_max

    if angle_res_rad >= laserscan.angle_increment:
        step = int(angle_res_rad / laserscan.angle_increment)
        new_scan.angle_increment = angle_res_rad
        new_scan.ranges = filtered_ranges[::step]
        new_scan.intensities = filtered_intensities[::step] if filtered_intensities else []
    else:
        new_scan.angle_increment = laserscan.angle_increment
        new_scan.ranges = filtered_ranges
        new_scan.intensities = filtered_intensities

    return new_scan

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