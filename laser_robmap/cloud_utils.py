from std_msgs.msg import Header
from sensor_msgs.msg import PointField
from sensor_msgs_py import point_cloud2
import laspy
import datetime
import open3d as o3d
import numpy as np

def create_cloud_msg(points, stamp, frame_id):
    header = Header()
    header.stamp = stamp
    header.frame_id = frame_id
    
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]

    cloud_msg = point_cloud2.create_cloud(header, fields, points)
    return cloud_msg

def save_cloud(points, pcd, ply, las, voxel, voxel_size):
    if points:
        point_cloud = o3d.geometry.PointCloud()
        points_np = np.array(points)
        point_cloud.points = o3d.utility.Vector3dVector(points_np[:, :3])

        vox_mesh = voxelization(point_cloud, voxel_size) if voxel else None

        now = datetime.datetime.now()
        timestamp = now.strftime("%Y%m%d_%H%M%S")

        if pcd:
            filename_pcd = f"laser_robmap_{timestamp}.pcd"
            o3d.io.write_point_cloud(filename_pcd, point_cloud)

        if ply:
            filename_ply = f"laser_robmap_{timestamp}.ply"
            o3d.io.write_point_cloud(filename_ply, point_cloud)
            if voxel and vox_mesh:
                o3d.io.write_triangle_mesh(f'voxel_laser_map_{timestamp}.ply', vox_mesh)

        if las:
            try:
                header = laspy.LasHeader(point_format=3, version="1.2")
                las_file = laspy.LasData(header)
                las_file.x = points_np[:, 0]
                las_file.y = points_np[:, 1]
                las_file.z = points_np[:, 2]
                filename_las = f"laser_robmap_{timestamp}.las"
                las_file.write(filename_las)
            except Exception as e:
                print(f"Error saving LAS file: {e}")

def voxelization(point_cloud, voxel_size):
    voxel_grid = o3d.geometry.VoxelGrid.create_from_point_cloud(point_cloud, voxel_size)
    voxels = voxel_grid.get_voxels()
    vox_mesh = o3d.geometry.TriangleMesh()
    
    for v in voxels:
        cube = o3d.geometry.TriangleMesh.create_box(width=1, height=1, depth=1)
        cube.paint_uniform_color([0.5, 0.5, 0.5]) 
        cube.translate(v.grid_index, relative=False)
        vox_mesh += cube    
    vox_mesh.merge_close_vertices(0.0000001)
    return vox_mesh