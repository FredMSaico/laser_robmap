#Libraries used
import numpy as np
import open3D as o3d
import laspy as lp

#Dataset preparation

#Gather the dataset
#Prepare the folder structure

#Create paths and load data
pcd_path = "../DATA/scanned_table.las"

#Import the dataset with laspy
point_cloud = lp.read(pcd_path)
xyz = np.vstack((point_cloud.x, point_cloud.y, point_cloud.z)).transpose()
rgb = np.vstack((point_cloud.red, point_cloud.green, point_cloud.blue)).transpose()/65535

#Transform to Open3D
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(xyz)
pcd.colors = o3d.utility.Vector3dVector(rgb)

#3. Creating a Voxel Grid

# Defining the Voxel Size 

vsize=max(pcd.get_max_bound()-pcd.get_min_bound())*0.005
vsize=round(vsize,4)

# Creating Voxel Grid
voxel_grid=o3d.geometry.VoxelGrid.create_from_point_cloud(pcd,voxel_size=vsize)

# Extracting the bounds
bounds = voxel_grid.get_max_bound()-voxel_grid.get_min_bound()
o3d.visualization.draw_geometries([voxel_grid])

#4. Generating a single Voxel Entity
cube =o3d.geometry.TriangleMesh.create_box(width = 1, height =1, depth =1)
cube.paint_uniform_color([1,0,0])

cube.compute_vertex_normals()
o3d.visualization.draw_geometries([cube])

#5. Automate and loop to create one Voxel Dataset

#grid index = integer value in a canonical space defined by the bounds
#use unit of 1 for voxel

voxels = voxel_grid.get_voxels()
vox_mesh = o3d.geometry.TriangleMesh()

for v in voxels:
    cube = o3d.geometry.TriangleMesh.create_box(width = 1, height =1, depth =1)
    cube.paint_uniform_color(v.color)
    cube.translate(v.grid_index, relative = False)
    vox_mesh += cube

o3d.visualization.draw_geometries([vox_mesh])

#6. Add Normals

vox_mesh.compute_vertex_normals()
o3d.visualization.draw_geometries([vox_mesh])

#7. Voxel Dataset Post-processing

#to align to the center of a cube of dimension 1
vox_mesh.translate([0.5,0.5,0.5], relative = True)

#to scale
vox_mesh.scale(vsize, [0,0,0])

#to translate
vox_mesh.translate(voxel_grid.origin, relative = True)

#to correct close vertices
vox_mesh.merge_close_vertices(0.0000001)

#8.Voxel Dataset Exports (3D Mesh)

o3d.io.write_triangle_mesh("..RESULTS/voxeL_mesh_heerLen_standard.pLy", vox_mesh)

#Rotate and export
T = np.array([[1,0,0,0], [0,0,1,0], [0,-1,0,0], [0,0,0,1]])
o3d.io.write_triangle_mesh("..RESULTS/voxeL_mesh_heerLen_standard.pLy", vox_mesh.transform(T))