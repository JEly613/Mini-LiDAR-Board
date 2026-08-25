import open3d as o3d

# 1. Load the point cloud
pcd = o3d.io.read_point_cloud("scan-20260818-215117.ply")

# 2. Estimate normals (Poisson needs these to know which way is 'out')
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
pcd.orient_normals_consistent_tangent_plane(100)

# 3. Run Poisson Surface Reconstruction
# A depth of 9 is usually a great starting point for detailed rooms
mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=30)

# 4. Save the result
o3d.io.write_triangle_mesh("reconstructed_room.obj", mesh)
print("Mesh successfully created!")
