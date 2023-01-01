"""import open3d as o3d
import numpy as np
# Load the point cloud
pcd = o3d.io.read_point_cloud("./pointclouds/object3d270.pcd",  remove_nan_points=True)


# Estimate normals for the point cloud

pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))


# Create a depth map from the point cloud
o3d.visualization.draw([pcd])
print('Running Poisson surface reconstruction ...')
mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=3)
print('Displaying reconstructed mesh ...')
o3d.visualization.draw([mesh])

"""
#above its for 1 at a atime
 
import open3d as o3d
import numpy as np

meshlist=[]
for i in range(1):
    # Load the point cloud
    pcd = o3d.io.read_point_cloud(f'./pointclouds/object3d{i+1}.pcd',  remove_nan_points=True)
    print("Currently on iteration "+str(i))

    # Estimate normals for the point cloud
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    print('Running Poisson surface reconstruction ...')
    try:
        # Create a depth map from the point cloud
        #depth is upper bound
        #mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=2)
        #meshlist.append(mesh)
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=18)
        meshlist.append(mesh)

    except:
        print("Point cloud with 0 points, unable to run surface reconstruction, proceeding")
        continue
    

print('Finished reconstruction, displaying '+ str(len(meshlist))+' reconstructed meshs at once...')
#draw all meshes
#okay isto é um erro não fazer tudo ao mm tempo

#o3d.visualization.draw([*meshlist])
#o3d.visualization.draw([meshlist[0]])
o3d.visualization.draw([mesh])

