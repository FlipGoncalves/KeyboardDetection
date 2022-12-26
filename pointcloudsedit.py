import numpy as np
import open3d as o3d
import copy
import cv2

vis = o3d.visualization.Visualizer()
vis.create_window()

geometry = o3d.geometry.PointCloud()
vis.add_geometry(geometry)

pointClouds = []

for i in range(10):
    source = o3d.io.read_point_cloud(f'./pointclouds/object3d{i+1}.pcd')

    arr_pt = np.asarray(source.points)
    arr_pt_ret = []

    for i in range(arr_pt.shape[0]): 
        if not np.any(np.isinf(arr_pt[i])):
            if arr_pt[i][2] < 16:
                arr_pt_ret.append(arr_pt[i]) 

    pcl = o3d.geometry.PointCloud() 
    pcl.points = o3d.utility.Vector3dVector(arr_pt_ret)
    pointClouds.append(pcl)

print("Finished loading point clouds")

for i in pointClouds:
    geometry.points = pcl.points
    vis.update_geometry(geometry)
    vis.poll_events()
    vis.update_renderer()
    cv2.waitKey(100)
    # o3d.visualization.draw_geometries([pcl])