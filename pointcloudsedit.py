import numpy as np
import open3d as o3d
import copy
import cv2

pointClouds = []

for i in range(70, 80):
    source = o3d.io.read_point_cloud(f'./pointclouds/object3d{i+1}.pcd')

    arr_pt = np.asarray(source.points)
    arr_pt_ret = []

    for i in range(arr_pt.shape[0]): 
        if not np.any(np.isinf(arr_pt[i])):
            if arr_pt[i][2] < 16:
                arr_pt_ret.append(arr_pt[i]) 

    pcl = o3d.geometry.PointCloud() 
    pcl.points = o3d.utility.Vector3dVector(arr_pt_ret)
    pcl.remove_duplicated_points()
    pointClouds.append(pcl)

print("Finished loading point clouds")

# cv2.namedWindow("Processed Image")
# capture = cv2.VideoCapture('VideoColor.avi')
# for i in range(10):
#     ret, frame = capture.read()
#     cv2.imshow("Processed Image", frame)
#     cv2.waitKey(30)

for pcl in pointClouds:
    o3d.visualization.draw_geometries([pcl])
    cv2.waitKey(-1)