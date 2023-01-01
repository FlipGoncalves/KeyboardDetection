#### Point clouds file
#
## Filipe Gonçalves - 98083
## Paulo Pereira - 98430

# imports
import sys
import open3d as o3d
import numpy as np
import copy


""" Condition function for the filtered points """
def condition(point, moda):
    # keyboard keys vary between
    # BR 0.12, -0.05
    # TR 0.12 0.15
    # BL -0.15 -0.05
    # TL -0.15 0.15

    # if the point is in between some defined variables
    # and if the standard deviation between the most used value and the height of the point is less than 0.025
    if point[0]<0.15 and point[0]>-0.2 and point[1]<0.2 and point[1]>-0.7 and abs(point[2] - moda) < 0.025:
        return True
    else: return False


""" Function to assert if the base board has been pressed or not """
def get_keypress(base_num_points, frame_num):

    # Load the point cloud
    pcd = o3d.io.read_point_cloud(f'./pointclouds/object3d{frame_num+1}.pcd',  remove_nan_points=True)
    
    # most common height value
    moda_dict = {}
    for point in np.asarray(pcd.points):
        if point[2] in moda_dict.keys():
            moda_dict[point[2]] += 1
        else:
            moda_dict[point[2]] = 1

    # base will be in this height with more or less 0.025 standard deviation to the other points
    moda = sorted(moda_dict.items(), key=lambda x: x[1], reverse=True)[0][0]

    # filter the points and create a new point cloud
    points_to_keep_base = [point for point in pcd.points if condition(point, moda)]
    base_points = o3d.geometry.PointCloud()
    base_points.points = o3d.utility.Vector3dVector(points_to_keep_base)

    # get number of points
    base_num_points = max(base_num_points, len(base_points.points))

    # if the number of points is very low, than the hand is above the board (somewhere)
    # as it creates a big "shadow" over the board
    # we need to understand the height in which the hand is
    if base_num_points - len(base_points.points) > 1000:

        # get the number of points around 1.14 as it represents the key press (start - press - finish)
        dict = {0: 1, 1: 0}
        for point in np.asarray(base_points.points):
            if abs(1.140 - point[2]) < 0.01:
                dict[1] += 1
            else:
                dict[0] += 1

        # if there are a lot of points, then it means there has been a key press
        if dict[1] >= 10:
            # o3d.visualization.draw_geometries_with_vertex_selection([base_points])
            return base_num_points, True
        
    return base_num_points, False 