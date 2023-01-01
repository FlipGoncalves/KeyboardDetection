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
import sys
import open3d as o3d
import numpy as np
import copy

def condition(point, moda):
    #os pontos do teclado variam entre
    #BR 0.12, -0.05
    #TR 0.12 0.15
    #BL -0.15 -0.05
    #TL -0.15 0.15
    # o centro do keyboard é +- 0 0 
    #a altura do teclado é 1.19
    specific_point = [0.0, 0.0, 1.19]  # specify the specific point here
    max_distance = 0.  # specify the maximum distance here
    if point[0]<0.15 and point[0]>-0.2 and point[1]<0.2 and point[1]>-0.7 and abs(point[2] - moda) < 0.025:
        return True
    else: return False

def touch_confirm(point,mean):
    #acho q os pontos certos são menos q media confirma
    variance=0.02
    if point[2]<mean-variance:
        return True
    else: return False

def main():
    np.set_printoptions(threshold=sys.maxsize)
    meshlist=[]

    base_num_points = 0

    key_pressed = False
    key_pressed_started = 0

    for i in range(100):

            # Load the point cloud
            pcd = o3d.io.read_point_cloud(f'./pointclouds/object3d{i+1}.pcd',  remove_nan_points=True)
            #See current image
            #o3d.visualization.draw_geometries([pcd])
            print("Currently on iteration "+str(i))
            # points_to_keep = [point for point in pcd.points if condition(point)]
            # filtered_point_cloud = o3d.geometry.PointCloud()
            # filtered_point_cloud.points = o3d.utility.Vector3dVector(points_to_keep)

            #use the following line to get a point in the point cloud
            # o3d.visualization.draw_geometries_with_vertex_selection([filtered_point_cloud])

            moda_dict = {}

            for point in np.asarray(pcd.points):
                if point[2] in moda_dict.keys():
                    moda_dict[point[2]] += 1
                else:
                    moda_dict[point[2]] = 1

            # base will be in this height with more or less 0.025 difference to the other points
            moda = sorted(moda_dict.items(), key=lambda x: x[1], reverse=True)[0][0]

            points_to_keep_base = [point for point in pcd.points if condition(point, moda)]
            base_points = o3d.geometry.PointCloud()
            base_points.points = o3d.utility.Vector3dVector(points_to_keep_base)

            base_num_points = max(base_num_points, len(base_points.points))

            if base_num_points - len(base_points.points) > 1000:

                dict = {0: 1, 1: 0}

                for point in np.asarray(base_points.points):
                    if abs(1.140 - point[2]) < 0.01:
                        dict[1] += 1
                    else:
                        dict[0] += 1

                if dict[1] >= 10:
                    if not key_pressed:
                        key_pressed_started = i
                        key_pressed = True
                    # use the following line to get a point in the point cloud
                    # o3d.visualization.draw_geometries_with_vertex_selection([base_points])

            if key_pressed and i > key_pressed_started + 20:
                key_pressed = False

            if key_pressed and i == key_pressed_started + 12:
                print("Key pressed")


            # # Estimate normals for the point cloud
            # filtered_point_cloud.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
 
            # print('Running Poisson surface reconstruction ...')
            # try:
            #     # Create a depth map from the point cloud
            #     #depth is upper bound
            #     mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(filtered_point_cloud, depth=8  )

            #     #this to view the current mesh
            #     #o3d.visualization.draw_geometries([mesh],point_show_normal=True)

            #     meshlist.append(mesh)

            #     #get the connectivity of points in the mesh
            #     with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
            #         triangle_clusters, cluster_n_triangles, cluster_area = (mesh.cluster_connected_triangles())
            #     triangle_clusters = np.asarray(triangle_clusters)
            #     cluster_n_triangles = np.asarray(cluster_n_triangles)
            #     cluster_area = np.asarray(cluster_area)    
            #     #remove small anomalies
            #     mesh_0 = copy.deepcopy(mesh)
            #     triangles_to_remove = cluster_n_triangles[triangle_clusters] < 100
            #     mesh_0.remove_triangles_by_mask(triangles_to_remove)
            #     #observer without anomalies
            #     #o3d.visualization.draw_geometries([mesh_0])
                
            #     #select biggest cluster
            #     mesh_1 = copy.deepcopy(mesh)
            #     largest_cluster_idx = cluster_n_triangles.argmax()
            #     triangles_to_remove = triangle_clusters != largest_cluster_idx
            #     mesh_1.remove_triangles_by_mask(triangles_to_remove)
            #     #go back to a could so i can check point var
            #     number_of_samples=len(mesh_1.vertices)
            #     final_cloud = mesh_1.sample_points_uniformly(number_of_samples)        

            #     o3d.visualization.draw_geometries_with_vertex_selection([final_cloud])
            #     mean, cov= final_cloud.compute_mean_and_covariance()
            #     final_points = [point for point in final_cloud.points if touch_confirm(point,mean)]
            #     print(mean)
            #     if len(final_points) !=0:
            #         print("Key pressd")
            #     #observe big
            #     #o3d.visualization.draw_geometries_with_vertex_selection([mesh_1])




            # except:
                
            #     print("An error occurred, unable to run surface reconstruction, proceeding")
            #     continue
            

    #print('Finished reconstruction, displaying '+ str(len(meshlist))+' reconstructed meshs at once...')
    #draw all meshes

    #o3d.visualization.draw([*meshlist])


if __name__ == "__main__":
    main()