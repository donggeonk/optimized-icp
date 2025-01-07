import os
import copy
import numpy as np
import open3d as o3d
import argparse

"""
Author: Donggeon Kim
Purpose: apply Colored ICP with optimized initial matrices to register source to target point clouds and output the matrix
Input: path to the target and source point cloud(s)
Output: transformation matrix in text file(s)

Date: 06/05/2023
Last revised: 07/27/2023
"""

#-------inital matrices-------
"""
List of initial matrices to optimize the point cloud registrations
Note: change the home_path according to the EMMA number
"""

home_path = "/Users/stevehawkim/Desktop/Temple_Allen/ICP"
# home_path = os.path.expanduser("~") + "/Tools/PointClouds_Registration/"
# home_path = "/home/tai26/catkin_ws"

matrix11 = np.identity(4)
matrix21 = np.asarray( # camera 2->1
                [[-0.51178556,  0.68080639,  0.52400208, -0.1398875],
                [-0.66926754, -0.69837118,  0.25369003, -0.08854452],
                [0.53866175, -0.22086271,  0.8130579,   0.03292804],
                [0.000000000000, 0.000000000000, 0.000000000000, 1.000000000000]])
matrix31 = np.asarray( # camera 3->1
                [[-0.89586144,  0.08384809,  0.4363505,  -0.45382248],
                [-0.40650243, -0.55117984, -0.72866769,  0.72264574],
                [0.17941019, -0.83016278,  0.52786524,  0.36465019],
                [0.000000000000,  0.000000000000,  0.000000000000, 1.000000000000]])
matrix41 = np.asarray( # camera 4->1
                [ [0.85646193,  0.0183795,   0.51588288, -0.16416221],
                [0.40048383,  0.60690206, -0.68650026,  0.863016],
                [-0.32570791,  0.7945641,   0.51242778,  0.26333272],
                [0.000000000000,  0.000000000000,  0.000000000000 , 1.000000000000]])
matrix = [matrix11, matrix21, matrix31, matrix41]


#-------colored ICP-------
"""
Backend algorithm - uses RGBD value to optimize registration of colored point clouds
Note: down_sample makes the execution faster but with less accuracy
"""
def colored_icp_registration(source, target, voxel_size, max_corres_dist, index):
    # optimized hyperparameters - DO NOT CHANGE
    voxel_radius = [5*voxel_size, 3*voxel_size, voxel_size]
    max_iter = [60, 35, 20]

    # initial matrice corresponding to the source to optimize the registration
    current_transformation = matrix[index - 1]

    # loop through each hyperparameter to gradually register source to target
    for scale in range(3):
        max_it = max_iter[scale]
        radius = voxel_radius[scale]

        # down sample
        # source_down = source.voxel_down_sample(radius)
        # target_down = target.voxel_down_sample(radius)

        # estimate normals of source point cloud
        source.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius*2, max_nn=30))
        source.orient_normals_to_align_with_direction(orientation_reference=np.array([0., 0., 1.]))
        
        # estimate normals of target point cloud
        target.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius*2, max_nn=30))
        target.orient_normals_to_align_with_direction(orientation_reference=np.array([0., 0., 1.]))

        # applying Open3D's colored ICP algorithm
        result = o3d.pipelines.registration.registration_colored_icp(
            source, 
            target, 
            radius, 
            current_transformation,
            o3d.pipelines.registration.TransformationEstimationForColoredICP(),
            o3d.pipelines.registration.ICPConvergenceCriteria(relative_fitness=1e-6,
                                                              relative_rmse=1e-6,
                                                              max_iteration=max_it))
        
        # obtain the matrix used to register source to target 
        current_transformation = result.transformation
    return current_transformation


#-------take terminal input arguments-------
def input():
    """
    Take the terminal inputs of target and source paths as a list
    Note: input the target path first, followed by sources
    """
    parser = argparse.ArgumentParser(description="source and target pcd path")
    parser.add_argument('pcd_paths', nargs='*', help='Input pcd paths separated by spaces')

    args = parser.parse_args()

    # edge cases: user writes less than two arguments
    if (len(args.pcd_paths) < 2):
        raise Exception("Need more than two pcd path arguments to register")

    return args.pcd_paths


#-------main-------
if __name__ == '__main__':
    # optimized hyperparameters - DO NOT CHANGE
    voxel_size = 0.01
    max_corres_dist = 20*voxel_size

    # take path from terminal arguments
    pcd_paths = input()

    # split target and source paths
    target_path = pcd_paths.pop(0)
    # create the Open3D point clouds object from the pcd
    target = o3d.io.read_point_cloud(os.path.join(home_path, target_path))
    
    pcds_before = [target]
    pcds_after = [target]

    # loop through the source paths and apply colored icp on target
    print("Applying Colored ICP registration...")
    for source_path in pcd_paths:
        # path to the source.pcd
        path = os.path.join(home_path, source_path)
        # create the Open3D point clouds object from the pcd
        source = o3d.io.read_point_cloud(path)
        # reads the camera number
        source_num = source_path[3]

        pcds_before.append(copy.deepcopy(source))

        # apply colored ICP
        t_matrix = colored_icp_registration(source, target, voxel_size, max_corres_dist, int(source_num))

        pcds_after.append(source.transform(t_matrix))

        print("Transformation matrix from camera " + source_num + " to camera 1:")
        print(t_matrix)

        # building an output directory
        path_arr = path.split("/")
        output_folder = "/".join(path_arr[:-1])

        # saving matrix as text file
        matrix_save = os.path.join(output_folder, path_arr[-1].replace(".", "_") + "_matrix.txt")
        np.savetxt(matrix_save, t_matrix, fmt='%f')
    
    if (len(pcd_paths) > 1):
        print("Saved the transformation matrices as text files")
    else:
        print("Saved the transformation matrix as a text file")

    # visualize the transformed point clouds to confirm the alignment
    print("Before")
    o3d.visualization.draw_geometries(pcds_before)
    print("Visualizing the registration...")
    o3d.visualization.draw_geometries(pcds_after)
