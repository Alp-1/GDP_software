import copy
import math

import open3d as o3d
import numpy as np
import pyrealsense2 as rs
import time
import scipy
import cv2
# cell_height = 120
# cell_width = 212
# grid_n = 4
# grid_m = 4
from matplotlib import pyplot as plt

cell_height = 60
cell_width = 106 #miert??
grid_n = 8
grid_m = 8


def get_slope_grid(depth_image,depth_intrinsics,angles):
    rotation_matrix = o3d.geometry.get_rotation_matrix_from_zyx((math.radians(angles[0]),0,math.radians(angles[2])))
    cell_height = int(depth_image.shape[0] // 8)
    cell_width = int(depth_image.shape[1] // 8)
    depth_grid = scipy.sparse.bsr_matrix(depth_image, blocksize=(cell_height, cell_width), dtype=np.float32).data
    np_grid = np.asanyarray(depth_grid)
    i = 0
    j = 0
    slope_grid = np.zeros((grid_n, grid_m))
    pcds = []
    for cell in np_grid:
        image_o3d = o3d.geometry.Image(cell.astype(np.float32))
        pcd = o3d.geometry.PointCloud.create_from_depth_image(
            image_o3d, o3d.camera.PinholeCameraIntrinsic(depth_intrinsics.width, depth_intrinsics.height,
                                                         depth_intrinsics.fx, depth_intrinsics.fy,
                                                         depth_intrinsics.ppx, depth_intrinsics.ppy))
        pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])  # flip it
        downpcd = pcd.voxel_down_sample(voxel_size=0.02)

        adjusted_pcd = copy.deepcopy(downpcd)
        adjusted_pcd.rotate(rotation_matrix)
        # pcds.append(adjusted_pcd)

        if len(downpcd.points)>30:
            plane_model, inliers = adjusted_pcd.segment_plane(distance_threshold=0.05,ransac_n=3,num_iterations=1000)
            [a, b, c, d] = plane_model

            if a == 0:
                pitch_degrees = 90
            else:
                pitch = math.acos(b / math.sqrt(math.pow(a, 2) + math.pow(b, 2) + math.pow(c, 2)))
                pitch_degrees = math.degrees(pitch)
        else:
            pitch_degrees = 90
        slope_grid[i, j] = pitch_degrees

        if j == (grid_n - 1):
            i += 1
            j = 0
        else:
            j += 1

    return slope_grid
