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


def apply_filters(depth_frame):
    decimation = rs.decimation_filter()
    spatial = rs.spatial_filter()
    spatial.set_option(rs.option.holes_fill,5) #do I still need hole filling???
    hole_filling = rs.hole_filling_filter(2) #use min of neighbour cells,might need changing
    threshold_filter = rs.threshold_filter(0.3, 4.0)
    depth_to_disparity = rs.disparity_transform(True)
    disparity_to_depth = rs.disparity_transform(False)

    # spatial.set_option(rs.option.holes_fill, 3) #try 5??
    frame = depth_frame
    frame = threshold_filter.process(frame)
    frame = decimation.process(frame)
    frame = depth_to_disparity.process(frame)
    frame = spatial.process(frame)
    frame = disparity_to_depth.process(frame)
    frame = hole_filling.process(frame)
    return frame


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

def save_pointcloud():
    return 0

def testing():
    global profile
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 60)  # RGB stream
    config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 60)  # Depth stream
    config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)  # Accelerometer data
    config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 400)  # Gyroscope data
    profile = pipeline.start(config)

    frames = pipeline.wait_for_frames()
    prof = frames.get_profile()
    depth_intrinsics = prof.as_video_stream_profile().get_intrinsics()
    print(depth_intrinsics)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    depth_frame = apply_filters(depth_frame)


    depth_image = np.asanyarray(depth_frame.get_data()) * depth_scale


    print(np.min(depth_image))
    print(np.max(depth_image))
    num_zeros = np.count_nonzero(depth_image == 0)
    print(num_zeros)
    start_time = time.time()

    image_o3d = o3d.geometry.Image(depth_image.astype(np.float32))

    image_o3d = o3d.geometry.Image(depth_image.astype(np.float32))
    pcd = o3d.geometry.PointCloud.create_from_depth_image(
        image_o3d,o3d.camera.PinholeCameraIntrinsic(depth_intrinsics.width, depth_intrinsics.height,
                                                          depth_intrinsics.fx, depth_intrinsics.fy,
                                                          depth_intrinsics.ppx, depth_intrinsics.ppy))


    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]]) #flip it
    down_pcd = pcd.voxel_down_sample(voxel_size=0.05)
    # down_pcd.estimate_normals()
    # down_pcd.orient_normals_consistent_tangent_plane(100)
    # o3d.visualization.draw_geometries([down_pcd], point_show_normal=True)
    #
    # print("Printing the normal vectors ...")
    # print(np.asarray(down_pcd.normals))
    # print("Print a normal vector of the 0th point")
    # print(downpcd.normals[0])
    # print("Print the normal vectors of the first 10 points")
    # print(np.asarray(downpcd.normals)[:10, :])

    plane_model, inliers = pcd.segment_plane(distance_threshold=0.01,
                                             ransac_n=3,
                                             num_iterations=1000)
    [a, b, c, d] = plane_model
    print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
    print("Displaying pointcloud with planar points in red ...")
    inlier_cloud = pcd.select_by_index(inliers)
    inlier_cloud.paint_uniform_color([1.0, 0, 0])
    outlier_cloud = pcd.select_by_index(inliers, invert=True)
    o3d.visualization.draw([inlier_cloud, outlier_cloud])




    # plane_model, inliers = downpcd.segment_plane(distance_threshold=0.1,
    #                                      ransac_n=3,
    #                                      num_iterations=200)
    # o3d.visualization.draw_geometries([pcd])
    # o3d.visualization.draw_geometries([pcd],
    #                                   zoom=0.3412,
    #                                   front=[0.4257, -0.2125, -0.8795],
    #                                   lookat=[2.6172, 2.0475, 1.532],
    #                                   up=[-0.0694, -0.9768, 0.2024])
    # [a, b, c, d] = plane_model
    # print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
    # slope = c/a
    # # step height =
    # print(c)
    # print(a)
    # print(f"slope:{slope}")
    # inlier_cloud = downpcd.select_by_index(inliers)
    # inlier_cloud.paint_uniform_color([1.0, 0, 0])
    # outlier_cloud = downpcd.select_by_index(inliers, invert=True)
    # o3d.visualization.draw_geometries([inlier_cloud,outlier_cloud],
    #                                   zoom=0.8,
    #                                   front=[-0.4999, -0.1659, -0.8499],
    #                                   lookat=[2.1813, 2.0619, 2.0999],
    #                                   up=[0.1204, -0.9852, 0.1215])
    #
    # inlier_points = np.asarray(inlier_cloud)
    # min_z = np.min(inlier_cloud[:, 2])
    # max_z = np.max(inlier_cloud[:, 2])
    # print(f"step height: {max_z-min_z}")
    print("--- %s seconds ---" % (time.time() - start_time))

if __name__ == "__main__":
    # Specify the CSV file containing depth data
    csv_filename = "depth_19-08-06.csv"

    # Load depth data from CSV file

    print("yo")