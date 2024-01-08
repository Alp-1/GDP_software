import copy
import math
import os
import time
from datetime import datetime

import cv2
import numpy as np
import pyrealsense2 as rs
from matplotlib import pyplot as plt

import geometric_map as geo
import mav_listener
from logging_config import setup_custom_logger
import camera_angle as cam
from semantic_map import SemanticSegmentation
logger = setup_custom_logger("navigation")
import geometric_map as geo
import open3d as o3d
import numpy as np
import pyrealsense2 as rs
import time
import scipy
import cv2

from matplotlib import pyplot as plt

grid_n = 1
grid_m = 4

column_width = 50
deadend_threshold = 1.0

def get_smallest_value(steering_image, mask):
    contains_only_6_and_22 = np.all(np.isin(mask, [6, 22]))
    if contains_only_6_and_22:
        return 99
    else:
        # Create a mask based on the conditions
        condition_mask = np.logical_and(mask != 6, mask != 22)

        # Apply the mask to the depth image and get the minimum value
        min_value = np.min(steering_image[condition_mask])

        return min_value

def apply_filters(depth_frame):
    decimation = rs.decimation_filter()
    spatial = rs.spatial_filter()
    #spatial.set_option(rs.option.holes_fill,5) #do I still need hole filling???
    hole_filling = rs.hole_filling_filter(2) #use min of neighbour cells,might need changing
    threshold_filter = rs.threshold_filter(0.0, 4.0)
    depth_to_disparity = rs.disparity_transform(True)
    disparity_to_depth = rs.disparity_transform(False)

    #spatial.set_option(rs.option.holes_fill, 3) #try 5??
    frame = depth_frame
    frame = threshold_filter.process(frame)
    frame = decimation.process(frame)
    frame = depth_to_disparity.process(frame)
    frame = spatial.process(frame)
    frame = disparity_to_depth.process(frame)
    #frame = hole_filling.process(frame)
    return frame


def initialize_realsense():
    global profile
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 60)  # RGB stream
    config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 60)  # Depth stream
    config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)  # Accelerometer data
    config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 400)  # Gyroscope data
    profile = pipeline.start(config)

    sensor = profile.get_device().query_sensors()[1]
    sensor.set_option(rs.option.enable_auto_exposure, False)
    sensor.set_option(rs.option.exposure, 78.0)
    sensor.set_option(rs.option.gain, 90.0)

    depth_sensor = profile.get_device().query_sensors()[0]
    # depth_sensor.set_option(rs.option.visual_preset,4) #high density preset, medium density is 5. doesn't work rn, maybe because of no advanced mode on pi?
    return pipeline, profile


def get_thresholded_image(depth_frame):
    distance_limit = 4.0
    decimation = rs.decimation_filter()
    spatial = rs.spatial_filter()
    spatial.set_option(rs.option.holes_fill, 5)  # do I still need hole filling???
    hole_filling = rs.hole_filling_filter(2)  # use min of neighbour cells,might need changing
    depth_to_disparity = rs.disparity_transform(True)
    disparity_to_depth = rs.disparity_transform(False)

    frame = depth_frame
    frame = decimation.process(frame)
    frame = depth_to_disparity.process(frame)
    frame = spatial.process(frame)
    frame = disparity_to_depth.process(frame)
    frame = hole_filling.process(frame)

    depth_image = np.asanyarray(frame.get_data()) * depth_scale
    depth_image[depth_image>distance_limit] = distance_limit

    return depth_image

def get_new_images(frames):
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    if not depth_frame or not color_frame:
        logger.info("problems")

    color_image = np.asanyarray(color_frame.get_data())
    #steering_image = get_thresholded_image(depth_frame)
    depth_frame = apply_filters(depth_frame)
    depth_image = np.asanyarray(depth_frame.get_data()) * depth_scale

    return depth_image, depth_image, color_image,depth_frame


def distance_to_obstacle(depth_image, slope_grid):
    pitch_threshold = 30
    danger_squares = []
    central_width = slope_grid.shape[1] // 4
    central_height = 2
    min_distance = 999

    start_row = (slope_grid.shape[0] - central_height) // 2
    start_col = (slope_grid.shape[1] - central_width) //  2
    patch_height = (depth_image.shape[0]//slope_grid.shape[0])
    patch_width = (depth_image.shape[1]//slope_grid.shape[1])
    print(patch_height,patch_width)
    for row_index in range(slope_grid.shape[0]):
        for col_index in range(3,5):
            if slope_grid[row_index][col_index] > 30:
                # print(f'slope:{slope_grid[row_index][col_index]}')
                # danger_squares.append((row_index,col_index))
                # print(row_index,col_index)
                patch_start_row = (row_index)*patch_height
                patch_start_column = (col_index)*patch_width
                # print(patch_start_row,patch_start_column)
                patch_end_row = patch_start_row+patch_height
                patch_end_column = patch_start_column+patch_width
                patch = depth_image[patch_start_row:patch_end_row,patch_start_column:patch_end_column]
                closest_in_patch = np.min(np.ma.masked_where(patch == 0, patch))
                min_distance = min(min_distance,closest_in_patch)


    return min_distance


def direction_to_euler_angles(direction):
    # Normalize the direction vector
    normalized_direction = direction / np.linalg.norm(direction)

    # Calculate pitch angle
    pitch_rad = np.arcsin(normalized_direction[2])
    pitch_deg = np.degrees(pitch_rad)

    # Calculate yaw angle
    yaw_rad = np.arctan2(normalized_direction[1], normalized_direction[0])
    yaw_deg = np.degrees(yaw_rad)

    return pitch_deg, yaw_deg


def pixel_to_3d_point(u, v, depth, intrinsics):
    """
    Convert pixel coordinates to 3D point coordinates.

    Parameters:
        u (float): Pixel x-coordinate.
        v (float): Pixel y-coordinate.
        depth (float): Depth value at the pixel coordinates.
        intrinsics (dict): Camera intrinsics, including 'fx', 'fy', 'cx', and 'cy'.

    Returns:
        tuple: 3D point coordinates (X, Y, Z).
    """
    fx = intrinsics.fx
    fy = intrinsics.fy
    cx = intrinsics.ppx
    cy = intrinsics.ppy

    X = (u - cx) * depth / fx
    Y = (v - cy) * depth / fy
    Z = depth

    return X, Y, Z


def get_slope_grid_accurate(depth_image,angles):
    grid_size = (1, 2)
    grid_rows, grid_cols = grid_size
    slope_grid = np.zeros(grid_size)
    central_outliers = np.empty((0, 3))
    rows, cols = depth_image.shape
    # Calculate the size of each grid segment
    segment_rows = rows // grid_rows
    segment_cols = cols // grid_cols
    rotation_matrix = o3d.geometry.get_rotation_matrix_from_zyx((math.radians(angles[0]),0,math.radians(angles[2])))
    for n in range(grid_rows):
        for m in range(grid_cols):
            # Calculate the start and end indices for the current grid segment
            start_row = n * segment_rows
            end_row = (n + 1) * segment_rows
            start_col = m * segment_cols
            end_col = (m + 1) * segment_cols

            # Get the original indices of the current grid segment
            indices = np.array(np.meshgrid(range(start_row, end_row), range(start_col, end_col))).T.reshape(-1, 2)
            print(indices)
            # Initialize an empty array to store the results
            points_3d = np.empty((len(indices), 3))
            # Iterate through the pixel indices and apply the function
            for i, (u, v) in enumerate(indices):
                depth = depth_image[u, v]
                print(depth)
                point_3d = pixel_to_3d_point(u, v, depth, depth_intrinsics)
                points_3d[i] = point_3d

            # Create an Open3D PointCloud object
            pcd = o3d.geometry.PointCloud()
            # Set the points in the PointCloud
            pcd.points = o3d.utility.Vector3dVector(points_3d)
            pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])  # flip it
            downpcd = pcd #.voxel_down_sample(voxel_size=0.02)
            adjusted_pcd = copy.deepcopy(downpcd)
            adjusted_pcd.rotate(rotation_matrix)

            print(len(downpcd.points))
            if len(downpcd.points)>100:
                plane_model, inliers = adjusted_pcd.segment_plane(distance_threshold=0.005,ransac_n=3,num_iterations=1000)
                [a, b, c, d] = plane_model
                inlier_cloud = pcd.select_by_index(inliers)
                inlier_cloud.paint_uniform_color([1.0, 0, 0])
                outlier_cloud = pcd.select_by_index(inliers, invert=True)

                if m == 1 or m == 2:    # if it's a central segment, should generalise this
                    outlier_points = np.asarray(outlier_cloud.points)
                    print(f'outliers shape:{outlier_points.shape}')
                    central_outliers = np.vstack((central_outliers, outlier_points))

                    # Find the index of the smallest number in each sublist
                    min_indices = np.argmin(outlier_points, axis=0)

                    # Extract the sublist with the smallest number for each element
                    result_rows = outlier_points[min_indices, np.arange(outlier_points.shape[1])]
                    print("\nLists with Smallest Numbers:")
                    print(result_rows)
                    print(f'nr of outlier points: {len(outlier_points)}')
                if a == 0:
                    pitch_degrees = 90
                else:
                    pitch_degrees,_ = direction_to_euler_angles(np.array([a,b,c]))
                    print(f"{i} {j}: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0   pitch:{pitch_degrees}")
            else:
                pitch_degrees = 90
            slope_grid[n, m] = pitch_degrees
    print(central_outliers.shape)
    # central_outliers[:, 2] *= -1
    return slope_grid, central_outliers


def new_obstacle_dist(depth_image,slope_grid, outlier_points):
    slope_threshold = 30
    print(outlier_points[:10])
    min_distance = 999
    # Calculate the size of the central square
    central_width = depth_image.shape[1] // 4
    central_height = depth_image.shape[0] // 20

    # Calculate the starting indices for the central square
    start_row = (depth_image.shape[0] - central_height) // 2
    start_col = (depth_image.shape[1] - central_width) // 2

    # Select the central square
    central_square = depth_image[start_row:start_row + central_height, start_col:start_col + central_width]
    printed = False
    # Create a masked array where 0 values are masked
    masked_array = np.ma.masked_where(central_square == 0, central_square)

    if slope_grid[1]>slope_threshold or slope_grid>slope_threshold:
        return 0.3
    else:
        for i in range(start_row,start_row+central_height):
            for j in range(start_col,start_col+central_width):
                if depth_image[i,j] != 0:
                    dist = depth_image[i,j]
                    point = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [i, j], dist)
                    rounded_point = np.round(point,decimals=2)
                    if not printed:
                        print(rounded_point)
                        printed = True
                    point_found = np.any(np.all(outlier_points == rounded_point, axis=1))
                    if point_found:
                        print(f'pixel:{i} {j}')
                        print(point)


# my_array = np.array([[1, 2, 3, 5],
#                      [4, 5, 6, 5],
#                      [7, 8, 9, 5]])
# get_slope_grid_accurate(my_array,5)

def get_slope_index(column, width):

    index = math.ceil(column / (width / 4)) - 1
    return index

def percentage_of_elements_equal_to_value(arr, value):
    total_elements = arr.size
    matching_elements = np.count_nonzero(arr == value)
    percentage = matching_elements / total_elements
    return percentage


def clearest_path(steering_image, slope_grid, mask):
    closest_obstacle = -1
    closest_vegetation = -1
    best_direction = 0
    best_vegetation_direction = 0

    width = steering_image.shape[1]
    central_square_height = steering_image.shape[0] // 40
    central_square_width = column_width
    start_row = (steering_image.shape[0] - central_square_height) // 2
    for start_col in range(width - central_square_width - 1):
        depth_square = steering_image[start_row:start_row + central_square_height,
                       start_col:start_col + central_square_width]
        mask_square = mask[start_row:start_row + central_square_height,
                      start_col:start_col + central_square_width]
        masked_depth = np.ma.masked_where(depth_square == 0, depth_square)
        # Calculate the middle index of the selected columns
        middle_col = start_col + central_square_width // 2
        index = get_slope_index(middle_col, width)
        ground_pitch_angle = slope_grid[index]
        print(f'Slope grid:{slope_grid}')
        print(get_slope_index(middle_col,width))
        print(f'pitch angle for seg:{ground_pitch_angle}')
        print(f'percent:{percentage_of_elements_equal_to_value(mask_square, 22)}')
        if ground_pitch_angle > 35 and percentage_of_elements_equal_to_value(mask_square,22) < 0.3:  # and not veg!!
            continue

        closest_point = get_smallest_value(masked_depth, mask_square)
        if closest_point > closest_obstacle:
            closest_obstacle = closest_point
            best_direction = middle_col
        if percentage_of_elements_equal_to_value(mask_square, 22) > 0.6:
            if closest_point > closest_vegetation:
                closest_vegetation = closest_vegetation
                best_vegetation_direction = middle_col

    if closest_obstacle >= deadend_threshold:
        return best_direction
    else:
        if closest_vegetation != -1:
            return best_vegetation_direction
        else:
            return best_direction




# Main execution loop
try:

    np.set_printoptions(suppress=True,precision=2)
    pipeline, profile = initialize_realsense()
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    logger.info("Depth Scale is: {}".format(depth_scale))


    frames = pipeline.wait_for_frames()
    prof = frames.get_profile()
    depth_intrinsics = prof.as_video_stream_profile().get_intrinsics()
    cam.initialize_angle(frames)
    clf = SemanticSegmentation()
    align_to = rs.stream.depth
    align = rs.align(align_to)
    pc = rs.pointcloud()
    while True:
        print(f'principal point: {depth_intrinsics.ppx} {depth_intrinsics.ppy}')
        frames = pipeline.wait_for_frames()
        frames = align.process(frames)

        steering_image, depth_image, color_image,depth_frame = get_new_images(frames)
        angle = cam.get_camera_angle(frames)
        print(f'camera angle:{angle}')

        start_time = time.time()
        slope_grid,central_outlier_points = geo.get_slope_grid(depth_image,depth_intrinsics,angle)
        print(slope_grid)
        print("Slope Grid: --- %s seconds ---" % (time.time() - start_time))

        # start_time = time.time()
        # slope_grid,central_outlier_points = get_slope_grid_accurate(depth_image,angle)
        # print(slope_grid)
        # print("Slope Grid(manual): --- %s seconds ---" % (time.time() - start_time))

        start_time = time.time()
        mask = clf.get_semantic_map(color_image)
        start_row = 0
        central_square_height = 5
        start_col = 10
        central_square_width = 10
        mask_square = mask[start_row:start_row + central_square_height, start_col:start_col + central_square_width]

        print(percentage_of_elements_equal_to_value(mask_square ,22))
        # logger.info("Segmenting image --- %s seconds ---" % (time.time() - start_time))
        # start_time = time.time()
        # new_obstacle_dist(depth_image,1,central_outlier_points)
        # print("Obstacle detection: --- %s seconds ---" % (time.time() - start_time))

        print(clearest_path(depth_image,slope_grid,mask))


        time.sleep(10)
except KeyboardInterrupt:
    logger.info("Script terminated by user")

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
