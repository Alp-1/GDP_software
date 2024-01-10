
import math
import os
from datetime import datetime

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

def save_rgb_image(image, current_time):
    folder_name = "test"
    filename_image = os.path.join(folder_name, f"{current_time}_image.png")
    cv2.imwrite(filename_image, image)


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

    # device = rs.context().query_devices()[0]
    # advnc_mode = rs.rs400_advanced_mode(device)
    # depth_table_control_group = advnc_mode.get_depth_table()
    # depth_table_control_group.disparityShift = 128
    # advnc_mode.set_depth_table(depth_table_control_group)

    # depth_sensor.set_option(rs.option.visual_preset,4) #high density preset, medium density is 5. doesn't work rn, maybe because of no advanced mode on pi?
    return pipeline, profile

def apply_filters(depth_frame):
    decimation = rs.decimation_filter()
    spatial = rs.spatial_filter()
    #spatial.set_option(rs.option.holes_fill, 5)  # do I still need hole filling???
    #hole_filling = rs.hole_filling_filter(2)  # use min of neighbour cells,might need changing
    threshold_filter = rs.threshold_filter(0.0, 4.0)
    depth_to_disparity = rs.disparity_transform(True)
    disparity_to_depth = rs.disparity_transform(False)

    # spatial.set_option(rs.option.holes_fill, 3) #try 5??
    frame = depth_frame
    frame = threshold_filter.process(frame)
    frame = decimation.process(frame)
    frame = depth_to_disparity.process(frame)
    frame = spatial.process(frame)
    frame = disparity_to_depth.process(frame)
    #frame = hole_filling.process(frame)
    return frame


def get_thresholded_image(depth_frame):
    distance_limit = 4.0
    decimation = rs.decimation_filter()
    spatial = rs.spatial_filter()
    # spatial.set_option(rs.option.holes_fill, 5)  # do I still need hole filling???
    # hole_filling = rs.hole_filling_filter(2)  # use min of neighbour cells,might need changing
    depth_to_disparity = rs.disparity_transform(True)
    disparity_to_depth = rs.disparity_transform(False)

    frame = depth_frame
    frame = decimation.process(frame)
    frame = depth_to_disparity.process(frame)
    frame = spatial.process(frame)
    frame = disparity_to_depth.process(frame)
    # frame = hole_filling.process(frame)

    depth_image = np.asanyarray(frame.get_data()) * depth_scale
    depth_image[depth_image > distance_limit] = distance_limit

    return depth_image


def get_new_images(frames):
    align_to = rs.stream.color
    align = rs.align(align_to)
    frames = align.process(frames)
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    if not depth_frame or not color_frame:
        logger.info("problems")

    color_image = np.asanyarray(color_frame.get_data())
    steering_image = get_thresholded_image(depth_frame)
    depth_frame = apply_filters(depth_frame)
    frames = align.process(frames)
    depth_image = np.asanyarray(depth_frame.get_data()) * depth_scale

    # num_zeros = np.count_nonzero(depth_image == 0)
    # logger.info(f"Number of zero values in depth image:{num_zeros}")
    return steering_image, depth_image, color_image



def is_deadend(steering_image, mask,direction_column):
    square_height = steering_image.shape[0] // 40
    square_width = column_width
    start_row = (steering_image.shape[0] - square_height) // 2
    start_col = direction_column - (square_width // 2)

    square = steering_image[start_row:start_row + square_height, start_col:start_col + square_width]
    mask_square = mask[start_row:start_row + square_height, start_col:start_col + square_width]
    # Create a masked array where 0 values are masked
    masked_array = np.ma.masked_where(square == 0,
                                      square)  # this might be bad, it also excludes points that are closer than minz

    closest_point = get_smallest_value(masked_array, mask_square)
    vegetation_percentage = percentage_of_elements_equal_to_value(mask_square, 22)

    # # Find the minimum value while excluding masked values (0s)
    # min_value_without_zeros = np.min(masked_array)
    # mean_dist = np.mean(masked_array)
    # logger.info(f"Distance to obstacle in chosen direction: {min_value_without_zeros}")
    # logger.info(f"Distance to obstacle in chosen direction(mean): {mean_dist}")
    # if min_value_without_zeros < deadend_threshold:
    #     return True
    # else:
    #     return False
    if closest_point < deadend_threshold and vegetation_percentage < 0.6:
        return True
    else:
        return False


def deadend_protocol():
    mavlink_velocity(0, 0, 0)
    time.sleep(1)
    mavlink_turn(0, 0, 0, 45)
    time.sleep(1)
    frames = pipeline.wait_for_frames()
    steering_image, depth_image, color_image = get_new_images(frames)

    camera_angle = cam.get_camera_angle(frames)
    start_time = time.time()
    slope_grid,central_outlier_points = geo.get_slope_grid(depth_image, depth_intrinsics, camera_angle)
    logger.info("Creating slope grid: --- %s seconds ---" % (time.time() - start_time))

    start_time = time.time()
    mask = clf.get_semantic_map(color_image)
    logger.info("Segmenting image --- %s seconds ---" % (time.time() - start_time))
    mask = cv2.resize(mask, (steering_image.shape[1], steering_image.shape[0]), interpolation=cv2.INTER_NEAREST)

    new_column, new_angle = find_clear_path_and_calculate_direction(steering_image, slope_grid,mask,depth_image, rover_width)
    if not is_deadend(steering_image, mask,new_column):
        movement_commands(new_angle)
    else:
        mavlink_turn(0, 0, 0, 270)
        time.sleep(1)
        frames = pipeline.wait_for_frames()
        steering_image, depth_image, color_image = get_new_images(frames)
        camera_angle = cam.get_camera_angle(frames)
        start_time = time.time()
        slope_grid,central_outlier_points = geo.get_slope_grid(depth_image, depth_intrinsics, camera_angle)
        logger.info("Creating slope grid: --- %s seconds ---" % (time.time() - start_time))

        start_time = time.time()
        mask = clf.get_semantic_map(color_image)
        logger.info("Segmenting image --- %s seconds ---" % (time.time() - start_time))
        mask = cv2.resize(mask, (steering_image.shape[1], steering_image.shape[0]), interpolation=cv2.INTER_NEAREST)

        new_column, new_angle = find_clear_path_and_calculate_direction(steering_image, slope_grid, mask, depth_image,
                                                                        rover_width)

        if not is_deadend(steering_image, mask,new_column):
            movement_commands(new_angle)
        else:
            logger.info("Alp stuff")


#             other stuff


def distance_to_obstacle(steering_image):
    # Calculate the size of the central square
    central_width = steering_image.shape[1] // 4
    central_height = steering_image.shape[0] // 40

    # Calculate the starting indices for the central square
    start_row = (steering_image.shape[0] - central_height) // 2
    start_col = (steering_image.shape[1] - central_width) // 2

    # Select the central square
    central_square = steering_image[start_row:start_row + central_height, start_col:start_col + central_width]
    # mask_square = mask[start_row:start_row + central_height, start_col:start_col + central_width]

    # Create a masked array where 0 values are masked
    masked_array = np.ma.masked_where(central_square == 0, central_square)

    # closest_point = get_smallest_value(masked_array, mask_square)


    # Find the minimum value while excluding masked values (0s)
    min_value_without_zeros = np.min(masked_array)
    mean_dist = np.mean(masked_array)
    logger.info(f"distance to obstacle (min): {min_value_without_zeros}")
    logger.info(f"distance to obstacle (mean): {mean_dist}")

    return min_value_without_zeros


def calculate_distance(depth_image, y1, x1, y2, x2):
    # udist = depth_frame.get_distance(x1, y1)
    # vdist = depth_frame.get_distance(x2, y2)
    udist = depth_image[y1, x1]
    vdist = depth_image[y2, x2]

    point1 = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [y1, x1], udist)
    point2 = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [y2, x2], vdist)

    # euclidean distance between two points, measured in meters
    dist = math.sqrt(
        math.pow(point1[0] - point2[0], 2) + math.pow(point1[1] - point2[1], 2) + math.pow(
            point1[2] - point2[2], 2))
    # result[0]: right, result[1]: down, result[2]: forward
    return dist

def get_slope_index(column, width):

    index = math.ceil(column / (width / 4)) - 1
    return index


def percentage_of_elements_equal_to_value(arr, value):
    total_elements = arr.size
    matching_elements = np.count_nonzero(arr == value)
    percentage = matching_elements / total_elements
    return percentage


def get_smallest_value(steering_image, mask): #what if all 22
    all_ground = np.all(steering_image == 6)
    contains_only_6_and_22 = np.all(np.isin(mask, [6, 22]))
    if all_ground:
        return deadend_threshold + 0.01
    elif contains_only_6_and_22:
        condition_mask = (mask != 22)
    else:
        # Create a mask based on the conditions
        condition_mask = np.logical_and(mask != 6, mask != 22)

    masked_array = steering_image[condition_mask]
    min_value = np.min(masked_array)

    return min_value

def terrain_type_distribution(patch):
    terrain_id = 6
    vegetation_id = 22
    tree_id = 25
    other_id = 99
    total_elements = patch.size

    matching_elements = np.count_nonzero(patch == terrain_id)
    terrain_percentage = matching_elements / total_elements
    matching_elements = np.count_nonzero(patch == vegetation_id)
    vegetation_percentage = matching_elements / total_elements
    matching_elements = np.count_nonzero(patch == tree_id)
    tree_percentage = matching_elements / total_elements
    matching_elements = np.count_nonzero(patch == other_id)
    other_percentage = matching_elements / total_elements
    return terrain_percentage,vegetation_percentage,tree_percentage,other_percentage


def clearest_path(steering_image, slope_grid, mask):
    closest_obstacle = -1
    closest_vegetation = -1
    best_direction = 30 #half of column width
    best_vegetation_direction = 30 #half of column width

    width = steering_image.shape[1]
    height = steering_image.shape[0]
    central_square_height = steering_image.shape[0] // 40
    central_square_width = column_width
    start_row = (steering_image.shape[0] - central_square_height) // 2
    printed =False
    for start_col in range(width - central_square_width - 1):
        depth_square = steering_image[start_row:start_row + central_square_height,
                       start_col:start_col + central_square_width]
        mask_square = mask[start_row:start_row + central_square_height,
                      start_col:start_col + central_square_width]
        masked_depth = np.ma.masked_where(depth_square == 0, depth_square)
        terrain_ahead = mask[start_row:height-1,start_col:start_col+central_square_width]

        ground,vegetation,tree,other = terrain_type_distribution(terrain_ahead)
        if not printed:
            printed = True
            print(terrain_ahead.shape)
            print(f'terrain distribution:{ground} {vegetation} {tree} {other}')
        middle_col = start_col + central_square_width // 2
        index = get_slope_index(middle_col, width)
        ground_pitch_angle = slope_grid[index]
        if (ground_pitch_angle > 35 and vegetation < 0.3) or tree>0.05 or other>0.2:  #terrain is unsafe
            continue

        closest_point = get_smallest_value(masked_depth, mask_square)
        if vegetation > 0.6:
            if closest_point > closest_vegetation:
                closest_vegetation = closest_vegetation
                best_vegetation_direction = middle_col
        else:
            if closest_point > closest_obstacle:
                closest_obstacle = closest_point
                best_direction = middle_col

    if closest_obstacle >= deadend_threshold:
        return best_direction
    else:
        if closest_vegetation != -1:
            return best_vegetation_direction
        else:
            return best_direction


# Function to find a clear path and calculate its direction
def find_clear_path_and_calculate_direction(steering_image, slope_grid,mask,depth_image, rover_width):
    start_time = time.time()
    index_of_highest_mean = clearest_path(steering_image,slope_grid,mask)
    print("Choosing direction: --- %s seconds ---" % (time.time() - start_time))
    angle = index_of_highest_mean / steering_image.shape[1] * 87 - (87 / 2)
    angle = (angle + 360) % 360
    return index_of_highest_mean, angle


def gap_size(depth_image, column):
    gap_threshold = 0.5
    # start_row = depth_image.shape[0] // 2
    width_left = column
    width_right = column
    height_up = depth_image.shape[0] // 2

    square_height = depth_image.shape[0] // 40
    # square_width = column_width
    start_row = (depth_image.shape[0] - square_height) // 2
    end_row = start_row + square_height
    gap_width = 9999.0
    for row in range(start_row, end_row + 1):
        width_left = column
        width_right = column
        if depth_image[row, column] != 0:
            while width_left > 1:
                difference = depth_image[row, width_left] - depth_image[row, width_left - 1]
                if difference > gap_threshold and depth_image[row, width_left - 1] < (2 * obstacle_threshold) and \
                        depth_image[row, width_left - 1] < depth_image[row, column] and depth_image[
                    row, width_left - 1] != 0:
                    break
                else:
                    width_left -= 1
            width_left -= 1

            while width_right < (depth_image.shape[1] - 2):
                difference = depth_image[row, width_right] - depth_image[row, width_right + 1]
                if difference > gap_threshold and depth_image[row, width_right + 1] < (2 * obstacle_threshold) and \
                        depth_image[row, width_right + 1] < depth_image[row, column] and depth_image[
                    row, width_right + 1] != 0:
                    break
                else:
                    width_right += 1
            width_right += 1
            width = calculate_distance(depth_image, row, width_left, row, width_right)
            if width > 0:
                gap_width = min(width, gap_width)

    while height_up > 1:
        difference = depth_image[height_up, column] - depth_image[height_up - 1, column]
        if difference > gap_threshold and depth_image[height_up - 1, column] < (2 * obstacle_threshold) and depth_image[
            height_up - 1, column] < depth_image[row, column] and depth_image[height_up - 1, column] != 0:
            break
        else:
            height_up -= 1
    height_up -= 1

    gap_height = calculate_distance(depth_image, row, column, height_up, column)
    logger.info(f"gap boundary pixels:{width_left} {width_right}")
    logger.info(f"gap: height from camera:{gap_height} width:{gap_width}")
    return gap_height, gap_width


def movement_commands(angle):
    mavlink_turn_and_go(0.2, 0, 0, angle)
    time.sleep(1)
    mavlink_velocity(0.5, 0, 0)
    # time.sleep(0.3) #the rover should only go forward blindly until the next image is processed


def is_tall_vegetation(steering_image, current_speed):
    percentage_threshold = 0.5
    nr_of_pixels = steering_image.size
    percentage = np.count_nonzero(steering_image == 0) / nr_of_pixels
    logger.info(f"percentage of pixels with 0 value:{percentage}")
    if percentage > percentage_threshold and current_speed > 0.3:
        return True
    else:
        return False


def is_collision(current_speed):
    if current_speed < 0.2 and target_speed > 0:
        return True
    else:
        return False


def avoid_flipping():
    angles = mav_listener.get_imu_data(mavlink_connection)
    logger.info("Roll: %f; Pitch: %f; Yaw: %f" % (angles[0], angles[1], angles[2]))
    if abs(angles[0]) > flipping_threshold_radians or abs(angles[1]) > flipping_threshold_radians:
        mavlink_connection.set_mode_apm("GUIDED")
        logger.info("ROVER IS FLIPPING OVER")
        # mavlink_velocity(0, 0, 0)
        # time.sleep(0.5)
        mav_sender.move_backward(mavlink_connection,0.5)


# Function to navigate while avoiding obstacles
def navigate_avoiding_obstacles(steering_image, depth_image, color_image, dist, camera_angle):
    vehicle_mode = mav_listener.get_mav_mode(mavlink_connection)
    logger.info(vehicle_mode)
    deadend_status = False
    if vehicle_mode == "AUTO" or vehicle_mode == "GUIDED":
        mavlink_connection.set_mode_apm("GUIDED")
        current_time = datetime.now().strftime("%H-%M-%S")
        start_time = time.time()
        slope_grid,central_outlier_points = geo.get_slope_grid(depth_image, depth_intrinsics, camera_angle)
        logger.info("Creating slope grid: --- %s seconds ---" % (time.time() - start_time))

        start_time = time.time()
        mask = clf.get_semantic_map(color_image)
        logger.info("Segmenting image --- %s seconds ---" % (time.time() - start_time))
        mask = cv2.resize(mask, (steering_image.shape[1], steering_image.shape[0]), interpolation=cv2.INTER_NEAREST)
        column_index, angle = find_clear_path_and_calculate_direction(steering_image, slope_grid,mask,depth_image, rover_width)
        logger.info(f"direction:{angle} column:{column_index}")
        if is_deadend(steering_image, mask,column_index):
            logger.info("deadend")
            deadend_status = True



        gap_height, gap_width = gap_size(depth_image, column_index)
        save_data_to_txt(dist, slope_grid, angle, deadend_status, gap_height, gap_width, current_time)
        save_rgb_image(color_image, current_time)

        if deadend_status:
            deadend_protocol()
        else:
            movement_commands(angle)


def navigate():
    frames = pipeline.wait_for_frames()
    steering_image, depth_image, color_image = get_new_images(frames)
    camera_angle = cam.get_camera_angle(frames)
    vehicle_mode = mav_listener.get_mav_mode(mavlink_connection)
    if vehicle_mode == "AUTO" or vehicle_mode == "GUIDED":
        avoid_flipping()
        distance = distance_to_obstacle(steering_image)
        current_speed = mav_listener.get_rover_speed(mavlink_connection)
        current_speed /= 100
        logger.info(f"current speed:{current_speed}")  # to test if target speed can be used for collision detection

        if is_collision(current_speed):
            logger.info("COLLISION")
            mav_sender.move_backward(mavlink_connection, 0.5)
        if is_tall_vegetation(steering_image, current_speed):
            logger.info("IN TALL VEGETATION")
            mavlink_connection.set_mode_apm("AUTO")
            return
            # add command to lower speed in AUTO mode
        if distance < 0.7 * obstacle_threshold:
            logger.info("Obstacle is very close! Stopping")
            mavlink_connection.set_mode_apm("GUIDED")
            mavlink_velocity(0, 0, 0)
            time.sleep(0.5)

        if distance < obstacle_threshold:
            logger.info("Obstacle detected! Taking evasive action.")
            mavlink_connection.set_mode_apm("GUIDED")
            mavlink_velocity(0, 0, 0)
            time.sleep(0.5)
            navigate_avoiding_obstacles(steering_image, depth_image, color_image, distance, camera_angle)
        else:
            logger.info("no obstacle ahead")
            mavlink_connection.set_mode_apm("AUTO")


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

#
# def get_slope_grid_accurate(depth_image,angles):
#     grid_size = (1, 2)
#     grid_rows, grid_cols = grid_size
#     slope_grid = np.zeros(grid_size)
#     central_outliers = np.empty((0, 3))
#     rows, cols = depth_image.shape
#     # Calculate the size of each grid segment
#     segment_rows = rows // grid_rows
#     segment_cols = cols // grid_cols
#     rotation_matrix = o3d.geometry.get_rotation_matrix_from_zyx((math.radians(angles[0]),0,math.radians(angles[2])))
#     for n in range(grid_rows):
#         for m in range(grid_cols):
#             # Calculate the start and end indices for the current grid segment
#             start_row = n * segment_rows
#             end_row = (n + 1) * segment_rows
#             start_col = m * segment_cols
#             end_col = (m + 1) * segment_cols
#
#             # Get the original indices of the current grid segment
#             indices = np.array(np.meshgrid(range(start_row, end_row), range(start_col, end_col))).T.reshape(-1, 2)
#             print(indices)
#             # Initialize an empty array to store the results
#             points_3d = np.empty((len(indices), 3))
#             # Iterate through the pixel indices and apply the function
#             for i, (u, v) in enumerate(indices):
#                 depth = depth_image[u, v]
#                 print(depth)
#                 point_3d = pixel_to_3d_point(u, v, depth, depth_intrinsics)
#                 points_3d[i] = point_3d
#
#             # Create an Open3D PointCloud object
#             pcd = o3d.geometry.PointCloud()
#             # Set the points in the PointCloud
#             pcd.points = o3d.utility.Vector3dVector(points_3d)
#             pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])  # flip it
#             downpcd = pcd #.voxel_down_sample(voxel_size=0.02)
#             adjusted_pcd = copy.deepcopy(downpcd)
#             adjusted_pcd.rotate(rotation_matrix)
#
#             print(len(downpcd.points))
#             if len(downpcd.points)>100:
#                 plane_model, inliers = adjusted_pcd.segment_plane(distance_threshold=0.005,ransac_n=3,num_iterations=1000)
#                 [a, b, c, d] = plane_model
#                 inlier_cloud = pcd.select_by_index(inliers)
#                 inlier_cloud.paint_uniform_color([1.0, 0, 0])
#                 outlier_cloud = pcd.select_by_index(inliers, invert=True)
#
#                 if m == 1 or m == 2:    # if it's a central segment, should generalise this
#                     outlier_points = np.asarray(outlier_cloud.points)
#                     print(f'outliers shape:{outlier_points.shape}')
#                     central_outliers = np.vstack((central_outliers, outlier_points))
#
#                     # Find the index of the smallest number in each sublist
#                     min_indices = np.argmin(outlier_points, axis=0)
#
#                     # Extract the sublist with the smallest number for each element
#                     result_rows = outlier_points[min_indices, np.arange(outlier_points.shape[1])]
#                     print("\nLists with Smallest Numbers:")
#                     print(result_rows)
#                     print(f'nr of outlier points: {len(outlier_points)}')
#                 if a == 0:
#                     pitch_degrees = 90
#                 else:
#                     pitch_degrees,_ = direction_to_euler_angles(np.array([a,b,c]))
#                     print(f"{i} {j}: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0   pitch:{pitch_degrees}")
#             else:
#                 pitch_degrees = 90
#             slope_grid[n, m] = pitch_degrees
#     print(central_outliers.shape)
#     # central_outliers[:, 2] *= -1
#     return slope_grid, central_outliers


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

DS5_product_ids = ["0AD1", "0AD2", "0AD3", "0AD4", "0AD5", "0AF6", "0AFE", "0AFF", "0B00", "0B01", "0B03", "0B07", "0B3A", "0B5C"]

def find_device_that_supports_advanced_mode() :
    ctx = rs.context()
    ds5_dev = rs.device()
    devices = ctx.query_devices();
    for dev in devices:
        if dev.supports(rs.camera_info.product_id) and str(dev.get_info(rs.camera_info.product_id)) in DS5_product_ids:
            if dev.supports(rs.camera_info.name):
                print("Found device that supports advanced mode:", dev.get_info(rs.camera_info.name))
            return dev
    raise Exception("No D400 product line device that supports advanced mode was found")


# Main execution loop
try:
    dev = find_device_that_supports_advanced_mode()
    advnc_mode = rs.rs400_advanced_mode(dev)
    print("Advanced mode is", "enabled" if advnc_mode.is_enabled() else "disabled")

    while not advnc_mode.is_enabled():
        print("Trying to enable advanced mode...")
        advnc_mode.toggle_advanced_mode(True)
        # At this point the device will disconnect and re-connect.
        print("Sleeping for 5 seconds...")
        time.sleep(5)
        # The 'dev' object will become invalid and we need to initialize it again
        dev = find_device_that_supports_advanced_mode()
        advnc_mode = rs.rs400_advanced_mode(dev)
        print("Advanced mode is", "enabled" if advnc_mode.is_enabled() else "disabled")

    depth_table_control_group = advnc_mode.get_depth_table()
    depth_table_control_group.disparityShift = 128
    advnc_mode.set_depth_table(depth_table_control_group)






    np.set_printoptions(suppress=True,precision=2)
    pipeline, profile = initialize_realsense()
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    logger.info("Depth Scale is: {}".format(depth_scale))


    frames = pipeline.wait_for_frames()
    prof = frames.get_profile()
    depth_intrinsics = prof.as_video_stream_profile().get_intrinsics()
    print(f'depth intrinsics {depth_intrinsics}')
    cam.initialize_angle(frames)
    clf = SemanticSegmentation()
    pc = rs.pointcloud()

    color_stream = profile.get_stream(rs.stream.color)
    color_video_stream = color_stream.as_video_stream_profile()
    # color_intrinsic = depth_aligned_to_color_intrinsic = color_video_stream.get_intrinsic()
    # print(f'color intrinsics:{color_intrinsic}')

    while True:

        print(f'principal point: {depth_intrinsics.ppx} {depth_intrinsics.ppy}')
        frames = pipeline.wait_for_frames()

        steering_image, depth_image, color_image = get_new_images(frames)
        current_time = datetime.now().strftime("%H-%M-%S")
        save_rgb_image(color_image,current_time)

        nr_of_pixels = steering_image.size
        percentage = np.count_nonzero(steering_image == 0) / nr_of_pixels
        logger.info(f"percentage of pixels with 0 value - steering:{percentage}")
        print(np.max(steering_image))
        print(np.min(steering_image))
        nr_of_pixels = depth_image.size
        percentage = np.count_nonzero(depth_image == 0) / nr_of_pixels
        logger.info(f"percentage of pixels with 0 value - depth:{percentage}")
        print(np.max(depth_image))
        print(np.min(depth_image))

        #
        angle = cam.get_camera_angle(frames)
        print(f'camera angle:{angle}')
        #
        start_time = time.time()
        slope_grid,central_outlier_points = geo.get_slope_grid(depth_image,depth_intrinsics,angle)
        print(slope_grid)
        print("Slope Grid: --- %s seconds ---" % (time.time() - start_time))
        #
        # # start_time = time.time()
        # # slope_grid,central_outlier_points = get_slope_grid_accurate(depth_image,angle)
        # # print(slope_grid)
        # # print("Slope Grid(manual): --- %s seconds ---" % (time.time() - start_time))
        #
        # start_time = time.time()
        mask = clf.get_semantic_map(color_image)
        # if isinstance(mask, np.ndarray):
        #     print("It's a NumPy array.")
        # else:
        #     print("It's not a NumPy array.")
        # print("Segmenting image --- %s seconds ---" % (time.time() - start_time))
        #
        #
        # print(f'depth shape{depth_image.shape}')
        # print(f'mask shape:{mask.shape}')
        # # logger.info("Segmenting image --- %s seconds ---" % (time.time() - start_time))
        # # start_time = time.time()
        # # new_obstacle_dist(depth_image,1,central_outlier_points)
        # # print("Obstacle detection: --- %s seconds ---" % (time.time() - start_time))
        mask = cv2.resize(mask, (depth_image.shape[1], depth_image.shape[0]), interpolation=cv2.INTER_NEAREST)

        # print(f'mask new shape:{mask.shape}')
        # start_time = time.time()
        # print(clearest_path(depth_image,slope_grid,mask))
        # print("Choosing direction: --- %s seconds ---" % (time.time() - start_time))
        # time.sleep(10)
        print(mask.shape)
        print(get_smallest_value(steering_image, mask))
        print(is_deadend(steering_image, mask, 25))
        print(clearest_path(steering_image, slope_grid, mask))
except KeyboardInterrupt:
    logger.info("Script terminated by user")

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
