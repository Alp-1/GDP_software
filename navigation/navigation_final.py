import numbers
import os
from datetime import datetime

import cv2
import numpy as np
from pyrealsense import pyrealsense2 as rs
# from dronekit import connect, VehicleMode, LocationGlobalRelative
from dronekit import *
from pymavlink.rotmat import Rotation

from . import geometric_map as geo
from . import mav_listener
from .logging_config import setup_custom_logger
from . import camera_angle as cam
from .semantic_map import SemanticSegmentation
from . import mav_sender
from pymavlink.quaternion import QuaternionBase


logger = setup_custom_logger("navigation")

# Connect to the vehicle
mavlink_connection = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
mavlink_connection.wait_heartbeat()
logger.info("Heartbeat from MAVLink system (system %u component %u)" % (
    mavlink_connection.target_system, mavlink_connection.target_component))

obstacle_threshold = 1.0
deadend_threshold = 1.0
flipping_threshold_radians = 0.4
column_width = 60  
# Specify the width of the rover in meters
rover_width = 0.5  # Adjust to your rover's width
folder_name = ""
pitch_threshold = 30
target_speed = 0
previous_speed = [1,1,1,1,1]
in_tall_vegetation = False

def create_folder(folder_name):
    # Create a folder if it doesn't exist
    if not os.path.exists(folder_name):
        os.makedirs(folder_name)


def save_data_to_txt(distance, slope_grid, angle, status, height, width, current_time):
    # Save data to a text file

    filename_data = os.path.join(folder_name, f"{current_time}_data.txt")
    with open(filename_data, 'w') as file:
        file.write(f"Distance to obstacle:{distance}\n")
        file.write(f"Slope Grid:\n")
        file.write(f"{slope_grid}\n")
        file.write(f"Angle: {angle}\n")
        file.write(f"It's a deadend: {status}\n")
        file.write(f"gap height: {height}\n")
        file.write(f"gap width: {width}\n")


def save_rgb_image(image, current_time):
    filename_image = os.path.join(folder_name, f"{current_time}_image.png")
    cv2.imwrite(filename_image, image)


def mavlink_turn(velocity_x, velocity_y, velocity_z, yaw):
    """
    Turn in place yaw degrees (ignores velocity) using pymavlink.
"""
    mavlink_connection.mav.set_position_target_local_ned_send(
        0,  # time_boot_ms (not used)
        mavlink_connection.target_system,  # target system
        mavlink_connection.target_component,  # target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
        0b100111111111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        math.radians(yaw), 0)  # yaw, yaw_rate


def mavlink_velocity(velocity_x, velocity_y, velocity_z):
    """
    Move vehicle in direction based on specified velocity vectors using pymavlink.
"""
    global target_speed
    mavlink_connection.mav.set_position_target_local_ned_send(
        0,  # time_boot_ms (not used)
        mavlink_connection.target_system,  # target system
        mavlink_connection.target_component,  # target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
        0b110111100111,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate

    target_speed = velocity_x


def mavlink_turn_and_go(velocity_x, velocity_y, velocity_z, yaw):
    """
    Turn yaw degrees and move vehicle in direction based on specified velocity vectors using pymavlink.
"""
    mavlink_connection.mav.set_position_target_local_ned_send(
        0,  # time_boot_ms (not used)
        mavlink_connection.target_system,  # target system
        mavlink_connection.target_component,  # target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
        0b100111100111,  # type_mask
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        math.radians(yaw), 0)  # yaw, yaw_rate



def mavlink_go_back(thrust):
    roll = pitch = yaw = 0
    mavlink_connection.mav.set_attitude_target_send(
        0,  # time_boot_ms (not used)
        mavlink_connection.target_system,  # target system
        mavlink_connection.target_component,  # target component
        0b00100111,
        QuaternionBase([math.radians(angle) for angle in (roll, pitch, yaw)]),
        0, 0, 0, thrust  # roll rate, pitch rate, yaw rate, thrust
    )


def go_back_and_turn():
    mavlink_connection.set_mode_apm("GUIDED")
    mavlink_go_back(-1)
    time.sleep(2)

    if in_tall_vegetation:
        mavlink_velocity(0, 0, 0)
        time.sleep(1.5)
        mavlink_turn_and_go(0.2, 0, 0, 45)
        time.sleep(1)
        mavlink_velocity(0.5, 0, 0)
    else:
        deadend_protocol()

# Function to be called whenever HEARTBEAT messages are received
def heartbeat_listener(self, name, message):
    logger.info("Heartbeat received")
    logger.info("Base Mode: {}".format(message.base_mode))
    logger.info("Custom Mode: {}".format(message.custom_mode))


# Global variable for the RealSense profile
profile = None


# Initialize RealSense pipeline and profile
def initialize_realsense():
    global profile
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 60)  # RGB stream
    config.enable_stream(rs.stream.depth, 424, 240, rs.format.z16, 60)  # Depth stream
    config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)  # Accelerometer data
    config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 400)  # Gyroscope data
    profile = pipeline.start(config)

    sensor = profile.get_device().query_sensors()[1]
    sensor.set_option(rs.option.enable_auto_exposure, False)
    sensor.set_option(rs.option.exposure, 78.0)
    sensor.set_option(rs.option.gain, 90.0)

    depth_sensor = profile.get_device().query_sensors()[0]
    return pipeline, profile


def apply_filters(depth_frame):
    decimation = rs.decimation_filter()
    spatial = rs.spatial_filter()
    threshold_filter = rs.threshold_filter(0.0, 4.0)
    depth_to_disparity = rs.disparity_transform(True)
    disparity_to_depth = rs.disparity_transform(False)

    frame = depth_frame
    frame = threshold_filter.process(frame)
    frame = depth_to_disparity.process(frame)
    frame = spatial.process(frame)
    frame = disparity_to_depth.process(frame)
    return frame


def get_thresholded_image(depth_frame):
    distance_limit = 4.0
    decimation = rs.decimation_filter()
    spatial = rs.spatial_filter()
    depth_to_disparity = rs.disparity_transform(True)
    disparity_to_depth = rs.disparity_transform(False)

    frame = depth_frame
    frame = depth_to_disparity.process(frame)
    frame = spatial.process(frame)
    frame = disparity_to_depth.process(frame)

    depth_image = np.asanyarray(frame.get_data()) * depth_scale
    depth_image[depth_image > distance_limit] = distance_limit

    return depth_image


def get_new_images(frames):
    # align_to = rs.stream.color
    # align = rs.align(align_to)
    # frames = align.process(frames)
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    if not depth_frame or not color_frame:
        logger.info("problems")

    color_image = np.asanyarray(color_frame.get_data())
    steering_image = get_thresholded_image(depth_frame)
    depth_frame = apply_filters(depth_frame)
    depth_image = np.asanyarray(depth_frame.get_data()) * depth_scale

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
            logger.info("Second phase of the deadend protocol, not implemented")


def distance_to_obstacle(steering_image):
    """
    Returns the closest non-zero point in central patch of the depth image (preliminary obstacle detection)
"""
    # Calculate the size of the central square
    central_width = steering_image.shape[1] // 4
    central_height = steering_image.shape[0] // 40

    # Calculate the starting indices for the central square
    start_row = (steering_image.shape[0] - central_height) // 2
    start_col = (steering_image.shape[1] - central_width) // 2

    # Select the central square
    central_square = steering_image[start_row:start_row + central_height, start_col:start_col + central_width]

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
    """
    Returns the real world distance between 2 depth image pixels
"""
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


def get_smallest_value(steering_image, mask):
    """
    Get the distance to the closest obstacle in image patch (pixel which is not labelled ground or vegetation)
    """
    all_ground = np.all(steering_image == 6)
    contains_only_6_and_22 = np.all(np.isin(mask, [6, 22]))
    if all_ground:
        return max(np.min(steering_image), deadend_threshold+0.01)
    elif contains_only_6_and_22:
        condition_mask = (mask != 22)
    else:
        # Create a mask based on the conditions
        condition_mask = np.logical_and(mask != 6, mask != 22)

    masked_array = steering_image[condition_mask]
    if masked_array.size==0:
        return max(np.min(steering_image), deadend_threshold+0.01)

    else:
        min_value = np.min(masked_array)
    if not isinstance(min_value,numbers.Number):
        return 0
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
    """
    Window sliding algorithm to choose direction column given semantic mask, ground slopes and depth image
"""
    closest_obstacle = -1
    closest_vegetation = -1
    best_direction = 30 #half of column width
    best_vegetation_direction = 30 #half of column width

    width = steering_image.shape[1]
    height = steering_image.shape[0]
    central_square_height = steering_image.shape[0] // 40
    central_square_width = column_width
    start_row = (steering_image.shape[0] - central_square_height) // 2
    for start_col in range(2,width - central_square_width - 2):
        depth_square = steering_image[start_row:start_row + central_square_height,
                       start_col:start_col + central_square_width]
        mask_square = mask[start_row:start_row + central_square_height,
                      start_col:start_col + central_square_width]
        masked_depth = np.ma.masked_where(depth_square == 0, depth_square)
        terrain_ahead = mask[start_row:height-1,start_col:start_col+central_square_width]

        ground,vegetation,tree,other = terrain_type_distribution(terrain_ahead)

        middle_col = start_col + central_square_width // 2
        index = get_slope_index(middle_col, width)
        ground_pitch_angle = slope_grid[index]
        if (ground_pitch_angle > 35 and vegetation < 0.3) or tree>0.05: #or other>0.2:  #terrain is unsafe
            continue

        closest_point = get_smallest_value(masked_depth, mask_square)
        print(middle_col)
        print(f'terrain distribution:{ground} {vegetation} {tree} {other}')
        print(closest_point)
        if vegetation > 0.6:
            if closest_point > closest_vegetation:
                closest_vegetation = closest_vegetation
                best_vegetation_direction = middle_col
        else:
            if closest_point > closest_obstacle:
                closest_obstacle = closest_point
                best_direction = middle_col

    if closest_obstacle >= deadend_threshold: #if there is a suitable direction without vegetation, choose it over the one with vegetation
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
    width_left = column
    width_right = column
    height_up = depth_image.shape[0] // 2

    square_height = depth_image.shape[0] // 40
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


def is_tall_vegetation(steering_image, current_speed):
    global in_tall_vegetation
    in_tall_vegetation = False
    percentage_threshold = 0.6
    nr_of_pixels = steering_image.size
    percentage = np.count_nonzero(steering_image == 0) / nr_of_pixels
    logger.info(f"percentage of pixels with 0 value:{percentage}")
    if percentage > percentage_threshold:
        in_tall_vegetation = True
        return True
    else:
        return False


def is_collision(current_speed):
    max_size = 5
    speed_threshold = 0.10
    global target_speed
    global previous_speed
    previous_speed.pop(0)
    previous_speed.append(current_speed)
    for element in previous_speed:
        if element >= speed_threshold:
            return False
    target_speed = 0
    return True


def is_flipping():
    start_time = time.time()
    angles = mav_listener.get_imu_data(mavlink_connection)
    logger.info("Get mavlink imu data: --- %s seconds ---" % (time.time() - start_time))

    logger.info("Roll: %f; Pitch: %f; Yaw: %f" % (angles[0], angles[1], angles[2]))
    if abs(angles[0]) > flipping_threshold_radians or abs(angles[1]) > flipping_threshold_radians:
        mavlink_connection.set_mode_apm("GUIDED")
        logger.info("ROVER IS FLIPPING OVER")
        return True
    else:
        return False


# Function to navigate while avoiding obstacles
def navigate_avoiding_obstacles(steering_image, depth_image, color_image, dist, camera_angle):

    deadend_status = False
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

    start_time = time.time()
    vehicle_mode = mav_listener.get_mav_mode(mavlink_connection)
    logger.info("Get mavlink mode data: --- %s seconds ---" % (time.time() - start_time))

    logger.info(vehicle_mode)

    if vehicle_mode == "AUTO" or vehicle_mode == "GUIDED":
        distance = distance_to_obstacle(steering_image)
        start_time = time.time()
        current_speed = mav_listener.get_rover_speed(mavlink_connection)
        logger.info("Get mavlink speed data: --- %s seconds ---" % (time.time() - start_time))

        current_speed /= 100
        logger.info(f"current speed:{current_speed}")  
        if is_flipping():
            go_back_and_turn()
        elif is_collision(current_speed):
            logger.info("COLLISION")
            go_back_and_turn()
        elif is_tall_vegetation(steering_image, current_speed):
            logger.info("IN TALL VEGETATION")
            mavlink_connection.set_mode_apm("AUTO")
            # add command to lower speed in AUTO mode
            return
        elif distance < obstacle_threshold:
            logger.info("Obstacle detected! Taking evasive action.")
            mavlink_connection.set_mode_apm("GUIDED")
            mavlink_velocity(0, 0, 0)
            time.sleep(0.5)
            navigate_avoiding_obstacles(steering_image, depth_image, color_image, distance, camera_angle)
        else:
            logger.info("no obstacle ahead")
            mavlink_connection.set_mode_apm("AUTO")


# Main execution loop
try:
    # Get the current date and time for the folder name
    current_date_and_time = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    folder_name = f"mission_{current_date_and_time}"
    # Create a folder for the mission
    create_folder(folder_name)

    pipeline, profile = initialize_realsense()
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    logger.info("Depth Scale is: {}".format(depth_scale))
    mavlink_connection.arducopter_arm()

    frames = pipeline.wait_for_frames()
    prof = frames.get_profile()
    depth_intrinsics = prof.as_video_stream_profile().get_intrinsics()
    clf = SemanticSegmentation()
    cam.initialize_angle(frames)

    vehicle_mode = mav_listener.get_mav_mode(mavlink_connection)
    if vehicle_mode == "AUTO" or vehicle_mode == "GUIDED":
        mavlink_connection.set_mode_apm("AUTO")
        time.sleep(0.5)
        while True:
            navigate()


except KeyboardInterrupt:
    logger.info("Script terminated by user")

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    mavlink_connection.close()
    logger.info("Connection closed.")
