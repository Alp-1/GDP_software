import os
from datetime import datetime

import cv2
import numpy as np
import pyrealsense2 as rs
# from dronekit import connect, VehicleMode, LocationGlobalRelative
from dronekit import *
import geometric_map as geo

# Connect to the vehicle
mavlink_connection = mavutil.mavlink_connection('/dev/serial0', baud=57600)
mavlink_connection.wait_heartbeat()
print("Heartbeat from MAVLink system (system %u component %u)" % (
    mavlink_connection.target_system, mavlink_connection.target_component))
vehicle = connect('/dev/serial0', wait_ready=False, baud=57600)

obstacle_threshold = 0.8
vegetation_threshold = 0.017
column_width = 40  # might need adjusting
# Specify the width of the rover in meters
rover_width = 0.5  # Adjust to your rover's width
folder_name = ""
# deadend_status = False

def create_folder(folder_name):
    # Create a folder if it doesn't exist
    if not os.path.exists(folder_name):
        os.makedirs(folder_name)

def save_data_to_txt(slope_grid, distance, angle, status, height, width, current_time):
    # Save data to a text file

    filename_data = os.path.join(folder_name, f"{current_time}_data.txt")
    with open(filename_data, 'w') as file:
        file.write(f"Slope Grid:\n")
        file.write(f"{slope_grid}\n")
        file.write(f"Distance: {distance}\n")
        file.write(f"Angle: {angle}\n")
        file.write(f"It's a deadend: {status}\n")
        file.write(f"gap height: {height}\n")
        file.write(f"gap width: {width}\n")

def save_rgb_image(image,current_time):
    filename_image = os.path.join(folder_name, f"{current_time}_image.png")
    cv2.imwrite(filename_image, image)


def mavlink_turn(velocity_x, velocity_y, velocity_z, yaw):
    """
    Move vehicle in direction based on specified velocity vectors using pymavlink.
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

    print("sexy")

# Function to be called whenever HEARTBEAT messages are received
def heartbeat_listener(self, name, message):
    print("Heartbeat received")
    print("Base Mode: {}".format(message.base_mode))
    print("Custom Mode: {}".format(message.custom_mode))

# Global variable for the RealSense profile
profile = None


# Function to clear RC overrides
def override_rc_channels(ch1, ch2, ch3, ch4):
    """
    Override RC channels using pymavlink.
    ch1, ch2, ch3, ch4: Channel values (1000 to 2000)
    """
    mavlink_connection.mav.rc_channels_override_send(
        mavlink_connection.target_system,  # target_system
        mavlink_connection.target_component,  # target_component
        ch1, ch2, ch3, ch4, 0, 0, 0, 0)  # channels 1-8 (set channels 5-8 to 0)


# Initialize RealSense pipeline and profile
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
    sensor.set_option(rs.option.gain, 85.0)

    depth_sensor = profile.get_device().query_sensors()[0]
    # depth_sensor.set_option(rs.option.visual_preset,4) #high density preset, medium density is 5. doesn't work rn, maybe because of no advanced mode on pi?
    return pipeline, profile

def apply_filters(depth_frame):
    decimation = rs.decimation_filter()
    spatial = rs.spatial_filter()
    spatial.set_option(rs.option.holes_fill,5) #do I still need hole filling???
    hole_filling = rs.hole_filling_filter(2) #use min of neighbour cells,might need changing
    threshold_filter = rs.threshold_filter(0.3, 16)
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

def get_new_images():
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    if not depth_frame or not color_frame:
        print("problems")

    color_image = np.asanyarray(color_frame.get_data())
    depth_frame = apply_filters(depth_frame)
    depth_image = np.asanyarray(depth_frame.get_data()) * depth_scale

    # num_zeros = np.count_nonzero(depth_image == 0)
    # print(f"Number of zero values in depth image:{num_zeros}")
    return depth_image,color_image

def is_deadend(depth_image,direction_column):
    square_height = depth_image.shape[0] // 10
    square_width = column_width
    start_row = (depth_image.shape[0] - square_height) // 2
    start_col = direction_column - (square_width // 2)

    square = depth_image[start_row:start_row + square_height, start_col:start_col + square_width]
    # Create a masked array where 0 values are masked
    masked_array = np.ma.masked_where(square == 0, square) #this might be bad, it also excludes points that are closer than minz

    # Find the minimum value while excluding masked values (0s)
    min_value_without_zeros = np.min(masked_array)
    if min_value_without_zeros < obstacle_threshold:
        return True
    else:
        return False

def deadend_protocol():
    mavlink_velocity(0, 0, 0)
    time.sleep(1)
    mavlink_turn(0, 0, 0, 45)
    time.sleep(1)
    depth_image,color_image = get_new_images()
    new_column, new_angle = find_clear_path_and_calculate_direction(depth_image, rover_width)
    if not is_deadend(depth_image, new_column):
        movement_commands(new_angle)
    else:
        mavlink_turn(0, 0, 0, 270)
        time.sleep(1)
        depth_image, color_image = get_new_images()
        new_column, new_angle = find_clear_path_and_calculate_direction(depth_image, rover_width)
        if not is_deadend(depth_image,new_column):
            movement_commands(new_angle)
        else:
            print("Alp stuff")
#             other stuff

def distance_to_obstacle(depth_image):
    # Calculate the size of the central square
    central_width = depth_image.shape[1] // 3
    central_height = depth_image.shape[0] // 10

    # Calculate the starting indices for the central square
    start_row = (depth_image.shape[0] - central_height) // 2
    start_col = (depth_image.shape[1] - central_width) // 2

    # Select the central square
    central_square = depth_image[start_row:start_row + central_height, start_col:start_col + central_width]

    # Create a masked array where 0 values are masked
    masked_array = np.ma.masked_where(central_square == 0, central_square)

    # Find the minimum value while excluding masked values (0s)
    min_value_without_zeros = np.min(masked_array)
    return min_value_without_zeros

def calculate_distance(depth_image,y1,x1,y2,x2):
    # udist = depth_frame.get_distance(x1, y1)
    # vdist = depth_frame.get_distance(x2, y2)
    udist = depth_image[y1,x1]
    vdist = depth_image[y2,x2]

    point1 = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [y1, x1], udist)
    point2 = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [y2, x2], vdist)

    # euclidean distance between two points, measured in meters
    dist = math.sqrt(
        math.pow(point1[0] - point2[0], 2) + math.pow(point1[1] - point2[1], 2) + math.pow(
            point1[2] - point2[2], 2))
    # result[0]: right, result[1]: down, result[2]: forward
    return dist

def clearest_path(depth_image):
    # Calculate the rolling mean for consecutive columns
    rolling_means = np.convolve(depth_image.mean(axis=0), np.ones(column_width) / column_width, mode='valid')

    # Find the starting index of the columns with the highest mean
    start_index = np.argmax(rolling_means)

    # Calculate the middle index of the selected columns
    middle_index = start_index + column_width // 2

    return middle_index

# Function to find a clear path and calculate its direction
def find_clear_path_and_calculate_direction(depth_image, rover_width):

    # column_means = np.mean(depth_image_meters, axis=0)
    # index_of_highest_mean = np.argmax(column_means)
    index_of_highest_mean = clearest_path(depth_image)
    angle = index_of_highest_mean / depth_image.shape[1] * 87 - (87 / 2)
    angle =(angle+360) % 360
    return index_of_highest_mean, angle

def detect_tall_vegetation(depth_image):
    """
    Detect tall vegetation in the path.
    This is a placeholder function; you need to replace it with actual logic based on your sensor setup.
    """
    # Placeholder: Assume we detect vegetation if the mean depth in the central area is less than a threshold
    central_area = depth_image[:, depth_image.shape[1] // 2]
    mean_depth = np.mean(central_area)
    if mean_depth < vegetation_threshold:  # vegetation_threshold is a predefined constant
        return True
    else:
        return False


def gap_size(depth_image, column):
    gap_threshold = 0.3
    # start_row = depth_image.shape[0] // 2
    start_row = 150
    width_left = column
    width_right = column
    while width_left > 1:
        difference = depth_image[start_row, width_left] - depth_image[start_row, width_left - 1]
        if difference > gap_threshold and depth_image[start_row, width_left - 1] < (2 * obstacle_threshold) and \
                depth_image[start_row, width_left - 1] < depth_image[start_row, column]:
            break
        else:
            width_left -= 1
    while width_right < (depth_image.shape[1] - 2):
        difference = depth_image[start_row, width_right] - depth_image[start_row, width_right + 1]
        if difference > gap_threshold and depth_image[start_row, width_right + 1] < (2 * obstacle_threshold) and \
                depth_image[start_row, width_left + 1] < depth_image[start_row, column]:
            break
        else:
            width_right += 1

    height_up = start_row
    while height_up > 1:
        difference = depth_image[height_up, column] - depth_image[height_up - 1, column]
        if difference > gap_threshold and depth_image[height_up - 1, column] < (2 * obstacle_threshold) and depth_image[
            height_up - 1, column] < depth_image[start_row, column]:
            break
        else:
            height_up -= 1

    gap_width = calculate_distance(depth_image, start_row, width_left, start_row, width_right)
    gap_height = calculate_distance(depth_image, start_row, column, height_up, column)
    print(f"GAP: height from camera:{gap_height} width:{gap_width}")
    # print(f"{width_left} {width_right}")
    return gap_height, gap_width


def movement_commands(angle):
    print(angle)
    mavlink_turn(0, 0, 0, angle)
    print("turning")
    time.sleep(1)
    mavlink_velocity(0.7, 0, 0)
    print("going forward")
    # time.sleep(1) the rover should only go forward blindly until the next image is processed

# Function to navigate while avoiding obstacles
def navigate_avoiding_obstacles(depth_image,color_image):
    print(vehicle.mode.name)
    deadend_status = False
    if vehicle.mode.name == "AUTO" or vehicle.mode.name == "GUIDED":
        vehicle.mode = VehicleMode("GUIDED")
        column_index, angle = find_clear_path_and_calculate_direction(depth_image, rover_width)

        if is_deadend(depth_image,column_index):
            print("deadend")
            deadend_status = True

        current_time = datetime.now().strftime("%H-%M-%S")
        start_time = time.time()
        slope_grid = geo.get_slope_grid(depth_image, depth_intrinsics)
        print("Creating slope grid: --- %s seconds ---" % (time.time() - start_time))

        gap_height,gap_width = gap_size(depth_image, column_index)
        save_data_to_txt(slope_grid, distance, angle, deadend_status, gap_height,gap_width, current_time)
        save_rgb_image(color_image, current_time)

        if deadend_status:
            deadend_protocol()
        else:
            movement_commands(angle)
        return angle
    return 0


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
    print("Depth Scale is: ", depth_scale)
    vehicle.armed = True

    frames = pipeline.wait_for_frames()
    prof = frames.get_profile()
    depth_intrinsics = prof.as_video_stream_profile().get_intrinsics()
    while True:
        # deadend_status = False
        depth_image,color_image = get_new_images()

        distance = distance_to_obstacle(depth_image)
        if distance < obstacle_threshold:
            print("Obstacle detected! Taking evasive action.")

        chosen_angle = navigate_avoiding_obstacles(depth_image,color_image)


except KeyboardInterrupt:
    print("Script terminated by user")

finally:
    # clear_rc_overrides()
    pipeline.stop()
    cv2.destroyAllWindows()
    vehicle.close()
    mavlink_connection.close()
    print("Connection closed.")
