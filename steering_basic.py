import math
import time
import numpy as np
import cv2
import pyrealsense2 as rs
import json
from pymavlink import mavutil
# from dronekit import connect, VehicleMode, LocationGlobalRelative
from dronekit import *
import mav_listener
# Connect to the vehicle
mavlink_connection = mavutil.mavlink_connection('/dev/serial0', baud=57600)
mavlink_connection.wait_heartbeat()
print("Heartbeat from MAVLink system (system %u component %u)" % (
mavlink_connection.target_system, mavlink_connection.target_component))
vehicle = connect('/dev/serial0', wait_ready=False, baud=57600)

# Threshold for what we consider an obstacle (in meters)
obstacle_threshold = 1.0

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

def send_ned_yaw_pymavlink(velocity_x, velocity_y, velocity_z, yaw, duration):
    """
    Move vehicle in direction based on specified velocity vectors using pymavlink.
    """
    for _ in range(duration):
        mavlink_connection.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (not used)
            mavlink_connection.target_system,  # target system
            mavlink_connection.target_component,  # target component
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
            0b0000111111000111,  # type_mask (only speeds enabled)
            0, 0, 0,  # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
            0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            math.radians(yaw), 0)  # yaw, yaw_rate
        time.sleep(1)

def send_ned_yaw_pymavlink_once(velocity_x, velocity_y, velocity_z, yaw):
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

def send_ned_pymavlink(velocity_x, velocity_y, velocity_z):
    mavlink_connection.mav.set_position_target_local_ned_send(
        0,  # time_boot_ms (not used)
        mavlink_connection.target_system,  # target system
        mavlink_connection.target_component,  # target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
        0b110111100111 ,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)  # yaw, yaw_rate

def spin_rover(duration, left_speed, right_speed):
    
    start_time = time.time()
    while time.time() - start_time < duration:
        left_speed = 1200
        right_speed = 1500
        override_rc_channels(left_speed,right_speed,0,0)
        time.sleep(0.1)

    # Stop the rover after spinning
    override_rc_channels(1500, 1500, 0, 0)

def wait_for_turn_completion(yaw_angle, turn_rate=30):
    """
    Wait for the rover to complete its turn.

    Parameters:
    yaw_angle (float): The yaw angle in degrees.
    turn_rate (float): Estimated turn rate in degrees per second.
    """
    turn_time = abs(yaw_angle / turn_rate)
    send_ned_yaw_pymavlink(0, 1, 0, 90, turn_time)
    

def turn_rover(yaw_angle, relative=True):
    """
    Turn the rover by a specified angle.

    Parameters:
    yaw_angle (float): The yaw angle in degrees. Positive values turn right, negative values turn left.
    relative (bool): If True, the turn is relative to the current heading.
    """
    set_yaw_angle(yaw_angle, relative)
    wait_for_turn_completion(yaw_angle)



def set_yaw_angle(yaw_angle, relative=False):
    """
    Set the yaw angle of the vehicle.

    Parameters:
    yaw_angle (float): The yaw angle in degrees.
    relative (bool): Set to True if the provided yaw angle is relative to the current heading.
    """
    if relative:
        is_relative = 1  # yaw relative to direction of travel
    else:
        is_relative = 0  # yaw is an absolute angle

    # Convert yaw angle to a valid range
    yaw_angle = yaw_angle % 360

    # Send COMMAND_LONG to set the yaw angle
    mavlink_connection.mav.command_long_send(
        mavlink_connection.target_system,  # target_system
        mavlink_connection.target_component,  # target_component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
        0,  # confirmation
        yaw_angle,  # param 1: yaw angle in degrees
        0,  # param 2: yaw speed (not used)
        1,  # param 3: direction (-1: CCW, 1: CW)
        is_relative,  # param 4: relative or absolute
        0, 0, 0  # params 5-7 (not used)
    )

# Function to be called whenever HEARTBEAT messages are received
def heartbeat_listener(self, name, message):
    print("Heartbeat received")
    print("Base Mode: {}".format(message.base_mode))
    print("Custom Mode: {}".format(message.custom_mode))

# Add the listener for the heartbeat message
#vehicle.add_message_listener('HEARTBEAT', heartbeat_listener)
# Global variable for the RealSense profile
profile = None


# Function to override RC channels
# def override_rc(channels):
#     channel_values = [0] * 8  # there are eight RC channels on most systems
#     for channel, value in channels.items():
#         channel_values[channel - 1] = value  # channels are 1-indexed in MAVLink
#     vehicle.channels.overrides = channel_values


#Create a message listener for all messages.
# @vehicle.on_message('*')
def listener(self, name, message):
    print ('message: %s' % message)


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

    jsonObj = json.load(open("camera_settings.json"))
    json_string = str(jsonObj).replace("'", '\"')
    dev = profile.get_device()
    # advnc_mode = rs.rs400_advanced_mode(dev)
    # advnc_mode.load_json(json_string)
    return pipeline, profile


# Specify the width of the rover in meters
rover_width = 0.5  # Adjust to your rover's width

def obstacle_ahead(depth_image, depth_scale):
    depth_image= depth_image * depth_scale

    # Calculate the size of the central square
    central_width = depth_image.shape[1] // 3
    central_height = depth_image.shape[0] // 4

    # Calculate the starting indices for the central square
    start_row = (depth_image.shape[0] - central_height) // 2
    start_col = (depth_image.shape[1] - central_width) // 2

    # Select the central square
    central_square = depth_image[start_row:start_row + central_height, start_col:start_col + central_width]

    # Create a masked array where 0 values are masked
    masked_array = np.ma.masked_where(central_square == 0, central_square)

    # Find the minimum value while excluding masked values (0s)
    min_value_without_zeros = np.min(masked_array)
    if min_value_without_zeros<obstacle_threshold:
        return True
    else:
        return False

# Function to find a clear path and calculate its direction
def find_clear_path_and_calculate_direction(depth_image, depth_scale, rover_width):
    # Convert depth image to meters
    depth_image_meters = depth_image * depth_scale

    # Threshold for what we consider an obstacle (in meters)
    column_means = np.mean(depth_image_meters, axis=0)
    # Find the index of the column with the highest mean
    index_of_highest_mean = np.argmax(column_means)
    angle = index_of_highest_mean / len(column_means) * 87 - (87 / 2)
    if angle>=0:
        yaw_angle = math.radians(angle)
    else:
        yaw_angle = math.radians(angle+360)
        
    # if angle>=0:
        # yaw_angle = angle
    # else:
        # yaw_angle = 360 + angle
    return yaw_angle

vegetation_threshold = 0.017
def detect_tall_vegetation(depth_image, depth_scale, vegetation_height_threshold):
    """
    Detect tall vegetation in the path using a RealSense depth image.

    Parameters:
    depth_image (ndarray): The depth image from the RealSense camera.
    depth_scale (float): The scale to convert depth units to meters.
    vegetation_height_threshold (float): The minimum height (in meters) to consider as vegetation.

    Returns:
    bool: True if tall vegetation is detected, False otherwise.
    """
    vegetation_height_threshold = 0.2 
    # Convert depth image to meters
    depth_in_meters = depth_image * depth_scale

    # Define the region of interest (ROI) in the image
    # For example, you might want to focus on the lower part of the image
    roi_start_row = int(depth_in_meters.shape[0] * 0.5)  # Starting from the middle row
    roi_end_row = depth_in_meters.shape[0]  # To the end of the image
    roi = depth_in_meters[roi_start_row:roi_end_row, :]

    # Find areas in the ROI where the depth suddenly changes
    # This can be done by calculating the gradient and looking for large changes
    depth_gradient = np.abs(np.gradient(roi, axis=0))
    vegetation_mask = depth_gradient > vegetation_height_threshold

    # Detect vegetation if there are significant changes in depth
    vegetation_detected = np.any(vegetation_mask)
    print("detected")
    return vegetation_detected

def move_back(steps):
    """
    Move the rover back by a certain number of steps.
    """
    for _ in range(steps):
        send_ned_velocity(-1, 0, 0, 1)
            
# Function to navigate while avoiding obstacles
def navigate_avoiding_obstacles(depth_scale):
    obstacle_threshold = 1.0
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    if not depth_frame:
        return

    depth_image = np.asanyarray(depth_frame.get_data())
    print(vehicle.mode.name)
    if vehicle.mode.name == "AUTO" or vehicle.mode.name == "GUIDED":
        if detect_tall_vegetation(depth_image, depth_scale):
            print("Tall vegetation detected. Moving back.")
            vehicle.mode = VehicleMode("GUIDED")
            move_back(2)  # Move back 2 steps
        else:
            if obstacle_ahead(depth_image,depth_scale):
                vehicle.mode = VehicleMode("GUIDED")
                clear_path_direction = find_clear_path_and_calculate_direction(depth_image, depth_scale, rover_width)
            #print(clear_path_direction)
         
                print("obstacle ahead")
#                 current_heading = vehicle.heading

#         # Calculate new heading: turn left by 90 degrees
#                 new_heading = (current_heading + 90) % 360
#                 # Example usage
#                 # Example usage: Spin the rover for 5 seconds
# # Assuming 1000 is full reverse, 1500 is stop, and 2000 is full forward.
#                 # spin_rover(5, 1000, 2000) 

#         # Set the new yaw angle
#                 set_yaw_angle(new_heading, relative=False)

#         # Move with the specified NED velocity while turning
#                 send_ned_yaw_pymavlink(0, 1, 0, new_heading, 5)
            # send_ned_velocity(1,0,0,5)
            else:
                print("no obstacle ahead")
                vehicle.mode = VehicleMode("AUTO")
                override_rc_channels(1600, 1500, 0, 0)  # Override throttle and steering channels
import time

def detect_collision(mavlink_connection):
    """
    Detects collision by comparing rover speed from MAVLink and optical flow data.
    """
    start_time = time.time()
    collision_detected = False

    while time.time() - start_time < 5:
        # Retrieve speed data from MAVLink
        current_speed = mav_listener.get_rover_speed(mavlink_connection)  # Speed in cm/s
        print(current_speed)
        # Retrieve optical flow data from MAVLink
        optical_flow_data = get_optical_flow_data_from_mavlink()
        print(optical_flow_data)
        # Check for near-zero optical flow and non-zero speed
        if is_near_zero(optical_flow_data) and current_speed > 0:
            collision_detected = True
            break
        time.sleep(0.1)  # Adjust time interval as needed

    if collision_detected:
        print("Collision detected. Reversing and finding another path.")
        # Implement actions after detecting collision, e.g., move_back, find_alternate_path, etc.

def is_near_zero(data, threshold=0.1):
    """
    Check if data is near zero within a given threshold.
    """
    x, y = data
    return abs(x) < threshold and abs(y) < threshold

def get_optical_flow_data_from_mavlink():
    """
    Retrieve optical flow data from MAVLink.
    """
    # Fetch optical flow data from a relevant MAVLink message
    # Example: Use flow_x and flow_y for optical flow data
    optical_flow_msg = mavlink_connection.recv_match(type='OPTICAL_FLOW', blocking=True)
    if optical_flow_msg:
        return optical_flow_msg.flow_x, optical_flow_msg.flow_y
    else:
        return 0, 0

# Implement move_back() and find_alternate_path() as needed






# Main execution loop
try:
    pipeline, profile = initialize_realsense()
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print("Depth Scale is: ", depth_scale)
    detect_collision(mavlink_connection)
    vehicle.armed = True
    # turn_rover(90, relative=True)
    print(vehicle.mode)
# After turning, stop any further movement
#     send_ned_yaw_pymavlink(0, 0, 0, 0, 1)
    send_ned_yaw_pymavlink_once(0,0,0,45)
    # while True:
    #     navigate_avoiding_obstacles(depth_scale)
    #     time.sleep(1)
except KeyboardInterrupt:
    print("Script terminated by user")

finally:
    # clear_rc_overrides()
    pipeline.stop()
    cv2.destroyAllWindows()
    vehicle.close()
    mavlink_connection.close()
    print("Connection closed.")
