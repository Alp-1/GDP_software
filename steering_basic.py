import math
import time
import numpy as np
import cv2
import pyrealsense2 as rs
import json
from pymavlink import mavutil
# from dronekit import connect, VehicleMode, LocationGlobalRelative
from dronekit import *

# Connect to the vehicle
mavlink_connection = mavutil.mavlink_connection('/dev/serial0', baud=57600)
mavlink_connection.wait_heartbeat()
print("Heartbeat from MAVLink system (system %u component %u)" % (
mavlink_connection.target_system, mavlink_connection.target_component))
vehicle = connect('/dev/serial0', wait_ready=False, baud=57600)

obstacle_threshold = 4.0

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
def spin_around_own_axis(yaw_rate, duration):
    """
    Spin the rover around its own axis.

    Parameters:
    yaw_rate (float): The yaw rate in degrees per second.
    duration (int): Duration to spin in seconds.
    """
    for _ in range(duration):
        mavlink_connection.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (not used)
            mavlink_connection.target_system,  # target system
            mavlink_connection.target_component,  # target component
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
            0b0000111111111111,  # type_mask (ignore all except yaw rate)
            0, 0, 0,  # x, y, z positions (not used)
            0, 0, 0,  # x, y, z velocity (not used)
            0, 0, 0,  # x, y, z acceleration (not used)
            0, math.radians(yaw_rate))  # yaw, yaw_rate
        time.sleep(1)

 # Example: Move forward with a yaw angle of 45 degrees for 5 seconds

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


# Function to send SET_POSITION_TARGET_LOCAL_NED command
def set_position_target_local_ned(x, y, z, vx, vy, vz, yaw, coordinate_frame, type_mask):
    mavlink_connection.mav.set_position_target_local_ned_send(
        time_boot_ms=0,
        target_system=mavlink_connection.target_system,
        target_component=mavlink_connection.target_component,
        coordinate_frame=coordinate_frame,
        type_mask=type_mask,
        x=x, y=y, z=z,
        vx=vx, vy=vy, vz=vz,
        afx=0, afy=0, afz=0,
        yaw=yaw,
        yaw_rate=0
    )


# Specify the width of the rover in meters
rover_width = 0.5  # Adjust to your rover's width

def obstacle_ahead(depth_image, depth_scale):
    depth_image_meters = depth_image * depth_scale
    # Calculate the mean of the middle column
    middle_column_mean = np.mean(depth_image_meters[:, depth_image_meters.shape[1] // 2])
    print(middle_column_mean)
    if middle_column_mean<obstacle_threshold:
        return True
    else:
        return False

# Function to find a clear path and calculate its direction
def find_clear_path_and_calculate_direction(depth_image, depth_scale, rover_width):
    # Convert depth image to meters
    depth_image_meters = depth_image * depth_scale

    # Threshold for what we consider an obstacle (in meters)
    obstacle_threshold = 1.0  # e.g., 1 meter
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
def detect_tall_vegetation(depth_image, depth_scale):
    """
    Detect tall vegetation in the path.
    This is a placeholder function; you need to replace it with actual logic based on your sensor setup.
    """
    # Placeholder: Assume we detect vegetation if the mean depth in the central area is less than a threshold
    central_area = depth_image[:, depth_image.shape[1] // 2]
    mean_depth = np.mean(central_area * depth_scale)
    if mean_depth < vegetation_threshold:  # vegetation_threshold is a predefined constant
        return True
    else:
        return False

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
            move_back(2)  # Move back 2 steps
        else:
            if obstacle_ahead(depth_image,depth_scale):
                vehicle.mode = VehicleMode("GUIDED")
                clear_path_direction = find_clear_path_and_calculate_direction(depth_image, depth_scale, rover_width)
            #print(clear_path_direction)
         
                print("obstacle ahead")
                current_heading = vehicle.heading

        # Calculate new heading: turn left by 90 degrees
                new_heading = (current_heading + 90) % 360
                # Example usage
                spin_around_own_axis(30, 5)  # Spin around at 30 degrees per second for 5 seconds

        # Set the new yaw angle
        #         set_yaw_angle(new_heading, relative=False)

        # # Move with the specified NED velocity while turning
        #         send_ned_yaw_pymavlink(0, 1, 0, new_heading, 5)
            # send_ned_velocity(1,0,0,5)
            else:
                print("no obstacle ahead")
                vehicle.mode = VehicleMode("AUTO")
                override_rc_channels(1600, 1500, 0, 0)  # Override throttle and steering channels



# Main execution loop
try:
    pipeline, profile = initialize_realsense()
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print("Depth Scale is: ", depth_scale)
    vehicle.armed = True
    while True:
        navigate_avoiding_obstacles(depth_scale)
        time.sleep(1)
except KeyboardInterrupt:
    print("Script terminated by user")

finally:
    # clear_rc_overrides()
    pipeline.stop()
    cv2.destroyAllWindows()
    vehicle.close()
    mavlink_connection.close()
    print("Connection closed.")
