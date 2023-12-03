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

#try:
vehicle = connect('/dev/serial0', wait_ready=True, baud=57600)

print("Base mode: ", vehicle.mode.name)
#print("Custom mode: ", vehicle._master.mavlink_version)

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors and
    for the specified duration.

    This uses the SET_POSITION_TARGET_LOCAL_NED command with a type mask enabling only 
    velocity components 
    (http://dev.ardupilot.com/wiki/copter-commands-in-guided-mode/#set_position_target_local_ned).
    
    Note that from AC3.3 the message should be re-sent every second (after about 3 seconds
    with no message the velocity will drop back to zero). In AC3.2.1 and earlier the specified
    velocity persists until it is canceled. The code below should work on either version 
    (sending the message multiple times does not cause problems).
    
    See the above link for information on the type_mask (0=enable, 1=ignore). 
    At time of writing, acceleration and yaw bits are ignored.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED, # frame
        0b110111100111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

def send_global_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, # lat_int - X Position in WGS84 frame in 1e7 * meters
        0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        velocity_x, # X velocity in NED frame in m/s
        velocity_y, # Y velocity in NED frame in m/s
        velocity_z, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)


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
def clear_rc_overrides():
    vehicle.channels.overrides = {}


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
    advnc_mode = rs.rs400_advanced_mode(dev)
    advnc_mode.load_json(json_string)
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


# Function to find a clear path and calculate its direction
def find_clear_path_and_calculate_direction(depth_image, depth_scale, rover_width):
    # Convert depth image to meters
    depth_image_meters = depth_image * depth_scale

    # Threshold for what we consider an obstacle (in meters)
    obstacle_threshold = 1.0  # e.g., 1 meter

    # Height and width of the depth image
    height, width = depth_image_meters.shape
    column_means = np.mean(depth_image, axis=0)
    # Find the index of the column with the highest mean
    index_of_highest_mean = np.argmax(column_means)
    angle = index_of_highest_mean / len(column_means) * 87 - (87 / 2)
    print("angle:", index_of_highest_mean / len(column_means) * 87 - (87 / 2))
    if angle>=0:
        yaw_angle = math.radians(angle)
    else:
        yaw_angle = math.radians(angle+180)
    return yaw_angle


# Function to navigate while avoiding obstacles
def navigate_avoiding_obstacles(depth_scale):
    obstacle_threshold = 1.0
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    if not depth_frame:
        return

    depth_image = np.asanyarray(depth_frame.get_data())
    cv2.imshow("",depth_image)
    clear_path_direction = find_clear_path_and_calculate_direction(depth_image, depth_scale, rover_width)
    # print(clear_path_direction)
    # if vehicle.mode.name == "AUTO":
    if True:
        clear_path_direction = find_clear_path_and_calculate_direction(depth_image, depth_scale, rover_width)
        if clear_path_direction is not None:
            print("Clear path found. Setting heading and moving forward.")
            vehicle.mode = VehicleMode("GUIDED")
            # print(vehicle.mode.name)
            a_location = LocationGlobalRelative(-34.364114, 149.166022, 30)
            vehicle.simple_goto(a_location)
            # override_rc({5: 1680})
            # Set the heading of the rover
            # set_position_target_local_ned(
            #     x=0, y=0, z=0,
            #     vx=0, vy=0, vz=0,
            #     yaw=clear_path_direction,
            #     coordinate_frame=mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            #     type_mask=0b111111111000
            # )
            #
            # # Move forward
            # set_position_target_local_ned(
            #     x=0, y=0, z=0,
            #     vx=1, vy=0, vz=0,  # Adjust speed as needed
            #     yaw=clear_path_direction,
            #     coordinate_frame=mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            #     type_mask=0b110111000111
            # )
            # Set the heading of the rover
            # msg = vehicle.message_factory.set_position_target_local_ned_encode(
                # 0,  # time_boot_ms (not used)
                # 0, 0,  # target_system, target_component
                # mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
                # 0b100111111111 ,  # type_mask (only speeds enabled)
                # 0, 0, 0,  # x, y, z positions
                # 0, 0, 0,  # x, y, z velocity in m/s
                # 0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
                # clear_path_direction, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
            # # send command to vehicle
            # vehicle.send_mavlink(msg)

            # msg = vehicle.message_factory.set_position_target_local_ned_encode(
                # 0,  # time_boot_ms (not used)
                # 0, 0,  # target_system, target_component
                # mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
                # 0b0000111111000111,  # type_mask (only speeds enabled)
                # 0, 0, 0,  # x, y, z positions
                # 1, 0, 0,  # x, y, z velocity in m/s
                # 0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
                # 0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
            # # send command to vehicle
            # vehicle.send_mavlink(msg)
            
            
            msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target_system, target_component
            mavutil.mavlink.MAV_FRAME_BODY_NED, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, 0, 0, # x, y, z positions
            1, 0, 0, # x, y, z velocity in m/s
            0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
            # send command to vehicle
            vehicle.send_mavlink(msg)



# Main execution loop
try:
    # Get some vehicle attributes (state)
    # print ("Get some vehicle attribute values:")
    print (" GPS: %s" % vehicle.gps_0)
    # print (" Battery: %s" % vehicle.battery)
    # print (" Last Heartbeat: %s" % vehicle.last_heartbeat)
    # print (" Is Armable?: %s" % vehicle.is_armable)
    # print (" System status: %s" % vehicle.system_status.state)
    print (" Mode: %s" % vehicle.mode.name)  # settable

    pipeline, profile = initialize_realsense()
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print("Depth Scale is: ", depth_scale)
    vehicle.armed = True
    send_ned_velocity(1,0,0,5)
    while True:
        navigate_avoiding_obstacles(depth_scale)
        # time.sleep(0.1)
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
