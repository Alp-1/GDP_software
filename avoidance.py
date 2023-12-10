import cv2
import numpy as np
import pyrealsense2 as rs
# from dronekit import connect, VehicleMode, LocationGlobalRelative
from dronekit import *

# Connect to the vehicle
mavlink_connection = mavutil.mavlink_connection('/dev/serial0', baud=57600)
mavlink_connection.wait_heartbeat()
print("Heartbeat from MAVLink system (system %u component %u)" % (
    mavlink_connection.target_system, mavlink_connection.target_component))
vehicle = connect('/dev/serial0', wait_ready=False, baud=57600)

obstacle_threshold = 2.0
column_width = 20

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

def mavlink_turn_and_go(velocity_x, velocity_y, velocity_z, yaw):
    mavlink_connection.mav.set_position_target_local_ned_send(
        0,  # time_boot_ms (not used)
        mavlink_connection.target_system,  # target system
        mavlink_connection.target_component,  # target component
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
        0b100111100111 ,  # type_mask (only speeds enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        math.radians(yaw), 0)  # yaw, yaw_rate

# Function to be called whenever HEARTBEAT messages are received
def heartbeat_listener(self, name, message):
    print("Heartbeat received")
    print("Base Mode: {}".format(message.base_mode))
    print("Custom Mode: {}".format(message.custom_mode))


# Add the listener for the heartbeat message
# vehicle.add_message_listener('HEARTBEAT', heartbeat_listener)
# Global variable for the RealSense profile
profile = None


# Function to override RC channels
# def override_rc(channels):
#     channel_values = [0] * 8  # there are eight RC channels on most systems
#     for channel, value in channels.items():
#         channel_values[channel - 1] = value  # channels are 1-indexed in MAVLink
#     vehicle.channels.overrides = channel_values


# Create a message listener for all messages.
# @vehicle.on_message('*')
def listener(self, name, message):
    print('message: %s' % message)


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
    threshold_filter = rs.threshold_filter()
    threshold_filter.set_option(rs.option.max_distance, 10)
    return pipeline, profile


# Specify the width of the rover in meters
rover_width = 0.5  # Adjust to your rover's width


def obstacle_ahead(depth_image, depth_scale):
    depth_image_meters = depth_image * depth_scale
    # Calculate the mean of the middle column
    middle_column_mean = np.mean(depth_image_meters[:, depth_image_meters.shape[1] // 2])
    print(middle_column_mean)
    if middle_column_mean < obstacle_threshold:
        return True
    else:
        return False

def clearest_path(depth_image):
    # Calculate the rolling mean for consecutive columns
    rolling_means = np.convolve(depth_image.mean(axis=0), np.ones(column_width) / column_width, mode='valid')

    # Find the starting index of the columns with the highest mean
    start_index = np.argmax(rolling_means)

    # Calculate the middle index of the selected columns
    middle_index = start_index + column_width // 2

    return middle_index

# Function to find a clear path and calculate its direction
def find_clear_path_and_calculate_direction(depth_image, depth_scale, rover_width):
    # Convert depth image to meters
    depth_image_meters = depth_image * depth_scale

    # Threshold for what we consider an obstacle (in meters)

    # column_means = np.mean(depth_image_meters, axis=0)
    # index_of_highest_mean = np.argmax(column_means)
    index_of_highest_mean = clearest_path(depth_image_meters)
    angle = index_of_highest_mean / len(depth_image.shape[1]) * 87 - (87 / 2)
    angle =(angle+360) % 360
    return angle


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
    # for _ in range(steps):
    #     send_ned_velocity(-1, 0, 0, 1)


# Function to navigate while avoiding obstacles
def navigate_avoiding_obstacles(depth_scale):
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    if not depth_frame:
        return

    depth_image = np.asanyarray(depth_frame.get_data())
    print(depth_image.shape[1])
    print(vehicle.mode.name)
    if vehicle.mode.name == "AUTO" or vehicle.mode.name == "GUIDED":
        vehicle.mode = VehicleMode("GUIDED")
        angle = find_clear_path_and_calculate_direction(depth_image, depth_scale, rover_width)
        print(angle)
        mavlink_turn(0,0,0,angle)
        print("turning")
        #mavlink_connection.wait_heartbeat()
        # while True:
            # ack_msg = mavlink_connection.recv_match(type='COMMAND_ACK',blocking=False)
            # print(ack_msg)
        time.sleep(2)
        print("going forward")
        mavlink_velocity(0.1,0,0)

        mavlink_turn_and_go(0.1,0,0,angle)
        time.sleep(1)
# Main execution loop
try:
    pipeline, profile = initialize_realsense()
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print("Depth Scale is: ", depth_scale)
    vehicle.armed = True
    while True:
        print("looking for path")
        navigate_avoiding_obstacles(depth_scale)
        #time.sleep(1)

except KeyboardInterrupt:
    print("Script terminated by user")

finally:
    # clear_rc_overrides()
    pipeline.stop()
    cv2.destroyAllWindows()
    vehicle.close()
    mavlink_connection.close()
    print("Connection closed.")
