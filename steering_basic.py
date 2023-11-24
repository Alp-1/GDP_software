import math
import time
import numpy as np
import cv2
import pyrealsense2 as rs
from pymavlink import mavutil
from dronekit import connect, VehicleMode

# Connect to the vehicle
mavlink_connection = mavutil.mavlink_connection('/dev/serial0', baud=57600)
mavlink_connection.wait_heartbeat()
print("Heartbeat from MAVLink system (system %u component %u)" % (
mavlink_connection.target_system, mavlink_connection.target_component))

vehicle = connect('/dev/serial0', wait_ready=True, baud=57600)

# Global variable for the RealSense profile
profile = None

# Initialize RealSense pipeline and profile
def initialize_realsense():
    global profile
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    profile = pipeline.start(config)
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
    print(clear_path_direction)
    if vehicle.mode.name == "AUTO":
        clear_path_direction = find_clear_path_and_calculate_direction(depth_image, depth_scale, rover_width)
        if clear_path_direction is not None:
            print("Clear path found. Setting heading and moving forward.")
            vehicle.mode = VehicleMode("GUIDED")

            # Set the heading of the rover
            set_position_target_local_ned(
                x=0, y=0, z=0,
                vx=0, vy=0, vz=0,
                yaw=clear_path_direction,
                coordinate_frame=mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                type_mask=0b111111111000
            )

            # Move forward
            set_position_target_local_ned(
                x=0, y=0, z=0,
                vx=1, vy=0, vz=0,  # Adjust speed as needed
                yaw=clear_path_direction,
                coordinate_frame=mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                type_mask=0b110111000111
            )


# Main execution loop
try:
    pipeline, profile = initialize_realsense()
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print("Depth Scale is: ", depth_scale)

    while True:
        navigate_avoiding_obstacles(depth_scale)
        # time.sleep(0.1)
        time.sleep(1)
except KeyboardInterrupt:
    print("Script terminated by user")

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    vehicle.close()
    mavlink_connection.close()
    print("Connection closed.")
