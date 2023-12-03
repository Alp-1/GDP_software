import time
import numpy as np
import cv2
import pyrealsense2 as rs
from pymavlink import mavutil
from dronekit import connect, VehicleMode

# Connect to the vehicle
mavlink_connection = mavutil.mavlink_connection('/dev/serial0', baud=57600)
mavlink_connection.wait_heartbeat()
print("Heartbeat from MAVLink system (system %u component %u)" % (mavlink_connection.target_system, mavlink_connection.target_component))

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
    print("work")

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

    # Calculate the required pixel width of a clear path
    pixel_width_for_rover = int((rover_width / obstacle_threshold) * width)
    print("pixel_width_for_rover")
    print(pixel_width_for_rover)
    # Initialize variables for path detection
    max_clear_path_width = 0
    path_center_x = 0

    # Scan each column in the depth image
    # for x in range(pixel_width_for_rover // 2, width - pixel_width_for_rover // 2):
    #     column = depth_image_meters[:, x]
    #     print("Column")
    #     print(column)
    #     # Check if this column is part of a clear path
    #     if np.all(column > obstacle_threshold):
    #         # Increment the width of the clear path
    #         max_clear_path_width += 1
    #         path_center_x += x
    #         break
    #     else:
    #         # Check if the current clear path is wide enough for the rover
    #         if max_clear_path_width >= pixel_width_for_rover:
    #             break
    #         else:
    #             # Reset the path width and center
    #             max_clear_path_width = 0
    #             path_center_x = 0
    max_clear_path_width = pixel_width_for_rover
    # Check if a valid path was found
    if max_clear_path_width >= pixel_width_for_rover:
        # Calculate the center of the path
        path_center_x /= max_clear_path_width

        # Calculate the yaw angle (assuming straight ahead is 0 radians)
        yaw_angle = np.arctan2(path_center_x - (width / 2), height)
        print("yaw_angle")
        print(yaw_angle)
        return yaw_angle

    return None

# Function to navigate while avoiding obstacles
def navigate_avoiding_obstacles(depth_scale):
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    if not depth_frame:
        return

    depth_image = np.asanyarray(depth_frame.get_data())
    print(depth_image)
    if vehicle.mode.name == "GUIDED":
        clear_path_direction = find_clear_path_and_calculate_direction(depth_image, depth_scale, rover_width)
        if clear_path_direction is not None:
            print("Clear path found. Setting heading and moving forward.")
            # vehicle.mode = VehicleMode("GUIDED")
            time.sleep(1)  # Allow time for mode switch
            print("sexy")
            # Set the heading of the rover
            # set_position_target_local_ned(
            #     x=0, y=0, z=0,
            #     vx=0, vy=0, vz=0,
            #     yaw=clear_path_direction,
            #     coordinate_frame=mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            #     type_mask=0b111111111000
            # )

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
    print (" Mode: %s" % vehicle.mode.name)
    while True:
        navigate_avoiding_obstacles(depth_scale)
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Script terminated by user")

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    vehicle.close()
    mavlink_connection.close()
    print("Connection closed.")
