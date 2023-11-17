
from dronekit import connect, VehicleMode, mavutil
import pyrealsense2 as rs
import time
import numpy as np
import cv2

# Connect to the vehicle
vehicle = connect('/dev/serial0', wait_ready=True, baud=57600)

# Function to override RC channels
def override_rc(channels):
    
    channel_values = [0]*8  # there are eight RC channels on most systems
    for channel, value in channels.items():
        channel_values[channel-1] = value  # channels are 1-indexed in MAVLink
    vehicle.channels.overrides = channel_values

# Function to clear RC overrides
def clear_rc_overrides():
    
    vehicle.channels.overrides = {}

# Initialize the RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)  # Depth stream
pipeline.start(config)

# Function to check for obstacles and avoid them
def check_and_avoid_obstacles():
    # Wait for the next set of frames from the camera
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    if not depth_frame:
        return

    # Convert the depth frame to a numpy array
    depth_image = np.asanyarray(depth_frame.get_data())
    # Assuming the obstacle is in front of the camera, get the depth value in the center of the image
    obstacle_distance = depth_image[depth_image.shape[0] // 2, depth_image.shape[1] // 2]

    # Define a threshold distance in meters (1 meter in this case)
    threshold_distance_m = 1.0

    # If there's an obstacle within the threshold distance, avoid it
    if obstacle_distance < threshold_distance_m * 1000:  # Convert meters to the scale of depth values
        if vehicle.mode.name != "GUIDED":
            print("Obstacle detected! Switching to GUIDED mode and overriding RC.")
            vehicle.mode = VehicleMode("GUIDED")
            time.sleep(1)  # Give some time to switch to GUIDED mode
            override_rc({5: 1680})  
        else:
            print("Already in GUIDED mode, overriding RC.")
            override_rc({5: 1680})  # Continue overriding RC commands
    else:
        if vehicle.mode.name == "GUIDED":
            print("No obstacles detected. Returning to AUTO mode and clearing RC overrides.")
            vehicle.mode = VehicleMode("AUTO")
            clear_rc_overrides()
        elif vehicle.mode.name == "AUTO":
            # If in AUTO mode and no overrides are active, keep checking for obstacles
            if vehicle.channels.overrides:
                clear_rc_overrides()

try:
    while True:
        check_and_avoid_obstacles()
        time.sleep(0.1)

except KeyboardInterrupt:
    print("Script terminated by user")

finally:
    # Clean up
    clear_rc_overrides()
    pipeline.stop()
    cv2.destroyAllWindows()
    vehicle.close()
    print("Connection closed.")
