import sys

import cv2
import pyrealsense2 as rs
import os
import numpy as np
import time
import json
np.set_printoptions(threshold=sys.maxsize)
# Create a 'data' directory if it doesn't exist
#data_dir = '/media/gdp/SanDisk/new_data'
data_dir = 'data'
os.makedirs(data_dir, exist_ok=True)

# Initialize the RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
# config.enable_stream(rs.stream.color, 1280, 800, rs.format.bgr8, 30)  # RGB stream
config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 60)  # RGB stream
config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 60)  # Depth stream
config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)  # Accelerometer data
config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 400)  # Gyroscope data
profile = pipeline.start(config)

sensor = profile.get_device().query_sensors()[1]
sensor.set_option(rs.option.enable_auto_exposure, False)
sensor.set_option(rs.option.exposure,78.0)
sensor.set_option(rs.option.gain,90.0)

jsonObj = json.load(open("camera_settings.json"))
json_string= str(jsonObj).replace("'", '\"')
dev = profile.get_device()
# advnc_mode = rs.rs400_advanced_mode(dev)
# advnc_mode.load_json(json_string)

# Set the save frequency
save_interval = 1.5  # in seconds
last_save_time = 0
# Getting the depth sensor's depth scale
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
count = 0
try:
    while True:
        time.sleep(3)
        print(sensor.get_option(rs.option.exposure))
        print(sensor.get_option(rs.option.gain))
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        # get imu frames
        accel_frame = frames.first_or_default(rs.stream.accel, rs.format.motion_xyz32f)
        gyro_frame = frames.first_or_default(rs.stream.gyro, rs.format.motion_xyz32f)

        # Check if all frames are available
        if not depth_frame or not color_frame or not accel_frame or not gyro_frame:
            continue

        current_time = cv2.getTickCount() / cv2.getTickFrequency()
        if current_time - last_save_time > save_interval:
            # Save depth image
            depth_image = np.asanyarray(depth_frame.get_data())
            depth_image_meters = depth_image * depth_scale

            # # Save depth image
            # depth_filename = os.path.join(data_dir, f'depth_{current_time}.png')
            # cv2.imwrite(depth_filename, depth_image_meters)

            # Save RGB image
            color_image = np.asanyarray(color_frame.get_data())
            color_filename = os.path.join(data_dir, f'rgb_{current_time}.png')
            cv2.imwrite(color_filename, color_image)

            # # Save IMU data
            # accel_data = str(accel_frame.as_motion_frame().get_motion_data())
            # gyro_data = str(gyro_frame.as_motion_frame().get_motion_data())
            # imu_filename = os.path.join(data_dir, f'imu_{current_time}.txt')
            #
            # with open(imu_filename, 'w') as file:
            #     # Write the first string followed by a newline character
            #     file.write(accel_data + '\n')
            #
            #     # Write the second string followed by a newline character
            #     file.write(gyro_data)
            last_save_time = current_time

except KeyboardInterrupt:
    pass
finally:
    pipeline.stop()
