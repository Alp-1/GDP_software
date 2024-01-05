import os
import time
from datetime import datetime

import cv2
import numpy as np
import pyrealsense2 as rs
from matplotlib import pyplot as plt

import geometric_map as geo
import mav_listener
from logging_config import setup_custom_logger
import camera_angle as cam
from semantic_map import SemanticSegmentation
logger = setup_custom_logger("navigation")
import geometric_map as geo

def apply_filters(depth_frame):
    decimation = rs.decimation_filter()
    spatial = rs.spatial_filter()
    spatial.set_option(rs.option.holes_fill,5) #do I still need hole filling???
    hole_filling = rs.hole_filling_filter(2) #use min of neighbour cells,might need changing
    threshold_filter = rs.threshold_filter(0.3, 4.0)
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
    sensor.set_option(rs.option.gain, 90.0)

    depth_sensor = profile.get_device().query_sensors()[0]
    # depth_sensor.set_option(rs.option.visual_preset,4) #high density preset, medium density is 5. doesn't work rn, maybe because of no advanced mode on pi?
    return pipeline, profile


def get_thresholded_image(depth_frame):
    distance_limit = 4.0
    decimation = rs.decimation_filter()
    spatial = rs.spatial_filter()
    spatial.set_option(rs.option.holes_fill, 5)  # do I still need hole filling???
    hole_filling = rs.hole_filling_filter(2)  # use min of neighbour cells,might need changing
    depth_to_disparity = rs.disparity_transform(True)
    disparity_to_depth = rs.disparity_transform(False)

    frame = depth_frame
    frame = decimation.process(frame)
    frame = depth_to_disparity.process(frame)
    frame = spatial.process(frame)
    frame = disparity_to_depth.process(frame)
    frame = hole_filling.process(frame)

    depth_image = np.asanyarray(frame.get_data()) * depth_scale
    depth_image[depth_image>distance_limit] = distance_limit

    return depth_image

def get_new_images(frames):
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    if not depth_frame or not color_frame:
        logger.info("problems")

    color_image = np.asanyarray(color_frame.get_data())
    steering_image = get_thresholded_image(depth_frame)
    depth_frame = apply_filters(depth_frame)
    depth_image = np.asanyarray(depth_frame.get_data()) * depth_scale

    # num_zeros = np.count_nonzero(depth_image == 0)
    # logger.info(f"Number of zero values in depth image:{num_zeros}")
    return steering_image, depth_image, color_image


def distance_to_obstacle(depth_image, slope_grid):
    pitch_threshold = 30
    danger_squares = []
    central_width = slope_grid.shape[1] // 4
    central_height = 2
    min_distance = 999

    start_row = (slope_grid.shape[0] - central_height) // 2
    start_col = (slope_grid.shape[1] - central_width) //  2
    patch_height = (depth_image.shape[0]//slope_grid.shape[0])
    patch_width = (depth_image.shape[1]//slope_grid.shape[1])
    print(patch_height,patch_width)
    for row_index in range(slope_grid.shape[0]):
        for col_index in range(3,5):
            if slope_grid[row_index][col_index] > 30:
                print(f'slope:{slope_grid[row_index][col_index]}')
                # danger_squares.append((row_index,col_index))
                print(row_index,col_index)
                patch_start_row = (row_index)*patch_height
                patch_start_column = (col_index)*patch_width
                print(patch_start_row,patch_start_column)
                patch_end_row = patch_start_row+patch_height
                patch_end_column = patch_start_column+patch_width
                patch = depth_image[patch_start_row:patch_end_row,patch_start_column:patch_end_column]
                closest_in_patch = np.min(np.ma.masked_where(patch == 0, patch))
                min_distance = min(min_distance,closest_in_patch)


    return min_distance

# Main execution loop
try:


    pipeline, profile = initialize_realsense()
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    logger.info("Depth Scale is: {}".format(depth_scale))


    frames = pipeline.wait_for_frames()
    prof = frames.get_profile()
    depth_intrinsics = prof.as_video_stream_profile().get_intrinsics()
    cam.initialize_angle(frames)
    clf = SemanticSegmentation()
    while True:
        frames = pipeline.wait_for_frames()
        steering_image, depth_image, color_image = get_new_images(frames)
        angle = cam.get_camera_angle(frames)
        # print(angle)
        # print(depth_image[depth_image.shape[0]//2][depth_image.shape[1]//2])
        # print(steering_image[steering_image.shape[0]//2][steering_image.shape[1]//2])
        # start_time = time.time()
        # print(geo.get_slope_grid(depth_image,depth_intrinsics,angle))
        # print("Slope Grid: --- %s seconds ---" % (time.time() - start_time))
        # mask = clf.get_semantic_map(color_image)
        # plt.imshow(mask)
        slope_grid = geo.get_slope_grid(depth_image,depth_intrinsics,angle)
        print(slope_grid)
        print(f'distance:{distance_to_obstacle(depth_image,slope_grid)}')
        time.sleep(3)
except KeyboardInterrupt:
    logger.info("Script terminated by user")

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
