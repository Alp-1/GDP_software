import os
from datetime import datetime

import cv2
import numpy as np
import pyrealsense2 as rs
# from dronekit import connect, VehicleMode, LocationGlobalRelative
from dronekit import *
from scipy.spatial.transform import Rotation

import geometric_map as geo
import mav_listener
from logging_config import setup_custom_logger
import camera_angle as cam
from semantic_map import SemanticSegmentation
import mav_sender

def mavlink_go_back0(velocity_x, velocity_y, velocity_z):
    global target_speed
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


def mavlink_go_back1(thrust):
    mavlink_connection.mav.set_attitude_target(
        0,  # time_boot_ms (not used)
        mavlink_connection.target_system,  # target system
        mavlink_connection.target_component,  # target component
        0b00100111,  # type_mask
        [1,0, 0, 0],  # x, y, z positions (not used)
        0, 0, 0,  # x, y, z velocity in m/s
        thrust) #thrust


def euler_to_quaternion(roll, pitch, yaw):
    # Convert degrees to radians
    roll_rad = roll * (3.141592653589793 / 180.0)
    pitch_rad = pitch * (3.141592653589793 / 180.0)
    yaw_rad = yaw * (3.141592653589793 / 180.0)

    # Create a Rotation object from Euler angles
    r = Rotation.from_euler('xyz', [roll_rad, pitch_rad, yaw_rad], degrees=False)

    # Get the quaternion as a numpy array
    quaternion = r.as_quat()

    return quaternion

def mavlink_go_back2(thrust):
    heading = mav_listener.get_heading(mavlink_connection)
    quater = euler_to_quaternion(0,0,heading)
    mavlink_connection.mav.set_attitude_target(
        0,  # time_boot_ms (not used)
        mavlink_connection.target_system,  # target system
        mavlink_connection.target_component,  # target component
        0b00100111,  # type_mask
        quater,  # x, y, z positions (not used)
        0, 0, 0,  # x, y, z velocity in m/s
        thrust)  # thrust

def mavlink_velocity(velocity_x, velocity_y, velocity_z):
    global target_speed
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

    logger.info("sexy")
    target_speed = velocity_x


# Connect to the vehicle
mavlink_connection = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
mavlink_connection.wait_heartbeat()

try:
    mavlink_connection.arducopter_arm()
    time.sleep(3)
    mavlink_go_back0(-1,0,0)
    # mavlink_go_back1(-1)
    time.sleep(1)
    #mavlink_velocity(0,0,0)

except KeyboardInterrupt:
    print("Script terminated by user")
