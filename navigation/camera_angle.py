import time

from pyrealsense import pyrealsense2 as rs
import math


last_ts_gyro = 0
accel_angle_x = 0
accel_angle_y = 0
accel_angle_z = 0
first = True
alpha = 0.98


def initialize_camera():
    # start the frames pipe
    p = rs.pipeline()
    conf = rs.config()
    conf.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)  # Accelerometer data
    conf.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 400)
    prof = p.start(conf)
    return p


def initialize_angle(frame):
    global last_ts_gyro, accel_angle_x, accel_angle_y, accel_angle_z

    # gather IMU data
    accel = frame[2].as_motion_frame().get_motion_data()
    gyro = frame[3].as_motion_frame().get_motion_data()
    ts = frame.get_timestamp()
    last_ts_gyro = ts

    # accelerometer calculation
    accel_angle_z = math.degrees(math.atan2(accel.y, accel.z))
    accel_angle_x = math.degrees(math.atan2(accel.x, math.sqrt(accel.y * accel.y + accel.z * accel.z)))
    accel_angle_y = math.degrees(math.pi)


def get_camera_angle(frame):
    global last_ts_gyro, accel_angle_x, accel_angle_y, accel_angle_z

    # gather IMU data
    accel = frame[2].as_motion_frame().get_motion_data()
    gyro = frame[3].as_motion_frame().get_motion_data()
    ts = frame.get_timestamp()

    # gyrometer calculations
    dt_gyro = (ts - last_ts_gyro) / 1000
    last_ts_gyro = ts

    gyro_angle_x = gyro.x * dt_gyro
    gyro_angle_y = gyro.y * dt_gyro
    gyro_angle_z = gyro.z * dt_gyro

    dangleX = gyro_angle_x * 57.2958
    dangleY = gyro_angle_y * 57.2958
    dangleZ = gyro_angle_z * 57.2958

    totalgyroangleX = accel_angle_x + dangleX
    totalgyroangleY = accel_angle_y + dangleY
    totalgyroangleZ = accel_angle_z + dangleZ

    # accelerometer calculation
    accel_angle_z = math.degrees(math.atan2(accel.y, accel.z))
    accel_angle_x = math.degrees(math.atan2(accel.x, math.sqrt(accel.y * accel.y + accel.z * accel.z)))
    accel_angle_y = math.degrees(math.pi)

    # combining gyrometer and accelerometer angles
    combinedangleX = totalgyroangleX * alpha + accel_angle_x * (1 - alpha)
    combinedangleZ = totalgyroangleZ * alpha + accel_angle_z * (1 - alpha)
    combinedangleY = totalgyroangleY

    return combinedangleX, combinedangleY, (combinedangleZ+90)