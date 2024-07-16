""" Dead End Stuff """

import json
import math
import numpy as np
from pyrealsense import pyrealsense2 as rs
from dronekit import *
import pymavlink
from mav_listener import *


def record_heading():
    """Records heading at dead-end"""
    mavlink_connection = initialise_mavlink()
    heading_at_point = get_heading(mavlink_connection)
    return heading_at_point


def determine_forbidden_zone(size):
    """
    Establishes the coordinate barrier for the previously encountered deadzone for small areas.

    Args:
    size (float): The half-length of one side of the square forbidden zone, in meters.

    Returns:
    dict: A dictionary containing the coordinates of the forbidden zone.
    """
    # Initialize MAVLink connection
    mavlink_connection = initialise_mavlink()

    # Get latitude and longitude at dead end
    lat_at_deadend = get_fused_lat(mavlink_connection)
    lon_at_deadend = get_fused_lon(mavlink_connection)

    # Convert the size from meters to degrees (simplified for small distances)
    size_in_degrees_lat = meters_to_lat_degrees(size)
    size_in_degrees_lon = meters_to_lon_degrees(size, lat_at_deadend)

    # Calculate the boundaries of the forbidden zone
    north_boundary = lat_at_deadend + size_in_degrees_lat
    south_boundary = lat_at_deadend - size_in_degrees_lat
    east_boundary = lon_at_deadend + size_in_degrees_lon
    west_boundary = lon_at_deadend - size_in_degrees_lon

    return {
        "north": north_boundary,
        "south": south_boundary,
        "east": east_boundary,
        "west": west_boundary
    }


def meters_to_lat_degrees(meters):
    """
    Converts a distance in meters to a change in latitude degrees.

    Args:
    meters (float): The distance in meters.

    Returns:
    float: The distance in latitude degrees.
    """
    # Approximate conversion factor for latitude
    return meters / 111111  # One degree of latitude is approximately 111111 meters


def meters_to_lon_degrees(meters, latitude):
    """
    Converts a distance in meters to a change in longitude degrees at a given latitude.

    Args:
    meters (float): The distance in meters.
    latitude (float): The latitude at which the conversion is taking place.

    Returns:
    float: The distance in longitude degrees.
    """
    # Earth's radius in meters at the equator
    earth_radius = 6378137.0

    # Calculate the width of one longitude degree in meters at the given latitude
    longitude_degree_width = math.cos(math.radians(latitude)) * (math.pi * earth_radius / 180)

    return meters / longitude_degree_width

# def return_to_the_last_gps_point(lat, lon, connection_string):
#     """
#     Establishes a MAVLink connection, sends a command to return to the last known GPS point,
#     and keeps the connection open until the vehicle reaches the point.

#     Args:
#     lat (float): Latitude of the target location.
#     lon (float): Longitude of the target location.
#     connection_string (str): String to establish a MAVLink connection (e.g., serial port, UDP endpoint).
#     """
#     # Establish the MAVLink connection
#     mavlink_connection = mavutil.mavlink_connection(connection_string)

#     # Wait for the first heartbeat
#     mavlink_connection.wait_heartbeat()

#     # Convert latitude and longitude to the format expected by MAVLink (degrees * 1E7)
#     lat_int = int(lat * 1e7)
#     lon_int = int(lon * 1e7)

#     # Altitude is not specified, so set it to a default or retrieve current altitude
#     default_altitude = 10  # Example altitude in meters

#     # Create the SET_POSITION_TARGET_GLOBAL_INT message
#     msg = mavlink_connection.mav.set_position_target_global_int_encode(
#         0,  # time_boot_ms (not used)
#         0, 0,  # target system, target component
#         mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
#         0b0000111111111000,  # type_mask (only positions enabled)
#         lat_int,  # lat_int
#         lon_int,  # lon_int
#         default_altitude,  # alt
#         0, 0, 0,  # X, Y, Z velocity
#         0, 0, 0,  # X, Y, Z acceleration
#         0, 0)  # yaw, yaw_rate

#     # Send the command
#     mavlink_connection.mav.send(msg)

#     # Wait until the vehicle reaches the target
#     arrival_threshold = 1.0  # distance in meters, adjust as necessary
#     while True:
#         # Fetch current vehicle position
#         current_msg = mavlink_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
#         current_lat = current_msg.lat / 1e7
#         current_lon = current_msg.lon / 1e7

#         # Check if the vehicle is within the arrival threshold
#         if get_distance_meters(current_lat, current_lon, lat, lon) < arrival_threshold:
#             print("Arrived at target location")
#             break

#         # Sleep for a short duration before checking again
#         time.sleep(1)

#     # Connection remains open for further operations or monitoring
#     # Remember to close it elsewhere in your program when appropriate

# def get_distance_meters(lat1, lon1, lat2, lon2):
#     """
#     Calculates the distance in meters between two GPS coordinates.
#     """
#     # approximate radius of earth in meters
#     R = 6373000.0

#     lat1 = math.radians(lat1)
#     lon1 = math.radians(lon1)
#     lat2 = math.radians(lat2)
#     lon2 = math.radians(lon2)

#     dlon = lon2 - lon1
#     dlat = lat2 - lat1

#     a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
#     c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

#     distance = R * c
#     return distance

# def set_new_clear_path(connection_string, dead_end_heading, forbidden_zone, escape_distance):
#     """
#     Aligns the vehicle's heading with the recorded heading, calculates a new GPS position outside
#     the forbidden zone, sends a command to move to this new position,
#     and waits until the vehicle reaches the new position before terminating the connection.

#     Args:
#     connection_string (str): String to establish a MAVLink connection.
#     dead_end_heading (float): The recorded heading at the dead-end.
#     forbidden_zone (dict): The dictionary containing the coordinates of the forbidden zone.
#     escape_distance (float): Distance to move away from the forbidden zone, in meters.
#     """
#     # Establish the MAVLink connection
#     mavlink_connection = mavutil.mavlink_connection(connection_string)
#     mavlink_connection.wait_heartbeat()

#     # Align the vehicle's heading with the recorded heading
#     command_long_message = mavlink_connection.mav.command_long_encode(
#         0, 0,  # target system, target component
#         mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
#         0,  # confirmation
#         dead_end_heading,  # param1: Yaw angle (degrees)
#         0,  # param2: Yaw speed not used
#         1,  # param3: Direction - clockwise
#         0,  # param4: Relative - 0 = absolute angle
#         0, 0, 0  # params 5-7 not used
#     )
#     mavlink_connection.mav.send(command_long_message)

#     # Calculate a new GPS position outside of the forbidden zone
#     # Example: Moving directly north by the escape distance
#     center_lat = (forbidden_zone["north"] + forbidden_zone["south"]) / 2
#     center_lon = (forbidden_zone["east"] + forbidden_zone["west"]) / 2
#     new_lat = center_lat + meters_to_lat_degrees(escape_distance)
#     new_lon = center_lon  # Keeping the same longitude

#     # Send a command to move to the new position
#     # Convert latitude and longitude to the format expected by MAVLink (degrees * 1E7)
#     lat_int = int(new_lat * 1e7)
#     lon_int = int(new_lon * 1e7)
#     default_altitude = 10  # Example altitude in meters

#     set_position_message = mavlink_connection.mav.set_position_target_global_int_encode(
#         0,  # time_boot_ms (not used)
#         0, 0,  # target system, target component
#         mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
#         0b0000111111111000,  # type_mask (only positions enabled)
#         lat_int,  # lat_int
#         lon_int,  # lon_int
#         default_altitude,  # alt
#         0, 0, 0,  # X, Y, Z velocity
#         0, 0, 0,  # X, Y, Z acceleration
#         0, 0)  # yaw, yaw_rate
#     mavlink_connection.mav.send(set_position_message)

#     # Wait until the vehicle reaches the new target position
#     arrival_threshold = 1.0  # Adjust as necessary
#     while True:
#         current_msg = mavlink_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
#         current_lat = current_msg.lat / 1e7
#         current_lon = current_msg.lon / 1e7

#         if get_distance_meters(current_lat, current_lon, new_lat, new_lon) < arrival_threshold:
#             print("Arrived at new position outside forbidden zone")
#             break

#         time.sleep(1)

#     # Close the MAVLink connection
#     mavlink_connection.close()
#     print("MAVLink connection closed.")


# # Main script execution
# if __name__ == "__main__":
#     # Connection string for MAVLink (adjust as per your setup, e.g., serial port, UDP, etc.)
#     connection_string = "'/dev/ttyAMA0', baud=57600"

#     # Record the heading at the dead-end
#     dead_end_heading = record_heading()
#     print(f"Recorded Heading at Dead End: {dead_end_heading} degrees")

#     # Determine the coordinates of the forbidden zone
#     forbidden_zone_size = 2  # Size in meters, adjust as needed
#     forbidden_zone = determine_forbidden_zone(forbidden_zone_size)
#     print(f"Forbidden Zone: {json.dumps(forbidden_zone, indent=2)}")

#     # Set a new clear path
#     escape_distance = 5  # Distance to move away from the forbidden zone, in meters
#     set_new_clear_path(connection_string, dead_end_heading, forbidden_zone, escape_distance)







