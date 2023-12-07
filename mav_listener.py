"""The module with the functions to parse the mavlink messages"""


import time
from pymavlink import mavutil
import math


def wait_for_msg(mavlink_connection, msg_name, timeout=5):
    """Wait for a message to be received, so that we can access its data"""
    mavlink_connection.recv_match(type=msg_name, blocking=True, timeout=timeout)
    try:
        msg = mavlink_connection.messages[msg_name]
        return msg
    except KeyError:
        print("Message %s not found" % msg_name)
        return None


def get_rover_speed(mavlink_connection):
    """Return the speed in cm/s"""
    global_position_int_msg = wait_for_msg(mavlink_connection, "GLOBAL_POSITION_INT")
    speed = math.sqrt(global_position_int_msg.vx**2 + global_position_int_msg.vy**2)
    return speed


def get_instantaneous_power(mavlink_connection):
    """Return the voltage and current in volts and amps"""
    sys_status_msg = wait_for_msg(mavlink_connection, "SYS_STATUS")
    voltage = sys_status_msg.voltage_battery / 1000
    current = sys_status_msg.current_battery / 100
    print("Voltage (V): ", voltage)
    print("Current (A): ", current)
    return voltage * current


def get_encoder_data(mavlink_connection):
    """Return the encoder data as an array of [front left, front right, rear left, rear right]"""
    named_value_float_msg = wait_for_msg(mavlink_connection, "NAMED_VALUE_FLOAT")
    return [
        named_value_float_msg["a"].value,
        named_value_float_msg["b"].value,
        named_value_float_msg["c"].value,
        named_value_float_msg["d"].value,
    ]


def get_current_data(mavlink_connection):
    """Return the current data as an array of [front left, front right, rear left, rear right]"""
    named_value_float_msg = wait_for_msg(mavlink_connection, "NAMED_VALUE_FLOAT")
    return [
        named_value_float_msg["e"].value,
        named_value_float_msg["f"].value,
        named_value_float_msg["g"].value,
        named_value_float_msg["h"].value,
    ]


def get_mav_mode(mavlink_connection):
    """Return the mode of the flight controller"""
    mav_mode_msg = wait_for_msg(mavlink_connection, "MAV_MODE")
    # print("Mode: ", mav_mode_msg.mode)
    return mav_mode_msg.mode


def initialise_mavlink(connection_string="/dev/serial0", baud=57600):
    """Initialise mavlink connection"""
    mavlink_connection = mavutil.mavlink_connection(connection_string, baud=baud)
    mavlink_connection.wait_heartbeat()
    print(
        "Heartbeat from MAVLink system (system %u component %u)"
        % (mavlink_connection.target_system, mavlink_connection.target_component)
    )
    return mavlink_connection


if __name__ == "__main__":
    mavlink_connection = initialise_mavlink()
    while True:
        print("Speed: ", get_rover_speed(mavlink_connection))
        print("Power: ", get_instantaneous_power(mavlink_connection))
        print("Encoders: ", get_encoder_data(mavlink_connection))
        print("Currents: ", get_current_data(mavlink_connection))
        print("Mode: ", get_mav_mode(mavlink_connection))
        print(" ")
        time.sleep(2)
