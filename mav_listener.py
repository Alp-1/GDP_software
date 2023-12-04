"""THE main loop for connection with flight controller"""


from pymavlink import mavutil
import asyncio
import math


# Global variables to store the encoder and current data
encoders = [0, 0, 0, 0]  # Front left, front right, rear left, rear right
currents = [0, 0, 0, 0]  # Front left, front right, rear left, rear right


def handle_glob_pos_int(global_position_int_msg):
    """Return the speed in cm/s"""
    speed = math.sqrt(global_position_int_msg.vx**2 + global_position_int_msg.vy**2)
    return speed


def handle_sys_status(sys_status_msg):
    """Return the voltage and current in volts and amps"""
    voltage = sys_status_msg.voltage_battery / 1000
    current = sys_status_msg.current_battery / 100
    return voltage, current


def handle_named_value_float(named_value_float_msg):
    """Return the encoder and current data of each wheel from the pico"""
    global encoders, currents
    if named_value_float_msg.name == "a":
        encoders[0] = named_value_float_msg.value
    elif named_value_float_msg.name == "b":
        encoders[1] = named_value_float_msg.value
    elif named_value_float_msg.name == "c":
        encoders[2] = named_value_float_msg.value
    elif named_value_float_msg.name == "d":
        encoders[3] = named_value_float_msg.value
    elif named_value_float_msg.name == "e":
        currents[0] = named_value_float_msg.value
    elif named_value_float_msg.name == "f":
        currents[1] = named_value_float_msg.value
    elif named_value_float_msg.name == "g":
        currents[2] = named_value_float_msg.value
    elif named_value_float_msg.name == "h":
        currents[3] = named_value_float_msg.value


def handle_mav_mode(mav_mode_msg):
    """Return the mode of the flight controller"""
    print("Mode: ", mav_mode_msg.mode)


def intialise_mavlink(connection_string="/dev/serial0", baud=57600):
    """Initialise mavlink connection"""
    mavlink_connection = mavutil.mavlink_connection(connection_string, baud=baud)
    mavlink_connection.wait_heartbeat()
    print(
        "Heartbeat from MAVLink system (system %u component %u)"
        % (mavlink_connection.target_system, mavlink_connection.target_component)
    )
    return mavlink_connection


async def listen_loop(mavlink_connection):
    """Loop on mavlink connection and do action based on data."""

    while True:
        mav_msg = mavlink_connection.recv_match(blocking=False)
        if mav_msg is None:
            continue
        if mav_msg.get_type() == "BAD_DATA":
            print("Bad data")
            continue
        if mav_msg.get_type() == "GLOBAL_POSITION_INT":
            vehicle_speed = handle_glob_pos_int(mav_msg)
            print("Speed is: cm/s", vehicle_speed)
        if mav_msg.get_type() == "SYS_STATUS":
            batt_voltage, batt_current = handle_sys_status(mav_msg)
            print("Battery voltage: V", batt_voltage)
            print("Battery current: A", batt_current)

        if mav_msg.get_type() == "NAMED_VALUE_FLOAT":
            handle_named_value_float(mav_msg)
            print("Encoders: ", encoders)
            print("Currents: ", currents)

        if mav_msg.get_type() == "MAV_MODE":
            handle_mav_mode(mav_msg)

        # Add more if we are waiting for more messages

        await asyncio.sleep(0.01)
