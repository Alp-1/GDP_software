"""The module with the functions to parse the mavlink messages"""


import time
from pymavlink import mavutil
import math


def wait_for_msg(mavlink_connection, msg_name, flush=True, timeout=5, condition=None):
    """Wait for a message to be received, so that we can access its data"""
    if flush:
        try:
            mavlink_connection.port.flushInput()
        except AttributeError:
            pass
    mavlink_connection.recv_match(
        type=msg_name, blocking=True, timeout=timeout, condition=condition
    )
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


def get_motor_encoder_data(mavlink_connection):
    """Return the encoder data as an array of [front left, front right, rear left, rear right]"""
    named_value_float_msg = wait_for_msg(
        mavlink_connection,
        "NAMED_VALUE_FLOAT",
        flush=False,
        condition="NAMED_VALUE_FLOAT[a] and NAMED_VALUE_FLOAT[b] and NAMED_VALUE_FLOAT[c] and NAMED_VALUE_FLOAT[d]",
    )

    result = [
        named_value_float_msg["a"].value,
        named_value_float_msg["b"].value,
        named_value_float_msg["c"].value,
        named_value_float_msg["d"].value,
    ]
    return result


def get_motor_current_data(mavlink_connection):
    """Return the current data as an array of [front left, front right, rear left, rear right]"""
    named_value_float_msg = wait_for_msg(
        mavlink_connection,
        "NAMED_VALUE_FLOAT",
        flush=False,
        condition="NAMED_VALUE_FLOAT[e] and NAMED_VALUE_FLOAT[f] and NAMED_VALUE_FLOAT[g] and NAMED_VALUE_FLOAT[h]",
    )
    result = [
        named_value_float_msg["e"].value,
        named_value_float_msg["f"].value,
        named_value_float_msg["g"].value,
        named_value_float_msg["h"].value,
    ]
    return result


def get_mav_mode(mavlink_connection):
    """Return the mode of the flight controller"""
    # MAV_TYPE.MAV_TYPE_GROUND_ROVER = 10
    heartbeat_msg = wait_for_msg(
        mavlink_connection, "HEARTBEAT", condition="HEARTBEAT.type==10"
    )
    return mavutil.mode_string_v10(heartbeat_msg)


def get_wheel_distances(mavlink_connection):
    """Return the rear left and rear right wheel distances in meters"""
    wheel_distance_msg = wait_for_msg(mavlink_connection, "WHEEL_DISTANCE")
    return wheel_distance_msg.distance[0], wheel_distance_msg.distance[1]


def set_message_interval(mavlink_connection, msg_id, interval_us):
    """Set the interval between messages in microseconds"""
    mavlink_connection.mav.command_long_send(
        mavlink_connection.target_system,
        mavlink_connection.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        msg_id,
        interval_us,
        0,
        0,
        0,
        0,
        0,
    )
    # Wait for a response (blocking) to the MAV_CMD_SET_MESSAGE_INTERVAL command and print result
    response = mavlink_connection.recv_match(type="COMMAND_ACK", blocking=True)
    if (
        response
        and response.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL
        and response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
    ):
        print("Command accepted")
    else:
        print("Command failed")


def get_imu_data(mavlink_connection):
    """Return the imu data as an array of [roll, pitch, yaw]"""
    msg = wait_for_msg(mavlink_connection, "ATTITUDE")
    result = [msg.roll, msg.pitch, msg.yaw]

    return result

def get_cmd_long(mavlink_connection, command_id):
    """Check if command long based on type exists"""
    msg = wait_for_msg(
        mavlink_connection, "COMMAND_LONG", condition=f"command=={command_id}"
    )
    if msg is None:
        return False
    else:
        return True


def initialise_mavlink(connection_string="/dev/ttyAMA0", baud=57600):
    """Initialise mavlink connection"""
    mavlink_connection = mavutil.mavlink_connection(connection_string, baud=baud)
    wait_for_msg(mavlink_connection, "HEARTBEAT", condition="HEARTBEAT.type==10")
    print(
        "Heartbeat from MAVLink system (system %u component %u)"
        % (mavlink_connection.target_system, mavlink_connection.target_component)
    )
    return mavlink_connection


if __name__ == "__main__":
    mavlink_connection = initialise_mavlink()
    while True:
        time.sleep(0.5)
        print("Speed: ", get_rover_speed(mavlink_connection))
        print("Power: ", get_instantaneous_power(mavlink_connection))
        print("Encoders: ", get_motor_encoder_data(mavlink_connection))
        print("Currents: ", get_motor_current_data(mavlink_connection))
        print("Mode: ", get_mav_mode(mavlink_connection))
        print(" ")
