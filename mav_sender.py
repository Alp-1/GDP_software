import time
import mav_listener

# 50.937472 -1.39575 is near B16
FAKE_LONGITUDE = -1.39575
FAKE_LATITUDE = 50.937472
FAKE_ALTITUDE = 0.0

# GPS_TYPE Enum
GPS_TYPE_NMEA = 5
GPS_TYPE_MAVLINK = 14

def get_gps_time(tnow):
    '''return gps_week and gps_week_ms for current unix time in seconds'''
    leapseconds = 18
    SEC_PER_WEEK = 7 * 86400
    UNIX_TO_GPS_EPOCH = 315964800

    epoch = UNIX_TO_GPS_EPOCH - leapseconds
    epoch_seconds = int(tnow - epoch)
    week = int(epoch_seconds) // SEC_PER_WEEK
    t_ms = int(tnow * 1000) % 1000
    week_ms = (epoch_seconds % SEC_PER_WEEK) * 1000 + ((t_ms//200) * 200)
    return week, week_ms

def switch_to_fake_gps(mavlink_connection):
    """Change GPS_TYPE to MAVLink GPS. This method will reboot the autopilot so
     that the change takes effect.
    """
    mavlink_connection.param_set_send("GPS_TYPE", GPS_TYPE_MAVLINK)
    mavlink_connection.reboot_autopilot()

def switch_to_real_gps(mavlink_connection):
    """Change GPS_TYPE to NMEA GPS. This method will reboot the autopilot so
     that the change takes effect.
    """
    mavlink_connection.param_set_send("GPS_TYPE", GPS_TYPE_NMEA)
    mavlink_connection.reboot_autopilot()

def send_fake_gps(mavlink_connection, lat=FAKE_LATITUDE, lon=FAKE_LONGITUDE, alt=FAKE_ALTITUDE):
    """Send a fake GPS message to test the vehicle in GUIDED mode indoors"""

    now = time.time()
    gps_week, gps_week_ms = get_gps_time(now)
    nsats = 12
    fix_type = 3
    time_us = int(now * 1e6)

    mavlink_connection.mav.gps_input_send(time_us, 0, 0, gps_week_ms, gps_week, fix_type,
                                       int(lat*1.0e7), int(lon*1.0e7), alt,
                                       1.0, 1.0,
                                       0, 0, 0,
                                       0.2, 1.0, 1.0,
                                       nsats,
                                       0)

def scale(val, src, dst):
    """
    Scale the given value from the scale of src to the scale of dst.
    """
    return ((val - src[0]) / (src[1]-src[0])) * (dst[1]-dst[0]) + dst[0]

# Function to RC overrides (Only the first 4 channels)
def override_rc_channels(mavlink_connection, ch1, ch2, ch3, ch4):
    """
    Override RC channels using pymavlink.
    ch1, ch2, ch3, ch4: Channel values (1000 to 2000)
    """
    mavlink_connection.mav.rc_channels_override_send(
        mavlink_connection.target_system,  # target_system
        mavlink_connection.target_component,  # target_component
        ch1, ch2, ch3, ch4, 0, 0, 0, 0)  # channels 1-8 (set channels 5-8 to 0)


def move_backward(mavlink_connection, speed:float):
    """Move backward at speed (0-100)"""

    # Store current mode
    current_vehicle_mode = mav_listener.get_mav_mode(mavlink_connection)

    # Put into Manual mode
    mavlink_connection.set_mode_apm("MANUAL")
    time.sleep(1)
    channel_pwm = round(scale(speed, (0,100), (1500, 2000)))

    override_rc_channels(mavlink_connection, 0, channel_pwm, 0, 0)
    time.sleep(0.5)

    # Put the vehicle back to intiial mode
    mavlink_connection.set_mode_apm(current_vehicle_mode)
