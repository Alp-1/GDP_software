import time

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
