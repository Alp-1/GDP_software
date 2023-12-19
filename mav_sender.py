import time
from pymavlink import mavutil



def send_fake_gps(mavlink_connection, lat, lon, alt):
    """Send a fake GPS message to test the vehicle in GUIDED mode indoors"""
    now_us = int(time.time() * 1000000)

    mavlink_connection.mav.gps_raw_int_send(
        now_us,  # time_usec                 : Timestamp (microseconds since UNIX epoch or microseconds since system boot) (uint64_t)
        3,  # fix_type                  : See the GPS_FIX_TYPE enum. (uint8_t)
        int(lat * 1e7),  # lat                       : Latitude (WGS84), in degrees * 1E7 (int32_t)
        int(lon * 1e7),  # lon                       : Longitude (WGS84), in degrees * 1E7 (int32_t)
        int(alt * 1000),  # alt                       : Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide the AMSL altitude in addition to the WGS84 altitude. (int32_t)
        0,  # eph                       : GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX (uint16_t)
        0,  # epv                       : GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX (uint16_t)
        0,  # vel                       : GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX (uint16_t)
        0,  # cog                       : Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX (uint16_t)
        12  # satellites_visible        : Number of satellites visible. If unknown, set to 255 (uint8_t)
    )
