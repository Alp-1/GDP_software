"""Creates a bridge between the Raspberry Pi Pico and a flight controller via MAVLink
# This requires the RP2040's UART (Rx, Tx, Gnd, Pwr) to be connected to a flight controller's
# TELEM ports.

# The TELEM port should be set to MAVLink2, 57600 baud
"""

import asyncio
from machine import Pin
import time
from asyncio import Lock

# import upy_mavlink.pymavminimal as pymav
from upy_mavlink.src.mavlite import MavLink, UART


m_id = [36, 251, 512]
mavobj = MavLink(message_ids=m_id)

mavlink_lock = Lock()


async def mavlink_main():
    """Main loop to receive and transmit data"""
    tasks = await mavobj.io_buffers(
        UART(s_id=0, baudrate=57600, tx=Pin(16), rx=Pin(17)),
        debug=False,
    )
    # [write_loop, read_loop, heartbeat_loop, command_listener]
    await asyncio.gather(
        tasks[0],
        tasks[2],
    )


async def send_name_value_floats(
    name: str,
    value: float,
):
    """Send a name and value to the flight controller"""
    await mavlink_lock.acquire()
    await mavobj.send_message(
        message_id=251,
        payload=[time.ticks_ms(), name, value],
    )
    mavlink_lock.release()
