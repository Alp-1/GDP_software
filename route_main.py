import csv
import time
from dataclasses import dataclass
from navigation.mav_listener import get_heading, get_local_position, initialise_mavlink

# from collections import deque


@dataclass
class Position:
    time: float
    x: float
    y: float
    heading: float


sampling_period = 100e-3  # seconds
# route_period = 5  # seconds
# number of points in the route
# max_route_length = int(route_period / sampling_period)
# max_route_length = 10e3
# Create a circular buffer
# positions: deque[Position] = deque(maxlen=max_route_length)
positions = []  # A regular list is used instead since circular buffer is not needed atm


def get_position(mavlink_connection):
    time, x, y, _ = get_local_position(mavlink_connection)
    if time is None:
        # Return invalid position if no position data is available
        return Position(-1, -1, -1, -1)
    return Position(time, x, y, get_heading(mavlink_connection))


def get_positions_blocking(mavlink_connection):
    """Collect a series of positions until the user interrupts the program."""
    clear_positions()
    try:
        while True:
            temp_pos = get_position(mavlink_connection)
            positions.append(temp_pos)
            time.sleep(sampling_period)
    except KeyboardInterrupt:
        pass
    return positions


def get_route():
    return positions


def clear_positions():
    positions.clear()


def export_positions_to_csv(positions, filename):
    """Export a series of positions to a CSV file."""
    with open(filename, mode="w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(["time", "x", "y", "heading"])
        for pos in positions:
            writer.writerow([pos.time, pos.x, pos.y, pos.heading])


# Example Usage:
if __name__ == "__main__":
    # # Create some example positions
    # positions = [
    #     Position(time=0, x=0, y=0, heading=0),
    #     Position(time=1, x=1, y=1, heading=45),
    #     Position(time=2, x=2, y=1, heading=90),
    #     Position(time=3, x=3, y=0, heading=135),
    # ]

    mavlink_connection = initialise_mavlink()
    pos = get_positions_blocking(mavlink_connection)
    # Export positions to CSV
    export_positions_to_csv(pos, "positions.csv")
