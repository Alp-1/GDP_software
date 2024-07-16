
from route_main import Position
import csv
import matplotlib.pyplot as plt
import numpy as np



def read_positions_from_csv(filename: str) -> list[Position]:
    positions = []
    with open(filename, mode='r') as file:
        reader = csv.DictReader(file)
        for row in reader:
            positions.append(Position(time=float(row["time"]),
                                      x=float(row["x"]),
                                      y=float(row["y"]),
                                      heading=float(row["heading"])))
    return positions

def plot_reference_square(square_bottom_left: tuple[float, float], side_length: float):

    # Reference square coordinates
    square_x = [square_bottom_left[0], square_bottom_left[0] + side_length, square_bottom_left[0] + side_length, square_bottom_left[0], square_bottom_left[0]]
    square_y = [square_bottom_left[1], square_bottom_left[1], square_bottom_left[1] + side_length, square_bottom_left[1] + side_length, square_bottom_left[1]]

    plt.figure(0)
    plt.plot(square_x, square_y, linestyle='--', color='red', label='Reference Square')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title('Route Plot with Reference Square')
    plt.legend()
    plt.grid(True)
    # plt.show()

def plot_route(positions: list[Position], arrow_length: float = 0.1):
    x = [pos.x for pos in positions]
    y = [pos.y for pos in positions]
    u = [np.cos(np.deg2rad(pos.heading)) for pos in positions]
    v = [np.sin(np.deg2rad(pos.heading)) for pos in positions]

    # Normalize the arrow lengths to unit vectors
    norm = np.sqrt(np.array(u)**2 + np.array(v)**2)
    u = (np.array(u) / norm) * arrow_length
    v = (np.array(v) / norm) * arrow_length

    plt.figure(0)
    plt.plot(x, y, marker='o', linestyle='-', color='blue', label='Measurement Route')
    # plt.quiver(x, y, u, v, angles='xy', scale_units='xy', scale=1, color='blue')
    plt.xlabel('X Position')
    plt.ylabel('Y Position')
    plt.title('Route Plot')
    plt.legend()
    plt.grid(True)
    # plt.show()


if __name__ == "__main__":
    positions = read_positions_from_csv("positions_optflow_heavy.csv")
    plot_route(positions)
    # plot_reference_square(square_bottom_left=(0.5, 0.5), side_length=2)
    plt.show()
