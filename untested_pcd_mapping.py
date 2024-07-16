"""UNTESTED code to visualise the route in 3D"""

from collections import deque
import time
from pyrealsense import pyrealsense2 as rs
import open3d as o3d
import numpy as np
from navigation.navigation_final import initialize_realsense
from route_main import Position, get_position

def create_point_cloud(pipeline=None):
    """Create point cloud from realsense camera. If pipeline is None, a random point cloud is returned"""
    if pipeline:
        frames = pipeline.wait_for_frames()
        # prof = frames.get_profile()

        # depth_intrinsics = prof.as_video_stream_profile().get_intrinsics()

        depth_frame = frames.get_depth_frame()
        if not depth_frame:
            return None
        # Create a point cloud
        pc = rs.pointcloud()
        points = pc.calculate(depth_frame)

        # Convert points to Open3D format
        vtx = np.asanyarray(points.get_vertices())
        vtx = np.array([[v[0], v[1], v[2]] for v in vtx])

        # Create Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(vtx)
    else:
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.random.rand(1000, 3))
    return pcd


def visualise_route(vis, line_set, route):
    """This function should draw the route in the 3d visualisation"""
    if len(route) < 2:
        return
    lines = [[i, i + 1] for i in range(len(route) - 1)]
    xyz_array = [[pos.x, pos.y, 0] for pos in route]
    line_set.points = o3d.utility.Vector3dVector(xyz_array)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(np.tile([1, 0, 0], (len(lines), 1)))
    vis.update_geometry(line_set)


def translate_point_cloud(pcd, translation):
    pcd.translate(translation)
    return pcd


def mock_get_position():
    """Test function to simulate the get_position function"""
    # Produce some diagonal positions
    for i in range(1000):
        k = i / 100
        yield Position(k, k, k, 45)
    yield Position(-1, -1, -1, -1)


def main(mavlink_connection, sampling_period=0.1):

    pipeline, profile = initialize_realsense()
    pcd = create_point_cloud(pipeline)
    positions = deque(maxlen=1000)

    # Translate the point cloud so that
    # it aligns with the robot's origin local position (x,y,z)
    # and subsequent positions are relative to the robot's origin
    translation = [0, 0, 0]
    pcd = translate_point_cloud(pcd, translation)

    # Initialize the Open3D visualizer
    vis = o3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(pcd)

    # Initialize line set for the route
    line_set = o3d.geometry.LineSet()
    vis.add_geometry(line_set)

    # Add a coordinate frame
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5, origin=[0, 0, 0])
    vis.add_geometry(axis)

    keep_running = True
    # iter_pos = mock_get_position()
    while keep_running:
        current_pos = get_position(mavlink_connection)
        # current_pos = next(iter_pos)
        if current_pos.time != -1:
            positions.append(current_pos)
            visualise_route(vis, line_set, positions)
        time.sleep(sampling_period)

        keep_running = vis.poll_events()
        vis.update_renderer()

    vis.destroy_window()
