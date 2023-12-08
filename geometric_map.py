import cv2
import open3d as o3d
import numpy as np
import pyrealsense2 as rs
import time

def capture_depth_image():
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 60)

    pipeline.start(config)
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()

    depth_data = np.asanyarray(depth_frame.get_data())

    pipeline.stop()
    return depth_frame


def create_point_cloud(depth_frame):
    intrinsics = rs.video_stream_profile(depth_frame.profile).get_intrinsics()
    points = rs.points()
    # points.import_depth_frame(depth_frame)
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(points.get_vertices())

    return pc


def fit_plane_to_point_cloud(point_cloud):
    # Apply RANSAC plane fitting
    plane_model, inliers = point_cloud.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)

    inlier_cloud = point_cloud.select_by_index(inliers)
    outlier_cloud = point_cloud.select_by_index(inliers, invert=True)

    return plane_model, inlier_cloud, outlier_cloud


def main():
    o3d.t.io.RealSenseSensor.list_devices()
    start_time = time.time()
    x = np.random.random((200000, 3))
    y = o3d.utility.Vector3dVector(x)
    depth_frame = capture_depth_image()
    depth_image = np.asanyarray(depth_frame.get_data())
    pcd = o3d.geometry.PointCloud.create_from_depth_image(depth_image,o3d.camera.PinholeCameraIntrinsic)
    print(pcd)

    print("--- %s seconds ---" % (time.time() - start_time))
if __name__ == "__main__":
    main()
