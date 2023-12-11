import open3d as o3d
import numpy as np
import pyrealsense2 as rs
import time
import scipy
import cv2
# cell_height = 120
# cell_width = 212
# grid_n = 4
# grid_m = 4
from matplotlib import pyplot as plt

cell_height = 60
cell_width = 106
grid_n = 8
grid_m = 8

def main():
    global profile
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 848, 480, rs.format.bgr8, 60)  # RGB stream
    config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 60)  # Depth stream
    config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 200)  # Accelerometer data
    config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 400)  # Gyroscope data
    profile = pipeline.start(config)

    frames = pipeline.wait_for_frames()
    prof = frames.get_profile()
    depth_intrinsics = prof.as_video_stream_profile().get_intrinsics()

    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()

    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()

    hole_filling = rs.hole_filling_filter(1)
    threshold_filter = rs.threshold_filter(0.3, 16)
    depth_frame = threshold_filter.process(depth_frame)
    depth_frame = hole_filling.process(depth_frame)
    depth_image = np.asanyarray(depth_frame.get_data()) * depth_scale


    print(np.min(depth_image))
    print(np.max(depth_image))
    num_zeros = np.count_nonzero(depth_image == 0)
    print(num_zeros)
    start_time = time.time()

    depth_grid = scipy.sparse.bsr_matrix(depth_image, blocksize=(cell_height,cell_width)).data
    np_grid = np.asanyarray(depth_grid)
    image_o3d = o3d.geometry.Image(depth_image.astype(np.float32))
    i=0
    j=0
    slope_grid = np.zeros((grid_n,grid_m))
    for cell in np_grid:
        image_o3d = o3d.geometry.Image(cell.astype(np.float32))
        pcd = o3d.geometry.PointCloud.create_from_depth_image(
            image_o3d, o3d.camera.PinholeCameraIntrinsic(depth_intrinsics.width, depth_intrinsics.height,
                                                         depth_intrinsics.fx, depth_intrinsics.fy,
                                                         depth_intrinsics.ppx, depth_intrinsics.ppy))
        downpcd = pcd.voxel_down_sample(voxel_size=0.05)
        plane_model, inliers = downpcd.segment_plane(distance_threshold=0.1,
                                                     ransac_n=3,
                                                     num_iterations=100)
        [a, b, c, d] = plane_model
        if a==0:
            slope = 100
        else:
            slope = -c / a
        slope_grid[i,j] = slope
        if j==(grid_n-1):
            i += 1
            j = 0
        else:
            j += 1
        # print(i)
        # print(j)
        # print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

    # slope_grid = slope_grid[:-1,:]

    for row in slope_grid:
        print(" ".join(map(str, row)))
    image_o3d = o3d.geometry.Image(depth_image.astype(np.float32))
    pcd = o3d.geometry.PointCloud.create_from_depth_image(
        image_o3d,o3d.camera.PinholeCameraIntrinsic(depth_intrinsics.width, depth_intrinsics.height,
                                                          depth_intrinsics.fx, depth_intrinsics.fy,
                                                          depth_intrinsics.ppx, depth_intrinsics.ppy))
    # print(pcd)
    # downpcd = pcd.voxel_down_sample(voxel_size=0.05)
    # plane_model, inliers = downpcd.segment_plane(distance_threshold=0.1,
    #                                      ransac_n=3,
    #                                      num_iterations=200)
    # o3d.visualization.draw_geometries([pcd])
    # o3d.visualization.draw_geometries([pcd],
    #                                   zoom=0.3412,
    #                                   front=[0.4257, -0.2125, -0.8795],
    #                                   lookat=[2.6172, 2.0475, 1.532],
    #                                   up=[-0.0694, -0.9768, 0.2024])
    # [a, b, c, d] = plane_model
    # print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
    # slope = c/a
    # # step height =
    # print(c)
    # print(a)
    # print(f"slope:{slope}")
    # inlier_cloud = downpcd.select_by_index(inliers)
    # inlier_cloud.paint_uniform_color([1.0, 0, 0])
    # outlier_cloud = downpcd.select_by_index(inliers, invert=True)
    # o3d.visualization.draw_geometries([inlier_cloud,outlier_cloud],
    #                                   zoom=0.8,
    #                                   front=[-0.4999, -0.1659, -0.8499],
    #                                   lookat=[2.1813, 2.0619, 2.0999],
    #                                   up=[0.1204, -0.9852, 0.1215])
    #
    # inlier_points = np.asarray(inlier_cloud)
    # min_z = np.min(inlier_cloud[:, 2])
    # max_z = np.max(inlier_cloud[:, 2])
    # print(f"step height: {max_z-min_z}")
    print("--- %s seconds ---" % (time.time() - start_time))

if __name__ == "__main__":
    main()
    # testing()