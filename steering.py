import cv2
import pyrealsense2 as rs
import time
import numpy as np

# Initialize the RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)     # Depth stream
pipeline.start(config)
# Get the depth stream profile to access stream properties
depth_stream = pipeline.get_active_profile().get_stream(rs.stream.depth)

# Get the width of the depth stream
depth_width = depth_stream.as_video_stream_profile().get_intrinsics().width
print(depth_width)
try:
    while True:
        # Wait for the next frame
        frames = pipeline.wait_for_frames()

        # Get the depth frame
        depth_frame = frames.get_depth_frame()

        if not depth_frame:
            continue

        # Convert depth frame to a numpy array
        depth_image = np.asanyarray(depth_frame.get_data())
        depth_image = cv2.convertScaleAbs(depth_image, alpha=0.03)  # Adjust depth visualization

        column_means = np.mean(depth_image, axis=0)
        print(len(column_means))
        # Find the index of the column with the highest mean
        index_of_highest_mean = np.argmax(column_means)
        print("Index of column with highest mean:", index_of_highest_mean)
        print("angle:",index_of_highest_mean/len(column_means)*87-(87/2))
        # Display the depth image
        # Apply a colormap to the depth image
        color_depth_image = cv2.applyColorMap(depth_image, cv2.COLORMAP_JET)
        cv2.imshow("Depth Image", color_depth_image)
        cv2.waitKey(1)

        # # Capture the depth image to a file (you can choose your own filename and format)
        # timestamp = time.time()
        # image_filename = f"depth_{int(timestamp)}.png"
        # cv2.imwrite(image_filename, depth_image)

        time.sleep(0.5)  # Capture every 1 second

except KeyboardInterrupt:
    pass

# Release resources
pipeline.stop()
cv2.destroyAllWindows()
