import cv2
import numpy as np
import pyrealsense2 as rs

def calibrate_realsense():
    # Configure RealSense pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    # Prepare object points
    checkerboard_size = (9, 6)  # Number of inner corners on the checkerboard
    square_size = 0.0254  # Size of a square in meters (adjust as needed)
    objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2)
    objp *= square_size

    # Arrays to store object points and image points
    objpoints = []  # 3D points in real world space
    imgpoints = []  # 2D points in image plane

    try:
        while len(objpoints) < 20:  # Collect 20 valid frames
            # Wait for a coherent frame
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            # Convert images to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, checkerboard_size, None)

            # If found, add object points, image points
            if ret:
                objpoints.append(objp)
                corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), 
                                            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
                imgpoints.append(corners2)

                # Draw and display the corners
                cv2.drawChessboardCorners(color_image, checkerboard_size, corners2, ret)
                cv2.imshow('Calibration', color_image)
                cv2.waitKey(500)

            print(f"Collected {len(objpoints)} valid frames")

        print("Calibrating camera...")
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

        # Get the intrinsics
        intrinsics = color_frame.profile.as_video_stream_profile().intrinsics

        # Print calibration results
        print("\nCalibration Results:")
        print("Camera Matrix:")
        print(mtx)
        print("\nDistortion Coefficients:")
        print(dist)
        print("\nCamera Intrinsics:")
        print(f"Width: {intrinsics.width}")
        print(f"Height: {intrinsics.height}")
        print(f"PPX (Principal Point X): {intrinsics.ppx}")
        print(f"PPY (Principal Point Y): {intrinsics.ppy}")
        print(f"FX (Focal Length X): {intrinsics.fx}")
        print(f"FY (Focal Length Y): {intrinsics.fy}")
        print(f"Distortion Model: {intrinsics.model}")
        print(f"Distortion Coefficients: {intrinsics.coeffs}")

    finally:
        # Stop streaming
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    calibrate_realsense()