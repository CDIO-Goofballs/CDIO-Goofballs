import cv2
import numpy as np
import math
import cv2.aruco as aruco

# Camera parameters (same as before)
focal_length = 800
cx, cy = 320, 240

camera_matrix = np.array([
    [focal_length, 0, cx],
    [0, focal_length, cy],
    [0, 0, 1]
], dtype=np.float32)

dist_coeffs = np.zeros((4, 1), dtype=np.float32)

camera_height = 1  # meters
aruco_marker_size = 0.178  # meters
qr_height = 0  # assuming the ArUco marker is on the ground

def find_aruco(image, scale_factor, width, height):

    # Convert the image to grayscale
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    parameters = cv2.aruco.DetectorParameters()

    # Create the ArUco detector
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    # Detect the markers
    corners, ids, rejected = detector.detectMarkers(gray)

    print("Detected markers:", ids)

    position = None
    angle_deg = 0


    if ids is not None:
        ids = ids.flatten()
        for i, marker_id in enumerate(ids):
            if marker_id == 1: #Robot id should be 1
                pts = corners[i][0].astype(np.float32)
                cv2.polylines(image, [pts.astype(np.int32)], True, (0, 255, 0), 1)

                # Estimate pose
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    [pts], aruco_marker_size, camera_matrix.astype(np.float32), dist_coeffs.astype(np.float32)
                )

                rvec = rvecs[0][0]
                tvec = tvecs[0][0]

                # Compute orientation (yaw)
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                angle_rad = math.atan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
                angle_deg = (math.degrees(angle_rad) + 360) % 360

                # Compute position (real-world x, y on ground)
                position = (tvec[0] * scale_factor, tvec[2] * scale_factor)

                print(f"Marker ID: {marker_id}")
                print(f"Position: x={position[0]:.2f} m, y={position[1]:.2f} m")
                print(f"Direction: {angle_deg:.1f} degrees")

                # Draw axis
                cv2.drawFrameAxes(image, camera_matrix, dist_coeffs, rvec, tvec, 0.1)

                break  # Stop after finding ID 1
            else:
                pts = corners[i][0].astype(np.float32)
                cv2.polylines(image, [pts.astype(np.int32)], True, (0, 255, 0), 1)

    return image, scale_factor, position, angle_deg