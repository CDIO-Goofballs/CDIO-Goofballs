import cv2
import numpy as np
import math

# Camera parameters (same as before)
focal_length = 800
cx, cy = 320, 240

camera_matrix = np.array([
    [focal_length, 0, cx],
    [0, focal_length, cy],
    [0, 0, 1]
], dtype=np.float32)

dist_coeffs = np.zeros((4, 1), dtype=np.float32)

camera_height = 1.70  # meters
scale_aruco_size = 0.15  # meters
robot_aruco_size = 0.08  # meters
robot_aruco_height = 0.4

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
            if marker_id == 1:  # Robot marker
                pts = corners[i][0].astype(np.float32)
                cv2.polylines(image, [pts.astype(np.int32)], True, (0, 255, 0), 2)
                center_x = np.mean(pts[:, 0])
                center_y = np.mean(pts[:, 1])

                # Scale factor is from earlier detected scale marker
                # position in real world (in meters or cm)
                real_x = center_x * scale_factor
                real_y = center_y * scale_factor

                # Estimate angle
                dx = pts[1][0] - pts[0][0]
                dy = pts[1][1] - pts[0][1]
                angle_rad = math.atan2(dy, dx)
                angle_deg = (math.degrees(angle_rad) + 360) % 360

                position = (real_x, real_y)
            elif marker_id == 0: # Scale id should be 0
                pts = corners[i][0].astype(np.float32)
                cv2.polylines(image, [pts.astype(np.int32)], True, (0, 255, 0), 2)

                # Set scale factor based on the detected marker size
                scale_factor = (scale_aruco_size / cv2.norm(pts[0] - pts[1])) * 100  # Convert to cm
                print(scale_factor)

    return image, scale_factor, position, angle_deg