import cv2
import numpy as np
import math

camera_height = 1.70  # meters
scale_aruco_size = 0.15  # meters
robot_aruco_size = 0.08  # meters
robot_aruco_height = 0.35

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

    angle_deg = 0
    position = None
    projected_x = None
    projected_y = None

    if ids is not None:
        ids = ids.flatten()
        for i, marker_id in enumerate(ids):
            if marker_id == 1:  # Robot marker
                pts = corners[i][0].astype(np.float32)
                cv2.polylines(image, [pts.astype(np.int32)], True, (0, 255, 0), 2)

                # Original 2D image center of the marker
                center_x = np.mean(pts[:, 0])
                center_y = np.mean(pts[:, 1])

                # Project marker to ground position
                relative_height = camera_height - robot_aruco_height

                # Assumed camera center in image
                cx = width / 2
                cy = height / 2

                # Projected ground position in image pixels
                projected_x = cx + (center_x - cx) * (relative_height / camera_height)
                projected_y = cy + (center_y - cy) * (relative_height / camera_height)

                # Estimate angle
                dx = pts[1][0] - pts[0][0]
                dy = pts[1][1] - pts[0][1]
                angle_rad = math.atan2(dy, dx)
                angle_deg = (math.degrees(angle_rad) + 360) % 360
                print(angle_deg)

            elif marker_id == 0: # Scale id should be 0
                pts = corners[i][0].astype(np.float32)
                cv2.polylines(image, [pts.astype(np.int32)], True, (0, 255, 0), 2)

                # Set scale factor based on the detected marker size
                scale_factor = (scale_aruco_size / cv2.norm(pts[0] - pts[1])) * 100  # Convert to cm
                print(scale_factor)

    if projected_x and projected_y:
        # Convert to real-world units
        real_x = projected_x * scale_factor
        real_y = projected_y * scale_factor
        position = (real_x, real_y)

    return image, scale_factor, position, angle_deg