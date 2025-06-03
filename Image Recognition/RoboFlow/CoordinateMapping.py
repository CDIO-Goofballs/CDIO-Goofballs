import math

import cv2
import numpy as np

qr = cv2.QRCodeDetector()

# Define camera parameters (this should come from calibration or estimation)
# Example: Assuming a focal length of 800 pixels, and camera's principal point is at the image center
focal_length = 800  # Example value, replace with actual focal length
cx, cy = 320, 240  # Principal point (image center for a 640x480 image)
camera_matrix = np.array([[focal_length, 0, cx],
                          [0, focal_length, cy],
                          [0, 0, 1]])

# Distortion coefficients (assumed to be zero if unknown)
dist_coeffs = np.zeros((4, 1))

# Input parameters (real-world values)
camera_height = 1  # Height of the camera above the ground (in meters)
qr_code_size = 0.178  # Physical size of the QR code (in meters), e.g., 20 cm
qr_height = 0
def find_qr(image, scale_factor, width, height):
    # Detect QR code(s)
    retval, decoded_info, polygons, straight_qrcode = qr.detectAndDecodeMulti(image)
    world_coordinates_x = 0
    world_coordinates_y = 0
    angle_deg = 0

    i = 0
    while i < len(decoded_info):
        # Get the points that form the QR code in the image
        points = polygons[i]
        data = decoded_info[i]

        if len(points) == 4:  # Expecting a rectangular QR code
            pts = np.array(points, dtype=np.float32)

            # Find the bounding box of the QR code
            x, y, w, h = cv2.boundingRect(pts)
            qr_center = (x + w // 2, y + h // 2)

            # Compute the QR code's size in image coordinates
            qr_image_size = np.linalg.norm(pts[0] - pts[1])  # Distance between two adjacent corners

            if "erdetfredag" in data:
                # Calculate the scale factor to convert image space to real-world space
                scale_factor = qr_code_size / qr_image_size  # Ratio of real-world size to image size
            else:
                # Use the first two points to compute angle
                pt1, pt2 = pts[0], pts[1]

                # Calculate angle in radians then convert to degrees
                dx = pt2[0] - pt1[0]
                dy = pt2[1] - pt1[1]
                angle_rad = math.atan2(dy, dx)
                angle_deg = math.degrees(angle_rad)
                if angle_deg < 0.0:
                    angle_deg = 360 - angle_deg

                # Compute vertical ratio
                ratio = (camera_height - qr_height) / camera_height  # How far "down" the object is in camera view

                # Compute image center
                cx, cy = width / 2, height / 2
                pixel_vec = np.array([qr_center[0] - cx, qr_center[1] - cy])

                # Scale pixel offset toward ground
                ground_offset = pixel_vec / ratio

                # Ground position in image coordinates (projected down)
                ground_pixel = np.array([cx, cy]) + ground_offset

                # Convert to meters from pixel coordinates
                ground_position_m = ground_pixel * scale_factor

                print("Ground point beneath object (in meters):", ground_position_m)

            # Draw the detected QR code and its world coordinates on the image
            cv2.polylines(image, [pts.astype(np.int32)], isClosed=True, color=(0, 255, 0), thickness=2)

        i += 1

    return image, scale_factor, (world_coordinates_x, world_coordinates_y), angle_deg


def main():
    # Load the image (or frame from the camera)
    cam = cv2.VideoCapture(1)

    scale = 0
    while True:
        ret, imageFrame = cam.read()
        imageFrame, scale, (x,y), angle = find_qr(imageFrame, scale, 640, 480)
        if x != 0 and y != 0:
            print(x, y)
        # Display the captured frame
        cv2.imshow("QR detection", imageFrame)

        # Press 'q' to exit the loop
        if cv2.waitKey(1) == ord('q'):
            break

    cam.release()
    cv2.destroyAllWindows()

if __name__=="__main__":
    main()