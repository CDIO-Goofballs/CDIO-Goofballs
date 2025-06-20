import cv2

from ImageRecognition.RoboFlow.CoordinateMapping import find_aruco
from ImageRecognition.RoboFlow.RoboFlow import object_recognition
from Pathfinding.Point import MyPoint

scale_factor = 0.33
latest_position = None
latest_angle = 0
balls = []
vip_ball = None
wall_corners = None
egg = None
cross = None
small_goal = None
big_goal = None

frame_width = 640  # Default width, can be adjusted
frame_height = 480  # Default height, can be adjusted

def initialize_camera(camera_index=0):
    global cam, frame_width, frame_height
    cam = cv2.VideoCapture(camera_index)

    # Get the default frame width and height
    frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

def run_image_recognition(imageFrame=None):
    global scale_factor, latest_position, latest_angle, balls, vip_ball, wall_corners, egg, cross, small_goal, big_goal, frame_height, frame_width

    if imageFrame is None:
        ret, imageFrame = cam.read()

    imageFrame, scale_factor, latest_position, latest_angle = find_aruco(imageFrame, scale_factor, frame_width,
                                                                      frame_height)
    imageFrame, balls, vip_ball, wall_corners, cross, egg, small_goal, big_goal = object_recognition(imageFrame, scale_factor)

    # Display the captured frame
    cv2.imshow("ImageRecognition", imageFrame)
    return cv2.waitKey(1) & 0xFF == ord('q')

def stop_image_recognition():
    # Release the capture and writer objects
    try:
        cam.release()
    except NameError as e:
        pass
    cv2.destroyAllWindows()

# Function to get the latest angle, position, and other recognized objects
def get_angle():
    """
    Returns the latest angle detected by the QR code.
    :return: 0 if the robot QR code isn't detected, otherwise the angle in degrees.
    """
    return (latest_angle + 90) % 360 - 180
def get_position():
    """
    Returns the latest position detected by the QR code.
    :return: None if the robot QR code isn't detected, otherwise a tuple (x, y) representing the position.
    """
    return latest_position
def get_position_mm():
    if not latest_position:
        return MyPoint(0,0)
    return MyPoint(latest_position[0] * 10, latest_position[1] * 10)
def get_scale_factor():
    """
    Returns the scale factor used for scaling the coordinates of detected objects.
    :return: 1 if the scale QR code isn't detected, otherwise the scale factor.
    """
    return scale_factor
def get_balls():
    """
    Returns the list of detected balls.
    :return: Empty list if no balls are detected, otherwise a list of tuples (x, y) representing the positions of the balls.
    """
    return balls
def get_vip_ball():
    """
    Returns the VIP ball position.
    :return: None if no VIP ball is detected, otherwise a tuple (x, y) representing the position of the VIP ball.
    """
    return vip_ball
def get_wall_corners():
    """
    Returns the list of detected wall corners.
    :return: None if no walls are detected, otherwise the corners as (top_left, bottom_left, bottom_right, top_right).
    """
    return wall_corners
def get_cross():
    """
    Returns the cross position.
    :return: None if no cross is detected, otherwise a tuple containing the top, bottom, right, and left points of the cross in that order.
    """
    return cross
def get_egg():
    """
    Returns the egg position.
    :return: None if no egg is detected, otherwise a tuple (x, y) representing the position of the egg.
    """
    return egg
def get_small_goal():
    """
    Returns the small goal position.
    :return: None if no small goal is detected, otherwise a tuple (x, y) representing the position of the small goal.
    """
    return small_goal
def get_big_goal():
    """
    Returns the big goal position.
    :return: None if no big goal is detected, otherwise a tuple (x, y) representing the position of the big goal.
    """
    return big_goal
def get_size():
    """
    Returns the size of the image frame used for scaling.
    :return:
    """
    return frame_width * scale_factor, frame_height * scale_factor  # Assuming a default size of 640x480