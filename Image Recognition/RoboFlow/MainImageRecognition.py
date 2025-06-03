import cv2
from ColorRecognition import color_recognition
from RoboFlow import object_recognition
from CoordinateMapping import find_qr

cam = cv2.VideoCapture(1)

# Get the default frame width and height
frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

print(frame_width, frame_height)

scale_factor = 1
latest_position = None
latest_angle = 0
balls = []
vip_ball = None
walls = []
egg = None
cross = None
small_goal = None

while True:
    ret, imageFrame = cam.read()

    print('--------------Start------------------')

    imageFrame, scale_factor, latest_position, latest_angle = find_qr(imageFrame, scale_factor, frame_width, frame_height)
    imageFrame, balls, vip_ball, walls, cross, egg, small_goal = object_recognition(imageFrame, scale_factor)

    print('--------------End--------------------\n')

    # Display the captured frame
    cv2.imshow("Image Recognition", imageFrame)

    # Press 'q' to exit the loop
    if cv2.waitKey(1) == ord('q'):
        break

# Function to get the latest angle, position, and other recognized objects
def get_angle():
    """
    Returns the latest angle detected by the QR code.
    :return: 0 if the robot QR code isn't detected, otherwise the angle in degrees.
    """
    return latest_angle
def get_position():
    """
    Returns the latest position detected by the QR code.
    :return: None if the robot QR code isn't detected, otherwise a tuple (x, y) representing the position.
    """
    return latest_position
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
def get_walls():
    """
    Returns the list of detected walls.
    TODO
    """
    return walls
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
def small_goal():
    """
    Returns the small goal position.
    :return: None if no small goal is detected, otherwise a tuple (x, y) representing the position of the small goal.
    """
    return small_goal

# Release the capture and writer objects
cam.release()
cv2.destroyAllWindows()

