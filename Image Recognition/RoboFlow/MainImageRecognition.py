import cv2
from ColorRecognition import color_recognition
from RoboFlow import object_recognition

cam = cv2.VideoCapture(0)

# Get the default frame width and height
frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

while True:
    ret, imageFrame = cam.read()

    imageFrame, latest_angle, latest_position, distance = color_recognition(imageFrame)
    imageFrame, ratio = object_recognition(imageFrame)

    if distance > 0:
        print(distance * ratio)

    # Display the captured frame
    cv2.imshow("Image Recognition", imageFrame)

    # Press 'q' to exit the loop
    if cv2.waitKey(1) == ord('q'):
        break

def get_angle():
    return latest_angle
def get_position():
    return latest_position
def get_ratio():
    return ratio # pixels * ratio = cm

# Release the capture and writer objects
cam.release()
cv2.destroyAllWindows()

