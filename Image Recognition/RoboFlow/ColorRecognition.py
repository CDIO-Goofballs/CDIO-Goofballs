import cv2
from math import atan2, cos, sin, sqrt, pi
import numpy as np

cam = cv2.VideoCapture(0)

# Get the default frame width and height
frame_width = int(cam.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cam.get(cv2.CAP_PROP_FRAME_HEIGHT))

def drawAxis(img, p_, q_, colour, scale):
    p = list(p_)
    q = list(q_)

    angle = atan2(p[1] - q[1], p[0] - q[0])  # angle in radians
    hypotenuse = sqrt((p[1] - q[1]) * (p[1] - q[1]) + (p[0] - q[0]) * (p[0] - q[0]))

    # Here we lengthen the arrow by a factor of scale
    q[0] = p[0] - scale * hypotenuse * cos(angle)
    q[1] = p[1] - scale * hypotenuse * sin(angle)
    cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv2.LINE_AA)

    # create the arrow hooks
    p[0] = q[0] + 9 * cos(angle + pi / 4)
    p[1] = q[1] + 9 * sin(angle + pi / 4)
    cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv2.LINE_AA)

    p[0] = q[0] + 9 * cos(angle - pi / 4)
    p[1] = q[1] + 9 * sin(angle - pi / 4)
    cv2.line(img, (int(p[0]), int(p[1])), (int(q[0]), int(q[1])), colour, 1, cv2.LINE_AA)


while True:
    ret, imageFrame = cam.read()

    middle_blue = []
    middle_green = []

    # Convert BGR to HSV colorspace
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)

    # Set range for green color
    green_lower = np.array([45,60,60], np.uint8)
    green_upper = np.array([80,245,245], np.uint8)

    # define mask
    green_mask = cv2.inRange(hsvFrame, green_lower, green_upper)

    # blue color
    blue_lower = np.array([98, 80, 80], np.uint8)
    blue_upper = np.array([139, 255, 255], np.uint8)
    #blue_lower = np.array([94, 80, 2], np.uint8)
    #blue_upper = np.array([120, 255, 255], np.uint8)
    blue_mask = cv2.inRange(hsvFrame, blue_lower, blue_upper)


    # to detect only that particular color
    kernal = np.ones((5, 5), "uint8")

    # green color
    green_mask = cv2.dilate(green_mask, kernal)
    res_green = cv2.bitwise_and(imageFrame, imageFrame, mask=green_mask)

    # blue color
    blue_mask = cv2.dilate(blue_mask, kernal)
    res_blue = cv2.bitwise_and(imageFrame, imageFrame, mask=blue_mask)

    # Creating contour to track green color
    contours, hierarchy = cv2.findContours(green_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)

    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if area > 100:
            x, y, w, h = cv2.boundingRect(contour)

            # Send x + w/2 and y + h/2 to path finding program
            middle_green.append((x+w/2, y+h/2))
            imageFrame = cv2.circle(imageFrame, (int(x + w / 2), int(y + h / 2)),
                                    5,
                                    (0, 255, 0), 2)

            imageFrame = cv2.rectangle(imageFrame, (x, y),
                                       (x + w, y + h),
                                       (0, 255, 0), 2)

            cv2.putText(imageFrame, "Green Colour", (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, (0, 255, 0))

    # Creating contour to track blue color
    contours, hierarchy = cv2.findContours(blue_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if area > 100:
            x, y, w, h = cv2.boundingRect(contour)

            # Send x + w/2 and y + h/2 to path finding program
            middle_blue.append((x+w/2, y+h/2))

            imageFrame = cv2.circle(imageFrame, (int(x + w / 2), int(y + h / 2)),
                                    5,
                                    (255, 0, 0), 2)

            imageFrame = cv2.rectangle(imageFrame, (x, y),
                                       (x + w, y + h),
                                       (255, 0, 0), 2)

            cv2.putText(imageFrame, "Blue Colour", (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0, (255, 0, 0))

    print(middle_blue, middle_green)
    if len(middle_green) != 0 and len(middle_blue) != 0:
        for (x,y) in middle_green:
            for (dx,dy) in middle_blue:
                drawAxis(imageFrame, (x,y), (dx,dy), (0, 255, 0), 1)

        # get angle from vector

    # Display the captured frame
    cv2.imshow("Color Detection", imageFrame)

    # Press 'q' to exit the loop
    if cv2.waitKey(1) == ord('q'):
        break

# Release the capture and writer objects
cam.release()
cv2.destroyAllWindows()