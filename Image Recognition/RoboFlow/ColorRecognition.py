import cv2
from math import atan2, cos, sin, sqrt, pi, degrees
import numpy as np


def angle_between(a, b):
    angle = degrees(atan2(a[1] - b[1], b[0] - a[0]))
    if angle < 0:
        angle += 360
    return angle

def draw_axis(img, p_, q_, colour, scale):
    p = list(p_) #green
    q = list(q_) #purple

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


def color_recognition(img):
    middle_purple = []
    middle_green = []

    # Convert BGR to HSV colorspace
    hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Set range and mask for green color
    green_lower = np.array([45,60,60], np.uint8)
    green_upper = np.array([80,245,245], np.uint8)
    green_mask = cv2.inRange(hsv_frame, green_lower, green_upper)

    # purple color
    purple_lower = np.array([130, 60, 60], np.uint8)
    purple_upper = np.array([155, 255, 255], np.uint8)
    purple_mask = cv2.inRange(hsv_frame, purple_lower, purple_upper)

    # to detect only that particular color
    kernal = np.ones((5, 5), "uint8")

    # green color
    green_mask = cv2.dilate(green_mask, kernal)
    res_green = cv2.bitwise_and(img, img, mask=green_mask)

    # purple color
    purple_mask = cv2.dilate(purple_mask, kernal)
    res_purple = cv2.bitwise_and(img, img, mask=purple_mask)

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
            img = cv2.circle(img, (int(x + w / 2), int(y + h / 2)),
                             1,
                             (0, 255, 0), 1)

            img = cv2.rectangle(img, (x, y),
                                (x + w, y + h),
                                (0, 255, 0), 2)

            cv2.putText(img, "Green Colour", (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (0, 255, 0))

    # Creating contour to track purple color
    contours, hierarchy = cv2.findContours(purple_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)
    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if area > 100:
            x, y, w, h = cv2.boundingRect(contour)

            # Send x + w/2 and y + h/2 to path finding program
            middle_purple.append((x + w / 2, y + h / 2))

            img = cv2.circle(img, (int(x + w / 2), int(y + h / 2)),
                                    1,
                                    (255, 0, 255), 1)

            img = cv2.rectangle(img, (x, y),
                                (x + w, y + h),
                                (255, 0, 255), 2)

            cv2.putText(img, "Purple Colour", (x, y),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7, (255, 0, 255))

    latest_angle = 0
    #print(middle_purple, middle_green)
    if len(middle_green) != 0 and len(middle_purple) != 0:
        for (x,y) in middle_green:
            for (dx,dy) in middle_purple:
                dist = sqrt(pow(x-dx, 2) + pow(y-dy, 2))
                if dist > 100:
                    continue
                latest_angle = angle_between((x,y), (dx,dy))
                draw_axis(img, (x, y), (dx, dy), (255, 0, 0), 1)

    return img, latest_angle
