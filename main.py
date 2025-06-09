import time
import traceback

import cv2
import numpy as np

from ImageRecognition.RoboFlow.MainImageRecognition import (
    initialize_camera, run_image_recognition, stop_image_recognition, get_wall_corners, get_vip_ball, get_balls, get_cross, get_egg,
    get_small_goal, get_angle, get_position, get_scale_factor, get_size)
from Pathing.AltPathing import path_finding

def pathing():
    try:
        cross = get_cross()
        start = get_position()
        vip = get_vip_ball()
        balls = get_balls()
        wall_corners = get_wall_corners()
        end = get_small_goal()
        width, height = get_size()

        end[1] += (width / 2 - end[1]) / 5

        path = path_finding(cross=cross, start=start, vip=vip, balls=balls, end=end, wall_corners=wall_corners, width=width, height=height)
        if path is not None:
            print("Path found:", path)
        else:
            print("No path found.")
    except Exception as e:
        print("Error in pathing thread:", e)
        traceback.print_exc()

# Initialize the camera
initialize_camera()

while True:
    print("--------------Start------------------")
    start_time = time.time()
    run_image_recognition()
    print ("Time taken for image recognition:", time.time() - start_time)
    pathing()
    print ("Time taken for image recognition and pathing:", time.time() - start_time)
    print("--------------End--------------------")
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

stop_image_recognition()