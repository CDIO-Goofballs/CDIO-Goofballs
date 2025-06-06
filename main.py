import time
import traceback
import cv2
from robotControls.point_to_point.controlCenter import points_to_commands, send_commands, connect

from ImageRecognition.RoboFlow.MainImageRecognition import (
    initialize_camera, run_image_recognition, stop_image_recognition, get_wall_corners, get_vip_ball, get_balls, get_cross, get_egg,
    get_small_goal, get_angle, get_position, get_scale_factor, get_size)
from Pathing.AltPathing import path_finding
import keyboard

def pathing():
    try:
        cross = get_cross()
        start = get_position()
        vip = get_vip_ball()
        balls = get_balls()
        wall_corners = get_wall_corners()
        end = get_small_goal()
        width, height = get_size()

        path = path_finding(cross=cross, start=start, vip=vip, balls=balls, end=end, wall_corners=wall_corners, width=width, height=height)
        if path is not None:
            print("Path found:", path)
            return path
        else:
            print("No path found.")
    except Exception as e:
        print("Error in pathing thread:", e)
        traceback.print_exc()


# Initialize the camera
#initialize_camera()

#while True:
print("--------------Start------------------")
connect()
start_time = time.time()
img = cv2.imread('test_image.png')
run_image_recognition(imageFrame=img)
print ("Time taken for image recognition:", time.time() - start_time)
path = pathing()
cmds = points_to_commands(0, path)
print(cmds)
send_commands(cmds)
print ("Time taken for image recognition and pathing:", time.time() - start_time)
print("--------------End--------------------")
keyboard.wait('q')
