import threading
import time
import traceback

from ImageRecognition.RoboFlow.MainImageRecognition import (
    start_image_recognition, get_wall_corners, get_vip_ball, get_balls, get_cross, get_egg,
    get_small_goal, get_angle, get_position, get_scale_factor, get_size)
from Pathing.AltPathing import path_finding

def pathing():
    print("Starting pathing thread...")
    while True:
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
            else:
                print("No path found.")
        except Exception as e:
            print("Error in pathing thread:", e)
            traceback.print_exc()

# Thread for pathing
pathing_thread = threading.Thread(target=pathing, daemon=True)
pathing_thread.start()

# Thread for image recognition
#image_recognition_thread = threading.Thread(target=start_image_recognition, daemon=True)
#image_recognition_thread.start()
time.sleep(3)
start_image_recognition()

pathing()

# Keep main thread alive
while True:
    print("Test")
    time.sleep(10)
