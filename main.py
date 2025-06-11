import time
import traceback
import cv2
from robotControls.point_to_point.controlCenter import points_to_commands, send_commands, connect, send_command, Command

import cv2


from ImageRecognition.RoboFlow.MainImageRecognition import (
    initialize_camera, run_image_recognition, stop_image_recognition, get_wall_corners, get_vip_ball, get_balls, get_cross, get_egg,
    get_small_goal, get_big_goal, get_angle, get_position, get_scale_factor, get_size)
from Pathing.Pathing import path_finding
import keyboard

def pathing():
    try:
        cross = get_cross()
        egg = get_egg()
        start = get_position()
        vip = get_vip_ball()
        balls = get_balls()
        wall_corners = get_wall_corners()
        end = get_small_goal()
        width, height = get_size()

        if not end:
            end = get_big_goal()
        if end:
            end = (end[0] + (width / 2 - end[0]) / 5, end[1])

        path = path_finding(
            cross=cross,egg = egg, start=start, vip=vip, balls=balls, end=end,
            wall_corners=wall_corners, width=width, height=height)

        if path is not None:
            print("Path found:", path)
            return path
        else:
            print("No path found.")
            return []
    except Exception as e:
        print("Error in pathing thread:", e)
        traceback.print_exc()
        return []

def with_robot(image=None, camera_index=0):
    if not image:
        initialize_camera(camera_index)
    keyboard.add_hotkey('e', send_command, args=((Command.STOP, None),))
    connect()
    run_image_recognition(image)
    print("Robot start angle: ", get_angle())
    path = pathing()
    modified_path = [(10 * p[0], -10 * p[1]) for p in path]
    cmds = points_to_commands(get_angle() + 90, modified_path)
    send_command((Command.SERVO, -30),)
    send_commands(cmds)
    keyboard.wait('q')
    """
    def use_path(path):
      init_robot_angle = get_angle()
      path = pathing()
      error = 2
      if(path == None):
          print("Path is None")
          return
      while len(get_balls()) > 0: # Runs during the whole course
          dest_reached = False
          path = pathing()
          while not dest_reached: # Runs during the travel to one ball
              run_image_recognition()
              cmds = points_to_commands(get_angle(), path[0:2])
              print(cmds)
              if(abs(init_robot_angle - get_angle()) > error):
                  send_command((Command.STOP, None),) # clear queue
                  break
              send_commands(cmds)            
"""

def no_robot(camera_index=0):
  # Initialize the camera
  initialize_camera(camera_index)

  while True:
      print("--------------Start------------------")
      start_time = time.time()
      has_ended = run_image_recognition()
      print ("Time taken for image recognition:", time.time() - start_time)
      pathing()
      print ("Time taken for image recognition and pathing:", time.time() - start_time)
      print("--------------End--------------------")
      if has_ended:
          break

  stop_image_recognition()

if __name__ == "__main__":
    # Uncomment the line below to run with the robot and a specific camera index
    # with_robot(camera_index=0)

    # Uncomment the line below to run without the robot and a specific camera index
    no_robot(camera_index=0)