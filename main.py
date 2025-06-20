import time

import keyboard

from ImageRecognition.RoboFlow.MainImageRecognition import initialize_camera, run_image_recognition, \
    stop_image_recognition
from Pathfinding.pathing_main import pathing
from robotControls.controlCenter import connect, send_command, Command
from robotControls.controls_main import collect_balls, more_balls_left


def with_robot(image=None, camera_index=0):
    if not image:
        initialize_camera(camera_index)
        
    connect()
    keyboard.wait('s')
    print("--------------Start--------------")
    start_time = time.time()
    keyboard.add_hotkey('e', send_command, args=((Command.STOP, None),))
    collect_balls(image)
    while more_balls_left():
        collect_balls(image)
    print("Time taken for completing course: ", time.time() - start_time)
    print("--------------DONE--------------")



def no_robot(camera_index=0):
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
    with_robot(camera_index = 1)
    #no_robot(camera_index = 1)