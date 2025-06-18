import time
from robotControls.controlCenter import connect, send_command, Command, wait_for_done
from robotControls.controls_main import collect_balls
from ImageRecognition.RoboFlow.MainImageRecognition import initialize_camera, run_image_recognition, stop_image_recognition
from Pathfinding.pathing_main import pathing
import keyboard
    
def with_robot(image=None, camera_index=0):
    if not image:
        initialize_camera(camera_index)
        
    connect()
    keyboard.wait('s')
    print("--------------Start--------------")
    keyboard.add_hotkey('e', send_command, args=((Command.STOP, None),))
    run_image_recognition(image)
    run_image_recognition(image)
    collect_balls(image)
    #run_image_recognition()y
    #if len(get_balls()) > 0 or get_vip_ball() is not None:
    #    collect_balls(image) # Run to goal
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

def test_wait_for_done():
    connect()
    print("Waiting for done...")
    send_command((Command.TEST, None))
    send_command((Command.TEST, None))
    send_command((Command.TEST, None))
    wait_for_done()
    wait_for_done()
    wait_for_done()
    print("Done received.")

if __name__ == "__main__":
    # Uncomment the line below to run with the robot and a specific camera index

    with_robot(camera_index=1)
    keyboard.wait('q')
    # Uncomment the line below to run without the robot and a specific camera index
    #no_robot(camera_index=1)