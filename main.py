import time
import traceback
import cv2
from robotControls.controlCenter import points_to_commands, send_commands, connect, send_command, Command, calculate_distance, calculate_turn, wait_for_done

import cv2


from ImageRecognition.RoboFlow.MainImageRecognition import (
    initialize_camera, run_image_recognition, stop_image_recognition, get_wall_corners, get_vip_ball, get_balls, get_cross, get_egg,
    get_small_goal, get_big_goal, get_angle, get_position, get_scale_factor, get_size)
from Pathfinding.Pathing import path_finding
import keyboard

ROBOT_LENGTH = 270

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

def get_difference_in_position(robot_position, new_robot_position):
    if robot_position is None or new_robot_position is None:
        return -1
    return abs(calculate_distance(robot_position, new_robot_position))
    
def get_difference_in_angle(robot_angle, new_robot_angle):
    if robot_angle is None or new_robot_angle is None:
        return -1
    return abs(robot_angle - new_robot_angle)
    
def with_robot(image=None, camera_index=0):
    if not image:
        initialize_camera(camera_index)
        
    connect()
    keyboard.add_hotkey('e', send_command, args=((Command.STOP, None),))
    run_image_recognition(image)
    run_image_recognition(image)
    #rotate_with_cam(turning_angle=calculate_turn((0,0), (-1,0), orientation=get_angle()+90), target_angle=-90)
    collect_balls(image) # Run until all balls is collected
    
    #run_image_recognition()
    #if len(get_balls()) > 0 or get_vip_ball() is not None:
    #    collect_balls(image) # Run to goal
    print("------------DONE--------------")

def rotate_with_cam(turning_angle, target_angle):
    print("Robot angle: ", get_angle())
    print("Target angle: ", target_angle)
    print("Turning angle: ", turning_angle)
    if turning_angle > 0:
        #turn right
        print("Turning left")
        send_command((Command.TURN, turning_angle),)
    else: 
        #turn left
        print("Turning right")
        send_command((Command.TURN, turning_angle),)
    wait_for_done()
    
    run_image_recognition()
    while abs(get_angle() - target_angle) > 1.5:
        #continue turning
        diff_angle = (get_angle() - target_angle + 180) % 360 - 180
        print("Angle diff: ", diff_angle)
        if diff_angle < 0:
            send_command((Command.TURN, -diff_angle/2),)
        else:
            send_command((Command.TURN, -diff_angle/2),)    
        wait_for_done()
        run_image_recognition()
    send_command((Command.STOP, None))
    
def drive_with_cam(robot_pos, target, angle):
    send_command((Command.DRIVE, calculate_distance(robot_pos, target) - ROBOT_LENGTH),)
    run_image_recognition()
    while abs(get_angle() - angle) < 1:
        run_image_recognition()
    send_command((Command.STOP, None),)

def collect_balls(image):
    offset = 90
    robot_position = None
    robot_angle = None
    position_error = 2
    angle_error = 2
    
    run_image_recognition(image)
    while len(get_balls()) > 0 or get_vip_ball() is not None:
        new_robot_position = get_position()
        new_robot_angle = get_angle()
        pos_diff = get_difference_in_position(robot_position, new_robot_position)
        angle_diff = get_difference_in_angle(robot_angle, new_robot_angle)
        if not pos_diff - position_error > 0 or not angle_diff - angle_error > 0:
            print("Robot is still! New path...")
            path = pathing()
            if not path:
                continue
            path = path[0:2]
            print(path)
            modified_path = [(10 * p.x, -10 * p.y) for p in path] # Convert from cm to mm
            turning_angle = calculate_turn(modified_path[0], modified_path[1], get_angle())
            target_angle = calculate_turn(modified_path[0], modified_path[1], 0)
            rotate_with_cam(turning_angle=turning_angle, target_angle=target_angle)
            #drive_with_cam(new_robot_position, modified_path[1], angle=target_angle)
            time.sleep(0.5) # Maybe needed
            break
        run_image_recognition(image)
            
        


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
    
    with_robot(camera_index=1)
    keyboard.wait('q')
    # Uncomment the line below to run without the robot and a specific camera index
    #no_robot(camera_index=0)