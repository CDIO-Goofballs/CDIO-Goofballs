import time
import traceback
from robotControls.controlCenter import connect, send_command, Command, calculate_distance, calculate_turn, wait_for_done


from ImageRecognition.RoboFlow.MainImageRecognition import (
    initialize_camera, run_image_recognition, stop_image_recognition, get_wall_corners, get_vip_ball, get_balls, get_cross, get_egg,
    get_small_goal, get_big_goal, get_angle, get_position, get_size, get_position_mm)
from Pathfinding.Pathing import path_finding
import keyboard

ROBOT_LENGTH = 245

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
        start_angle = get_angle()

        if not end:
            end = get_big_goal()
        if end:
            end = (end[0] + (width / 2 - end[0]) / 5, end[1])

        path = path_finding(
            cross=cross,egg = egg, start=start, vip=vip, balls=balls, end=end,
            wall_corners=wall_corners, width=width, height=height, start_angle=start_angle,)

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
    collect_balls(image)
    #run_image_recognition()
    #if len(get_balls()) > 0 or get_vip_ball() is not None:
    #    collect_balls(image) # Run to goal
    print("------------DONE--------------")

def rotate_with_cam(target):
    print(get_angle())
    turning_angle = calculate_turn(get_position_mm(), target, get_angle())
    target_angle = calculate_turn(get_position_mm(), target, 0)
    send_command((Command.TURN, turning_angle),)
    wait_for_done()
    run_image_recognition()
    while abs(get_angle() - target_angle) > 1:
        print("Robot angle: ", get_angle())
        diff_angle = (get_angle() - target_angle + 180) % 360 - 180
        print("Diff angle: ", diff_angle)
        send_command((Command.TURN, -diff_angle*2/3),)    
        wait_for_done()
        run_image_recognition()
        target_angle = calculate_turn(get_position_mm(), target, 0)
    
def drive_with_cam(target):
    position = get_position_mm()
    angle = get_angle()
    print("Drive distance: ", calculate_distance(position, target) - ROBOT_LENGTH)
    distance = calculate_distance(position, target) - ROBOT_LENGTH

    while distance > 80:
        send_command((Command.DRIVE, distance * 3 / 4),)
        wait_for_done()
        run_image_recognition()
        position = get_position_mm()
        target_angle = calculate_turn(position, target, 0)
        if abs(get_angle() - target_angle) > 2:
            print("Off course, correcting...")
            rotate_with_cam(target)
            position = get_position_mm()
        distance = calculate_distance(position, target) - ROBOT_LENGTH

    send_command((Command.SERVO, 30),)
    send_command((Command.DRIVE, distance), )
    time.sleep(4)
    send_command((Command.SERVO, 0),)

def collect_balls(image):
    robot_position = None
    robot_angle = None
    position_error = 2
    angle_error = 2
    
    run_image_recognition(image)
    while len(get_balls()) > 0 or get_vip_ball() is not None:
        new_robot_position = get_position_mm()
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
            modified_path = [(10 * p.x, 10 * p.y) for p in path] # Convert from cm to mm
            turning_angle = calculate_turn(modified_path[0], modified_path[1], get_angle())
            target_angle = calculate_turn(modified_path[0], modified_path[1], 0)
            rotate_with_cam(target=modified_path[1])
            time.sleep(0.5)
            run_image_recognition()
            path = pathing()
            drive_with_cam(modified_path[1])
            print("Collected ball")
            time.sleep(0.5) # Maybe needed
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
    #no_robot(camera_index=1)


# TODO change robot speed when close to the ball