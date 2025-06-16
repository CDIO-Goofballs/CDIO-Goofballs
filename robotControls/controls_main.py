from ImageRecognition.RoboFlow.MainImageRecognition import run_image_recognition, get_vip_ball, get_balls, get_angle, get_position_mm
from robotControls.controlCenter import send_command, Command, calculate_distance, calculate_turn, wait_for_done
from Pathfinding.pathing_main import pathing
import time

ROBOT_LENGTH = 245

def get_difference_in_position(robot_position, new_robot_position):
    if robot_position is None or new_robot_position is None:
        return -1
    return abs(calculate_distance(robot_position, new_robot_position))
    
def get_difference_in_angle(robot_angle, new_robot_angle):
    if robot_angle is None or new_robot_angle is None:
        return -1
    return abs(robot_angle - new_robot_angle)

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
            rotate_with_cam(target=modified_path[1])
            time.sleep(0.5)
            run_image_recognition()
            path = pathing()
            drive_with_cam(modified_path[1])
            print("Collected ball")
            time.sleep(0.5) # Maybe needed
        run_image_recognition(image)