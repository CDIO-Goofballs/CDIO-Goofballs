from ImageRecognition.RoboFlow.MainImageRecognition import run_image_recognition, get_vip_ball, get_balls, get_angle, get_position_mm
from Pathfinding.Point import MyPoint
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
        diff_angle = (get_angle() - target_angle + 180) % 360 - 180
        send_command((Command.TURN, -diff_angle*2/3),)
        wait_for_done()
        run_image_recognition()
        target_angle = calculate_turn(get_position_mm(), target, 0)
    
def drive_with_cam(target, drive_back=False):
    position = get_position_mm()
    targeting_ball = target.type == 'ball' or target.type == 'vip'

    offset = ROBOT_LENGTH if targeting_ball else 0
    distance = calculate_distance(position, target) - offset
    original_distance = distance

    while distance > 80:
        send_command((Command.DRIVE, distance * 3 / 4),)
        wait_for_done()
        run_image_recognition()
        position = get_position_mm()
        target_angle = calculate_turn(position, target, 0)
        if abs(get_angle() - target_angle) > 2:
            rotate_with_cam(target)
            position = get_position_mm()
        distance = calculate_distance(position, target) - offset

    if targeting_ball:
        send_command((Command.SERVO, 30),)
    send_command((Command.DRIVE, distance), )
    wait_for_done()
    if targeting_ball:
        time.sleep(4)
        send_command((Command.SERVO, 0),)
    if drive_back:
        send_command((Command.DRIVE, -original_distance), )
        wait_for_done()

def collect_ball(target, drive_back=False):
    rotate_with_cam(target=target)
    time.sleep(0.5)
    run_image_recognition()
    drive_with_cam(target, drive_back=drive_back)
    time.sleep(0.5)  # Maybe needed
    run_image_recognition()
    if target.type == 'safe':
        collect_ball(target.target, drive_back=True)

def collect_balls(image):
    run_image_recognition(image)
    while len(get_balls()) > 0 or get_vip_ball() is not None:
        path = pathing()
        if not path:
            continue
        path = path[0:2]
        print(path)
        modified_path = [MyPoint(10 * p.x, 10 * p.y, type=p.type, target=p.target) for p in path] # Convert from cm to mm
        collect_ball(modified_path[1])
        run_image_recognition(image)