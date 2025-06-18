from ImageRecognition.RoboFlow.MainImageRecognition import run_image_recognition, get_vip_ball, get_balls, get_angle, get_position_mm
from Pathfinding.Point import MyPoint
from robotControls.controlCenter import send_command, Command, calculate_distance, calculate_turn, wait_for_done
from Pathfinding.pathing_main import pathing
import time

ROBOT_LENGTH = 240


def get_difference_in_position(robot_position, new_robot_position):
    if robot_position is None or new_robot_position is None:
        return -1
    return abs(calculate_distance(robot_position, new_robot_position))


def get_difference_in_angle(robot_angle, new_robot_angle):
    if robot_angle is None or new_robot_angle is None:
        return -1
    return abs(robot_angle - new_robot_angle)


def rotate_with_cam(target):
    turning_angle = calculate_turn(get_position_mm(), target, get_angle())
    target_angle = calculate_turn(get_position_mm(), target, 0)
    send_command((Command.TURN, turning_angle), )
    wait_for_done()
    run_image_recognition()
    while abs(get_angle() - target_angle) > 1.8:
        diff_angle = (get_angle() - target_angle + 180) % 360 - 180
        send_command((Command.TURN, -diff_angle * 2 / 3), )
        wait_for_done()
        run_image_recognition()
        target_angle = calculate_turn(get_position_mm(), target, 0)

def boogie_woogie():
    send_command((Command.TURN, -10), )
    send_command((Command.TURN, 20), )
    send_command((Command.TURN, -10), )


def drive_with_cam(target, drive_back=False):
    position = get_position_mm()
    targeting_ball = target.type == 'ball' or target.type == 'vip'

    offset = ROBOT_LENGTH if targeting_ball else 50
    if target.type == 'end':
        offset = 80
    distance = calculate_distance(position, target) - offset
    original_distance = distance

    print("Distance to target:", distance, "mm")
    slow_zone = 80 if targeting_ball else 280
    slow_speed = 20 if targeting_ball else 35
    off_course_angle = 2 if targeting_ball else 6

    while distance > slow_zone:
        send_command((Command.DRIVE, (distance * 2 / 3, 50), ))
        wait_for_done()
        run_image_recognition()
        run_image_recognition()
        position = get_position_mm()
        target_angle = calculate_turn(position, target, 0)
        distance = calculate_distance(position, target) - offset
        if not targeting_ball and distance < 80:
            print("Distance is less than 80mm, stopping")
            break
        if abs(get_angle() - target_angle) > off_course_angle:
            rotate_with_cam(target)
            position = get_position_mm()
        distance = calculate_distance(position, target) - offset
        print("Distance to target:", distance, "mm")

    if targeting_ball:
        send_command((Command.SERVO, 35),)
        wait_for_done()
    send_command((Command.DRIVE, (distance, slow_speed)), )
    wait_for_done()
    if targeting_ball:
        time.sleep(2)
        boogie_woogie()
    if drive_back:
        send_command((Command.DRIVE, (-original_distance*1.3, 40)), )
        wait_for_done()

def drive_to_target(target, drive_back=False):
    rotate_with_cam(target=target)
    time.sleep(0.5)
    run_image_recognition()
    drive_with_cam(target, drive_back=drive_back)
    time.sleep(0.5)  # Maybe needed
    run_image_recognition()
    if target.type == 'safeV1' or target.type == 'safeV2':
        print("Arrived at safe point, proceeding to target")
        print("Target:", target.target)
        drive_to_target(target.target, drive_back=target.target.type != 'end')

def collect_balls(image):
    run_image_recognition(image)
    end_reached = False
    while not end_reached:
        path = pathing()
        if not path:
            continue
        # Find first subpath in the path that ends with a ball or VIP
        sub_path = []
        for i in range(1, len(path)):
            if path[i].type != 'turn':
                sub_path = path[:i + 1]
                break
        modified_path = [MyPoint(10 * p.x, 10 * p.y, type=p.type, target=p.target) for p in sub_path] # Convert from cm to mm
        for p in modified_path:
            if p.type == 'safeV1' or p.type == 'safeV2':
                p.target = MyPoint(10 * p.target.x, 10 * p.target.y, type=p.target.type)

        print("Modified path: ", modified_path)
        for point in modified_path[1:]:
            drive_to_target(point)
            run_image_recognition(image)
        if modified_path[-1].type == 'safeV1' or modified_path[-1].type == 'safeV2':
            if modified_path[-1].target.type == 'end':
                print("End has been reached")
                end_reached = True


    send_command((Command.SERVO, -100), )
    wait_for_done()
    time.sleep(6)
    send_command((Command.SERVO, 0), )
    wait_for_done()