import time

from ImageRecognition.RoboFlow.MainImageRecognition import run_image_recognition, get_angle, get_position_mm
from Pathfinding.Pathing import get_safe_points
from Pathfinding.Point import MyPoint, calculate_distance, calculate_turn
from Pathfinding.pathing_main import pathing
from robotControls.controlCenter import send_command, Command

ROBOT_LENGTH = 250


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
    run_image_recognition()
    while abs(get_angle() - target_angle) > 1.8:
        diff_angle = (get_angle() - target_angle + 180) % 360 - 180
        send_command((Command.TURN, -diff_angle * 3 / 4), )
        run_image_recognition()
        target_angle = calculate_turn(get_position_mm(), target, 0)


def drive_with_cam(target, drive_back=False):
    position = get_position_mm()
    targeting_ball = target.type == 'ball' or target.type == 'vip'

    offset = ROBOT_LENGTH if targeting_ball else 50
    distance = calculate_distance(position, target) - offset
    original_distance = distance

    print("Distance to target:", distance, "mm")
    slow_zone = 80 if targeting_ball else 280
    slow_speed = 20 if targeting_ball else 35
    off_course_angle = 2 if targeting_ball else 6
    if target.type == 'end':
        offset = 60

    if (target.type == 'safe' and target.target.type == 'end') or target.type == 'end':
        slow_zone = 180
        slow_speed = 20
        off_course_angle = 3

    while distance > slow_zone:
        send_command((Command.DRIVE, (distance * 2 / 3, 50), ))
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
    send_command((Command.DRIVE, (distance, slow_speed)), )
    if targeting_ball:
        time.sleep(1)
    if drive_back:
        drive_back_extra_distance = -30 if target.type == 'safeV3' else 80
        send_command((Command.DRIVE, (-(abs(original_distance) + drive_back_extra_distance), 40)), )

def drive_to_target(target, drive_back=False):
    skip_target = False
    if target.type == 'turn' or 'safe' in target.type:
        if calculate_distance(get_position_mm(), target) < 60:
            skip_target = True

    if not skip_target:
        rotate_with_cam(target=target)
        run_image_recognition()
        drive_with_cam(target, drive_back=drive_back)

    run_image_recognition()
    if 'safe' in target.type:
        print("Arrived at safe point, proceeding to target")
        print("Target:", target.target)
        if target.target.type == 'end': send_command((Command.SERVO, 0), )
        drive_to_target(target.target, drive_back=target.target.type != 'end')

def more_balls_left():
    send_command((Command.DRIVE, (300, 50)), )
    run_image_recognition()
    path = pathing()
    if not path:
        return False
    if path[1].type == 'safe':
        return path[1].target.type != 'end'
    return True

def collect_balls(image):
    global ROBOT_LENGTH
    run_image_recognition(image)
    end_reached = False
    last_path = None
    while not end_reached:
        path = pathing()
        if not path:
            safe_points = get_safe_points()
            modified_safe_points = [MyPoint(10 * pt.x, 10 * pt.y, type='turn') for pt in safe_points]  # Convert from cm to mm
            modified_safe_points.sort(key=lambda x: calculate_distance(get_position_mm(), x))

            drive_to_target(modified_safe_points[0])  # Convert from cm to mm
            run_image_recognition(image)
            continue
        # Find first subpath in the path that ends with a ball or VIP
        sub_path = []
        for i in range(1, len(path)):
            if path[i].type != 'turn':
                sub_path = path[:i + 1]
                break
        modified_path = [MyPoint(10 * p.x, 10 * p.y, type=p.type, target=p.target) for p in sub_path] # Convert from cm to mm
        for p in modified_path:
            if 'safe' in p.type:
                p.target = MyPoint(10 * p.target.x, 10 * p.target.y, type=p.target.type)

        print("Modified path: ", modified_path)

        if 'safe' in path[1].type:
            if last_path and 'safe' in last_path[1].type:
                if calculate_distance(last_path[1], modified_path[1]) < 20 and calculate_distance(
                        last_path[1].target, modified_path[1].target) < 20:
                    ROBOT_LENGTH = 240
                    print("Same path, drive longer")

        for point in modified_path[1:]:
            drive_to_target(point)
            run_image_recognition(image)
        if modified_path[-1].type == 'safe':
            if modified_path[-1].target.type == 'end':
                print("End has been reached")
                end_reached = True

        last_path = modified_path
        ROBOT_LENGTH = 250

    position = get_position_mm()
    # Rotate to be perpendicular to the wall
    safe_points = get_safe_points()
    safe_points.sort(key=lambda x: calculate_distance(position, x))
    points_to_use = safe_points[:2]
    # Sort points by their x-coordinate according to get_angle() > 0
    points_to_use.sort(key=lambda x: x.x if get_angle() > 0 else -x.x)
    v = points_to_use[1] - points_to_use[0]
    rotate_with_cam(target=MyPoint(position.x + v.x, position.y + v.y))
    send_command((Command.SERVO, -100), )
    time.sleep(4)
    send_command((Command.DRIVE, (-50, 10)), )
    time.sleep(4)
    send_command((Command.SERVO, (0, False)), )