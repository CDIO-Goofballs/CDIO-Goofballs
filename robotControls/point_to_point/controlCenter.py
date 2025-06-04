import socket
import keyboard
from typing import Any
from enum import Enum
import point
from point import Point


ev3_ip = '192.168.137.30'
port = 12345

positions = [
    Point(0,0),
    Point(200,0),
    Point(200,200),
    Point(0,200),
    Point(0,0)
]
robot_start_angle = 0 # 0 is straight up

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((ev3_ip, port))

class Command(Enum):
    DRIVE = "drive"
    TURN = "turn"
    STOP = "stop"
    SERVO= "servo"

def send_command(command: tuple[Command, Any]):
    name, val = command
    message = ','.join((name.value, str(val))) + ';'
    client_socket.send(message.encode())

def points_to_commands(robot_start_angle, positions):
    commands = []
    current_angle = robot_start_angle

    for i in range(len(positions) - 1):
        turn_angle = point.calculate_turn(positions[i], positions[i + 1], current_angle)
        commands.append((Command.TURN, turn_angle))
        current_angle += turn_angle
        distance = point.calculate_distance(positions[i], positions[i + 1])
        commands.append((Command.DRIVE, distance))
    return commands

def send_commands(commands):
    for command in commands:
        send_command(command)
#go_to_list_of_points(robot_start_angle = 0, positions = positions)


keyboard.add_hotkey('e', send_command, args=((Command.STOP, None),))
#keyboard.add_hotkey('w', send_command, args=((Command.DRIVE, 800),))
#keyboard.add_hotkey('d', send_command, args=((Command.TURN, 90),))
#keyboard.add_hotkey('a', send_command, args=((Command.TURN, -90),))
#keyboard.add_hotkey('s', send_command, args=((Command.DRIVE, -800),))
keyboard.add_hotkey('t', send_command, args=((Command.SERVO, -30),))
keyboard.add_hotkey('y', send_command, args=((Command.SERVO, 0),))
keyboard.add_hotkey('u', send_command, args=((Command.SERVO, 80),))

keyboard.wait('q')

client_socket.close()
