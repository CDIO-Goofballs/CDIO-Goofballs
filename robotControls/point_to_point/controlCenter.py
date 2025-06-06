import socket
import keyboard
from typing import Any
from enum import Enum
import point
from point import Point
import time

positions = [
    Point(0,0),
    Point(1000,0),
    Point(1000,1000),
    Point(0,1000),
    Point(0,0)
]

"""positions = [
    Point(0,0),
    Point(1000,1000),
    Point(1600,0),
    Point(1600,-1200),
    Point(400,1000),
    Point(0,0)
]"""
robot_start_angle = 90 # 0 is towards positive on the x-axis

# Setup server socket
port = 12345
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind(('', port))
server_socket.listen(1)

print("Waiting for a client to connect...")
conn, addr = server_socket.accept()
print(f"Connected by {addr}")


def reconnect():
    global conn
    counter = 0
    while True:
        try:
            print("Waiting for a client to connect...")
            conn, addr = server_socket.accept()
            print(f"Connected by {addr}")
            break
        except Exception as e:
            print(f"Connection attempt failed: {e}")
            if(counter > 20):
                break
            counter += 1
            time.sleep(1)  # Wait before retrying"""

class Command(Enum):
    DRIVE = "drive"
    TURN = "turn"
    STOP = "stop"
    SERVO= "servo"

def send_command(command: tuple[Command, Any]):
    print(command)
    global conn
    name, val = command
    message = ','.join((name.value, str(val))) + ';'
    try:
        conn.send(message.encode())
    except (BrokenPipeError, ConnectionResetError, OSError):
        print("Connection lost. Reconnecting...")
        reconnect()

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



keyboard.add_hotkey('b', send_commands, args=(points_to_commands(robot_start_angle=robot_start_angle, positions=positions),))
keyboard.add_hotkey('e', send_command, args=((Command.STOP, None),))
#keyboard.add_hotkey('w', send_command, args=((Command.DRIVE, 800),))
#keyboard.add_hotkey('d', send_command, args=((Command.TURN, 90),))
#keyboard.add_hotkey('a', send_command, args=((Command.TURN, -90),))
#keyboard.add_hotkey('s', send_command, args=((Command.DRIVE, -800),))
keyboard.add_hotkey('t', send_command, args=((Command.SERVO, -30),))
keyboard.add_hotkey('y', send_command, args=((Command.SERVO, 0),))
keyboard.add_hotkey('u', send_command, args=((Command.SERVO, 80),))

keyboard.wait('q')
