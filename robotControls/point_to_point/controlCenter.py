import socket
import keyboard
import math

ev3_ip = '192.168.0.138'  # Replace with EV3's IP
port = 12346

class Point:
    x = 0
    y = 0
    def __init__(self, x, y):
        self.x = x
        self.y = y

p1 = Point(5, 2) # the robots position
p2 = Point(2, 2) # position of the next ball
orientation = 0 # 0 .. 360 robots direction

client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect((ev3_ip, port))

# Function to send command to the server
def send_command(command: str):
    client_socket.send(command.encode())

def calculate_turn(p1, p2, orientation):
    v = Point(p2.x - p1.x, p2.y - p1.y)
    target_angle = math.degrees(math.atan2(v.y, v.x))
    turn_angle = target_angle - orientation
    return (turn_angle + 180) % 360 - 180 # Normalize turn_angle to the range -180 to 180

def calculate_distance(p1, p2):
    return math.sqrt((p2.x - p1.x)**2+(p2.y - p1.y)**2)

# Setup hotkeys for key press events
keyboard.add_hotkey('e', send_command, args=("stop",))
keyboard.add_hotkey('r', send_command, args=("resetgyro",))
keyboard.add_hotkey('w', send_command, args=("forward," + str(calculate_distance(p1, p2)),))
keyboard.add_hotkey('s', send_command, args=("backwards," + str(calculate_distance(p1, p2)),))
keyboard.add_hotkey('1', send_command, args=("toggle_servo",))

keyboard.wait('q')  # Keep waiting until 'q' is pressed to exit

client_socket.close()
