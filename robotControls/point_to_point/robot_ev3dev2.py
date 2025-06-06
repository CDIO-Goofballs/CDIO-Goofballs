#!/usr/bin/env python3
from ev3dev2.motor import MediumMotor, LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_D, MoveTank, SpeedPercent
from ev3dev2.sensor import INPUT_3
from ev3dev2.sensor.lego import GyroSensor
import socket
import time
import threading
import queue

queue = queue.Queue()

# Initialize the motors.
motorA = LargeMotor(OUTPUT_B) # Reverse motors because we drive inverted
motorB = LargeMotor(OUTPUT_A)
servo = MediumMotor(OUTPUT_D)

servo_position = 0 # -30 is closed, 0 is slightly open, 80 is open and letting balls out.
servo.reset()

tank = MoveTank(OUTPUT_A, OUTPUT_B)
# Initialize the tank's gyro sensor
tank.gyro = GyroSensor(INPUT_3)
tank.gyro.mode = "GYRO-ANG"

shutdown_event = threading.Event()
stop_event = threading.Event()

# Calibrate the gyro to eliminate drift, and to initialize the current angle as 0
tank.gyro.calibrate()

# Constants
WHEEL_DIAMETER_MM = 68  # Adjust based on your wheel
DEGREES_PER_ROTATION = 360
PI = 3.1416
WHEEL_CIRCUMFERENCE_CM = 6.8 * PI   # Approx. for standard EV3 wheels (5.6cm diameter)
WHEEL_BASE_CM = 17
DEGREES_TO_ROTATIONS = lambda angle: (WHEEL_BASE_CM * 3.1416 * angle) / (360 * WHEEL_CIRCUMFERENCE_CM)
WHEEL_DIAMETER_CM = 6.8  # Standard EV3 wheel diameter
CM_TO_ROTATIONS = lambda cm: cm / WHEEL_CIRCUMFERENCE_CM

def mm_to_degrees(distance_mm):
    wheel_circumference = PI * WHEEL_DIAMETER_MM
    rotations_needed = distance_mm / wheel_circumference
    return rotations_needed * DEGREES_PER_ROTATION


def degrees_to_mm(degrees):
    """Convert rotation in degrees to linear distance in mm."""
    circumference = PI * WHEEL_DIAMETER_MM
    return (degrees / 360) * circumference


def stop():
    print("Stopped")
    tank.stop()
    tank.off()
    stop_event.set()
    servo.on_to_position(SpeedPercent(20), 0)
    while (not queue.empty()):
        queue.get()
    
def rotate_robot(angle, speed=15):
    """
    Rotates the robot in place by a specified angle (in degrees).
    Positive angle = right turn; Negative = left turn.
    """
    # Calculate how many wheel rotations are needed for the given angle
    tank.gyro.reset()
    time.sleep(0.5)
    # Determine direction
    if angle > 0:
        # Turn right
        tank.on(speed, -speed)
        while -tank.gyro.angle < angle and not stop_event.is_set():
            time.sleep(0.001)
    else:
        # Turn left
        tank.on(-speed, speed)
        while -tank.gyro.angle > angle and not stop_event.is_set():
            time.sleep(0.001)
    tank.off()
    

def straight(distance, speed = 60):
    rotations = CM_TO_ROTATIONS(abs(distance/10))
    if distance > 0:
        tank.on_for_rotations(speed, speed, rotations)
    else:
        tank.on_for_rotations(-speed, -speed, rotations)

def execute_command():
    while not shutdown_event.is_set():
        if(stop_event.is_set()): 
            continue
        command = queue.get()
        cmd = command[0]
        arg = command[1]
        if(cmd == "drive"): straight(-float(arg)) # Invert directionen since front is back.
        elif(cmd == "turn"): rotate_robot(float(arg))
        elif(cmd == "servo"): move_servo(int(arg))
        time.sleep(0.1)

def move_servo(position):
    global servo_position

    if position == servo_position:
        return
    servo.on_to_position(SpeedPercent(20), position)
    servo_position = position
    servo.off()
        

def check_for_commands(conn):
    try:
        while not shutdown_event.is_set():
            data = conn.recv(1024)
            if not data:
                break
            
            commands = data.decode().strip().rstrip(';').split(';') # Remove trailing semicolon to prevent empty string at the end
            
            for cmd in commands:
                cmds = cmd.split(',')
                cmd_name = cmds[0].strip()
                if(cmd_name == "stop"): 
                    stop()
                    continue
                else:
                    stop_event.clear()
                if(len(commands) > 0):
                    arg = cmds[1].strip()
                    queue.put((cmd_name, arg))
                else: queue.put((cmd_name, None))
                time.sleep(0.1)
            
    finally:
        stop()
        conn.close()
        shutdown_event.set()
        print("Connection closed")

# Server info
server_ip = '192.168.137.1'
server_port = 12345

while not shutdown_event.is_set():
    try:
        print("Attempting to connect to: ",  server_ip, server_port)
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client_socket:
            client_socket.connect((server_ip, server_port))
            print("Connected to server.")

            shutdown_event.clear()

            producer = threading.Thread(target=check_for_commands, args=(client_socket,), daemon=True)
            consumer = threading.Thread(target=execute_command, daemon=True)

            producer.start()
            consumer.start()

            while not shutdown_event.is_set():
                time.sleep(0.1)

    except (ConnectionRefusedError, OSError):
        print("Connection failed. Retrying in 5 seconds...")
        stop()
        time.sleep(5)

    print("Disconnected. Reconnecting...")
    shutdown_event.clear()
    time.sleep(1)
    
stop()