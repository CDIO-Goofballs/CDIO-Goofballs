#!/usr/bin/env python3
from ev3dev2.motor import MediumMotor, LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_D, SpeedPercent, MoveDifferential
from ev3dev2.sensor import INPUT_3
from ev3dev2.sensor.lego import GyroSensor
from ev3dev2.wheel import Wheel
import socket
import time
import threading
import queue
import ast

# Constants
WHEEL_DIAMETER_MM = 67 # Real life size is 68.8 mm
WHEEL_WIDTH_MM = 36 # Real life size is 36 mm
WHEEL_BASE_MM = 220 # Real life size is 170 mm

ROTATION_SPEED = 40
STRAIGHT_SPEED = 50

SERVER_IP = '192.168.137.1'
SERVER_PORT = 12345

queue = queue.Queue()

motorA = LargeMotor(OUTPUT_B) # Reverse motors because we drive inverted
motorB = LargeMotor(OUTPUT_A)
servo = MediumMotor(OUTPUT_D)

servo_position = 0 # -30 is closed, 0 is slightly open, 80 is open and letting balls out.
servo.reset()

class CustomWheel(Wheel):
    def __init__(self):
        super().__init__(WHEEL_DIAMETER_MM, WHEEL_WIDTH_MM)

move_diff = MoveDifferential(OUTPUT_A, OUTPUT_B, CustomWheel, WHEEL_BASE_MM)
move_diff.gyro = GyroSensor(INPUT_3)
move_diff.gyro.calibrate()

shutdown_event = threading.Event()
stop_event = threading.Event()

def stop():
    print("Stopped")
    move_diff.stop()
    move_diff.off()
    stop_event.set()
    #servo.on_to_position(SpeedPercent(20), 0)
    while not queue.empty():
        queue.get()
    
def rotate(angle, speed=ROTATION_SPEED):
    move_diff.turn_degrees(speed = SpeedPercent(speed), degrees = angle, use_gyro = False)
    

def straight(distance, speed = STRAIGHT_SPEED):
    if distance > 0:
        move_diff.on_for_distance(SpeedPercent(speed), -distance)
    else:
        move_diff.on_for_distance(SpeedPercent(-speed), distance)

def execute_command(conn):
    while not shutdown_event.is_set():
        if stop_event.is_set():
            continue
        command = queue.get()
        cmd = command[0]
        arg = command[1]
        if cmd == "drive": straight(arg[0], speed=arg[1])
        elif cmd == "turn": rotate(-arg)
        elif cmd == "servo": move_servo(arg[0], arg[1]) if isinstance(arg, tuple) else move_servo(arg)
        elif cmd == "test": test()
        time.sleep(0.1)
        conn.send("Done".encode())

def test():
    print("Wait for sleep")
    time.sleep(10)
    print("Test complete.")

def move_servo(position, brake=True):
    global servo_position

    if position == servo_position:
        return
    servo.on_to_position(SpeedPercent(20), position, brake=brake)
    servo_position = position
    servo.off(brake=brake)
        

def check_for_commands(conn):
    try:
        while not shutdown_event.is_set():
            data = conn.recv(1024)
            if not data:
                break
            
            commands = data.decode().strip().rstrip(';').split(';') # Remove trailing semicolon to prevent empty string at the end
            
            for cmd in commands:
                cmds = cmd.split(',', 1) # Split only on the first comma
                cmd_name = cmds[0].strip()
                if cmd_name == "stop":
                    stop()
                    continue
                else:
                    stop_event.clear()
                if len(cmds) > 1:
                    arg = ast.literal_eval(cmds[1].strip())
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
            consumer = threading.Thread(target=execute_command, args=(client_socket,), daemon=True)

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