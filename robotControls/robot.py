#!/usr/bin/env python3
from ev3dev2.motor import MediumMotor, LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_D, SpeedPercent, MoveDifferential
from ev3dev2.sensor import INPUT_3
from ev3dev2.sensor.lego import GyroSensor
from ev3dev2.wheel import Wheel
import socket
import time
import threading
import queue

# Constants
WHEEL_DIAMETER_MM = 67 # Real life size is 68.8 mm
WHEEL_WIDTH_MM = 36 # Real life size is 36 mm
WHEEL_BASE_MM = 200 # Real life size is 170 mm

ROTATION_SPEED = 10
STRAIGHT_SPEED = 10

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
    servo.on_to_position(SpeedPercent(20), 0)
    while (not queue.empty()):
        queue.get()
    
def adjust_to_gyro(target_deg, speed=20):
    print("Target angle: ", target_deg)
    while move_diff.gyro.angle - target_deg > 1 and not stop_event.is_set():  # allow a tiny error range
        current_angle = move_diff.gyro.angle
        if current_angle > target_deg:
            # turn right (clockwise)
            move_diff.turn_right(SpeedPercent(speed), 2)
        else:
            # turn left (counter-clockwise)
            move_diff.turn_left(SpeedPercent(speed), 2)
        time.sleep(0.01)
    
def rotate(angle, speed=ROTATION_SPEED):
    move_diff.turn_degrees(speed = SpeedPercent(speed), degrees = angle, use_gyro = False)
    #adjust_to_gyro(-angle)
    #move_diff.gyro.reset()
    

def straight(distance, speed = STRAIGHT_SPEED):
    if distance > 0:
        move_diff.on_for_distance(SpeedPercent(speed), -distance)
    else:
        move_diff.on_for_distance(SpeedPercent(-speed), distance)

def execute_command(conn):
    while not shutdown_event.is_set():
        if(stop_event.is_set()): 
            continue
        command = queue.get()
        cmd = command[0]
        arg = command[1]
        if(cmd == "drive"): straight(float(arg)) # Invert directionen since front is back.
        elif(cmd == "turn"): rotate(float(arg))
        elif(cmd == "servo"): move_servo(int(arg))
        print("Done!")
        time.sleep(0.1)
        conn.send("Done".encode())

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