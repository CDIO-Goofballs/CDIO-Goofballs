#!/usr/bin/env python3
from ev3dev2.motor import MediumMotor, LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_D, MoveSteering, MoveDifferential, MoveTank, SpeedPercent, SpeedDPM, follow_for_ms, FollowGyroAngleErrorTooFast
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import GyroSensor
from ev3dev2.wheel import EV3Tire
import socket
import time
import threading
import queue
import math

queue = queue.Queue()

# Initialize the motors.
motorA = LargeMotor(OUTPUT_A)
motorB = LargeMotor(OUTPUT_B)
servo = MediumMotor(OUTPUT_D)

servo_position = 0 # -30 is closed, 0 is slightly open, 80 is open and letting balls out.
servo.reset()

tank = MoveTank(OUTPUT_A, OUTPUT_B)
steering = MoveSteering(OUTPUT_A, OUTPUT_B)
move_diff = MoveDifferential(OUTPUT_A, OUTPUT_B, EV3Tire, 170)

# Initialize the tank's gyro sensor
tank.gyro = GyroSensor()
move_diff.gyro = GyroSensor()

shutdown_event = threading.Event()
stop_event = threading.Event()

# Calibrate the gyro to eliminate drift, and to initialize the current angle as 0
tank.gyro.calibrate()

# Constants
WHEEL_DIAMETER_MM = 68  # Adjust based on your wheel
DEGREES_PER_ROTATION = 360
PI = 3.1416
WHEEL_CIRCUMFERENCE_CM = 6.8 * PI   # Approx. for standard EV3 wheels (5.6cm diameter)
WHEEL_BASE_CM = 16.5
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
    steering.off()
    move_diff.off()
    stop_event.set()
    servo.on_to_position(SpeedPercent(20), 0)
    while (not queue.empty()):
        queue.get()

def adjust_to_gyro(target_deg, speed=1):
    print("Adjust gyro")
    while abs(tank.gyro.angle - target_deg) > 1 and not stop_event.is_set:  # allow a tiny error range
        current_angle = tank.gyro.angle
        print("Gyro Angle: ", current_angle)
        if current_angle < target_deg:
            # turn right (clockwise)
            tank.on(SpeedPercent(speed), SpeedPercent(-speed))
        else:
            # turn left (counter-clockwise)
            tank.on(SpeedPercent(-speed), SpeedPercent(speed))
        time.sleep(0.01)

def rotate(deg):
    print("Turning by ", deg)
    tank.gyro.reset()
    tank.turn_degrees(speed=SpeedPercent(30), target_angle=deg)
    #time.sleep(0.5)
    #adjust_to_gyro(deg)
    
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
    

def straight(distance, speed = 30):
    rotations = CM_TO_ROTATIONS(abs(distance/10))
    if distance > 0:
        tank.on_for_rotations(speed, speed, rotations)
    else:
        tank.on_for_rotations(-speed, -speed, rotations)

def test():
    time.sleep(10)

def execute_command():
    while not shutdown_event.is_set():
        if(stop_event.is_set()): 
            continue
        command = queue.get()
        cmd = command[0]
        arg = command[1]
        print("Got command: ", (cmd, arg))
        if(cmd == "drive"): straight(-float(arg)) # Invert directionen since front is back.
        elif(cmd == "backwards"): straight(float(arg))
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
            print(commands)
            for cmd in commands:
                print(cmd)
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

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server_socket:
    server_socket.bind(('', 12345))
    server_socket.listen(1)

    print("Waiting for connection...")
    client_socket, addr = server_socket.accept()
    print("Connected to {}".format(addr))
    
    producer = threading.Thread(target=check_for_commands, args=(client_socket,), daemon=True)
    consumer = threading.Thread(target=execute_command, daemon=True)

    
    consumer.start()
    producer.start()
    
    while(not shutdown_event.is_set()):
        time.sleep(0.1)
    
stop()