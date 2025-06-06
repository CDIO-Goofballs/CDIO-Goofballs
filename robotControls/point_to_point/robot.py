#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor
from pybricks.parameters import Port, Direction, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait
import socket
import math

fans = True # true when fans are on.
is_open = False

# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize the motors.
left_motor = Motor(Port.A)
right_motor = Motor(Port.B)
servo = Motor(Port.C)
servo.reset_angle(0)
gyro_sensor = GyroSensor(Port.S1)
gyro_sensor.reset_angle(0)

# Drivebase
drivebase = DriveBase(left_motor, right_motor, 70, 170)

# Setup EV3 server
# server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# server_socket.bind(('', 12346))
# server_socket.listen(1)

# print("Waiting for connection...")
# client_socket, addr = server_socket.accept()
# print("Connected to {}".format(addr))  

# Constants
WHEEL_DIAMETER_MM = 70  # Adjust based on your wheel
DEGREES_PER_ROTATION = 360
PI = 3.1416

def mm_to_degrees(distance_mm):
    wheel_circumference = PI * WHEEL_DIAMETER_MM
    rotations_needed = distance_mm / wheel_circumference
    return rotations_needed * DEGREES_PER_ROTATION


def degrees_to_mm(degrees):
    """Convert rotation in degrees to linear distance in mm."""
    circumference = PI * WHEEL_DIAMETER_MM
    return (degrees / 360) * circumference

def rotate(deg):
    gyro_sensor.reset_angle(0)
    drivebase.turn(deg)
    adjust_to_gyro(deg)

def adjust_to_gyro(deg):
    while(gyro_sensor.angle() != deg):
        if(gyro_sensor.angle() < deg):
            drivebase.turn(1)
        else:
            drivebase.turn(-1)
        wait(100)
        print(gyro_sensor.angle())

def straight(distance, error=60-30):
    degrees = math.trunc(mm_to_degrees(distance+error))
    left_motor.dc(100)
    right_motor.dc(100)
    # if(distance == 0): return
    # direction = 1
    # if(distance < 0):
    #     direction = -1
    # print(gyro_sensor.angle())
    # gyro_sensor.reset_angle(0)
    # total_distance = abs(distance)+error  # mm
    # parts = 4
    # base_step = total_distance // parts
    # remainder = total_distance % parts  # Handle leftover mm

    # # Create a list of all step sizes
    # steps = [base_step] * parts
    # steps[0] += remainder  # Add the extra to the first step (can distribute differently if you want)
    # for i, step_mm in enumerate(steps):
    #     drivebase.straight(direction*step_mm)
    #     wait(100) 
    #     if(abs(gyro_sensor.angle()) > 1):
    #         adjust_to_gyro(0)

def stop():
    left_motor.stop()
    right_motor.stop()

def toggle_servo():
    global is_open
    is_open = not is_open
    if(is_open):
        servo.run_target(500, 0)
    else:
        servo.run_target(500, 180)

#straight(1000)
def rotation_test():
    rotate(90)
    rotate(-90)
    rotate(180)
    rotate(360)

def combined_test():
    straight(1800)
    rotate(180)
    straight(1800)
    rotate(180)

def straight_test():
    straight(850*2)

combined_test()
# def execute_command(command, arg = None):
#     if(command == "stop"): stop()
#     elif(command == "forward"): straight(float(arg)) # Invert directionen since front is back.
#     #elif(command == "turn"): robot.turn(float(arg))
#     elif(command == "backwards"): straight(-float(arg)) # Invert directionen since front is back.
#     elif(command == "fan_off"): fans = False
#     elif(command == "fan_on"): fans = True
#     elif(command == "resetgyro"): gyro_sensor.reset_angle(0)
#     elif(command == "toggle_servo"): toggle_servo()

# # Keep connection open for multiple commands
# try:
#     while fans:
#         data = client_socket.recv(1024)
#         if not data:
#             break

#         commands = data.decode().strip().split(',')
#         print("Command received:", commands)
#         command = commands.pop(0)
#         if(len(commands) > 0):
#             arg = commands.pop(0)
#             execute_command(command, arg)
#         else: execute_command(command)
        
        

# finally:
#     client_socket.close()
#     server_socket.close()
#     print("Connection closed")