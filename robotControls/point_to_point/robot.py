#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, GyroSensor
from pybricks.parameters import Port, Direction, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait
import socket

fans = True # true when fans are on.

# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize the motors.
left_motor = Motor(Port.A)
right_motor = Motor(Port.B)
gyro_sensor = GyroSensor(Port.S1)
gyro_sensor.reset_angle(0)

# Setup EV3 server
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('', 12345))
server_socket.listen(1)

print("Waiting for connection...")
client_socket, addr = server_socket.accept()
print("Connected to {}".format(addr))  

# Constants
WHEEL_DIAMETER_MM = 68.8  # Adjust based on your wheel
DEGREES_PER_ROTATION = 360
PI = 3.1416

def mm_to_degrees(distance_mm):
    wheel_circumference = PI * WHEEL_DIAMETER_MM
    rotations_needed = distance_mm / wheel_circumference
    return rotations_needed * DEGREES_PER_ROTATION

def straight(rotations):
    left_motor.run_angle(90, rotations, then=Stop.HOLD, wait=False)
    right_motor.run_angle(90, rotations, then=Stop.HOLD, wait=False)
    wait(1000)
    while(gyro_sensor.angle() == 0): # Does angle() return int or float?
        wait(10)
    stop()
    if(gyro_sensor.angle() < 0):
        right_motor.run_target(10, right_motor.angle() + 10, then=Stop.HOLD, wait=True) # Does not return to exactly 0 degress
    else:
        left_motor.run_target(10, left_motor.angle() + 10, then=Stop.HOLD, wait=True) # Does not return to exactly 0 degress
    straight(rotations - (left_motor.angle() + right_motor.angle())/2)

def stop():
    left_motor.stop()
    right_motor.stop()

def execute_command(command, arg = None):
    if(command == "stop"): stop()
    elif(command == "forward"): straight(mm_to_degrees(float(arg))) # Invert directionen since front is back.
    #elif(command == "turn"): robot.turn(float(arg))
    elif(command == "backwards"): straight(mm_to_degrees(-float(arg))) # Invert directionen since front is back.
    elif(command == "fan_off"): fans = False
    elif(command == "fan_on"): fans = True
    elif(command == "resetgyro"): gyro_sensor.reset_angle(0)

# Keep connection open for multiple commands
try:
    while fans:
        data = client_socket.recv(1024)
        if not data:
            break

        commands = data.decode().strip().split(',')
        print("Command received:", commands)
        command = commands.pop(0)
        if(len(commands) > 0):
            arg = commands.pop(0)
            execute_command(command, arg)
        else: execute_command(command)
        
        

finally:
    client_socket.close()
    server_socket.close()
    print("Connection closed")