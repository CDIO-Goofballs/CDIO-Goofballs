#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
import socket

# Initialize the EV3 Brick.
ev3 = EV3Brick()

# Initialize the motors.
left_motor = Motor(Port.A)
right_motor = Motor(Port.B)

# Initialize the drive base.
robot = DriveBase(left_motor, right_motor, wheel_diameter=68.8, axle_track=145)

# Setup EV3 server
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('', 12346))
server_socket.listen(1)

print("Waiting for connection...")
client_socket, addr = server_socket.accept()
print("Connected to {}".format(addr))

fans = True # true when fans are on.

def execute_command(command, arg = None):
    if(command == "stop"): robot.stop()
    elif(command == "forward"): robot.straight(-float(arg)) # Invert directionen since front is back.
    elif(command == "turn"): robot.turn(float(arg))
    elif(command == "reverse"): robot.straight(float(arg)) # Invert directionen since front is back.
    elif(command == "fan_off"): fans = False
    elif(command == "fan_on"): fans = True

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