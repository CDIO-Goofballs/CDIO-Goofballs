#!/usr/bin/env pybricks-micropython
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import GyroSensor
import socket

# speed variables
speed = 30 # 0 .. 100
rotationSpeed = 5 # 0 .. 100
direction = 0

# Initialize the EV3 Brick.

# Initialize the motors.
motorA = LargeMotor(OUTPUT_A)
motorB = LargeMotor(OUTPUT_B)

# Initialize the sensors.
gyro = GyroSensor(INPUT_1)
gyro.mode = GyroSensor.MODE_GYRO_ANG
# Setup EV3 server
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('', 12346))
server_socket.listen(1)

# Initialize the drive base.
#tank_drive = MoveTank(OUTPUT_A, OUTPUT_B)

print("Waiting for connection...")
client_socket, addr = server_socket.accept()
print("Connected to {}".format(addr))

def setMotorSpeed(speed):
    if(direction == 1): # Forward
        motorA.on(speed)
        motorB.on(speed)
    elif(direction == 2): # Backward
        motorA.on(-speed)
        motorB.on(-speed)
    elif(direction == 3): # Left
        motorA.on(-speed)
        motorB.on(speed)
    elif(direction == 4): # Right
        motorA.on(speed)
        motorB.on(-speed)

# Keep connection open for multiple commands
try:
    while True:
        data = client_socket.recv(1024)
        if not data:
            break

        command = data.decode().strip()
        print("Command received:", command)

        if command == "forward":
            print("Moving forward!")
            direction = 1
            motorA.on(speed)
            motorB.on(speed)
            client_socket.send(b"Moving forward")
        elif command == "stop":
            print("Stopping!")
            direction = 0
            motorA.on(0)
            motorB.on(0)
            client_socket.send(b"Stopping")
        elif command == "right":
            print("Turning right")
            direction = 3
            motorA.on(rotationSpeed)
            motorB.on(-rotationSpeed)
            gyro.wait_until_angle_changed_by(90)
            motorA.on(0)
            motorB.on(0)
            client_socket.send(b"Turning right")
        elif command == "left":
            print("Turning left")
            direction = 4
            motorA.on(-rotationSpeed)
            motorB.on(rotationSpeed)
            gyro.wait_until_angle_changed_by(90)
            motorA.on(0)
            motorB.on(0)
            client_socket.send(b"Turning left")
        elif command == "backwards":
            print("Moving backwards")
            direction = 2
            motorA.on(-speed)
            motorB.on(-speed)
            client_socket.send(b"Moving backwards")
        elif command == "exit":
            print("Exiting...")
            client_socket.send(b"Goodbye!")
            break
        elif command == "speedup":
            print("Increasing speed")
            speed = max(min(speed + 10, 100), 0)
            setMotorSpeed(speed)
        elif command == "speeddown":
            print("decreasing speed")
            speed = max(min(speed - 10, 100), 0)
            setMotorSpeed(speed)
        elif command == "calibrateGyro":
            print("Calibrating gyro sensor please do not move the robot")
            gyro.mode = GyroSensor.MODE_GYRO_CAL
            print("Calibration done. Back to angle.")
            gyro.mode = GyroSensor.MODE_GYRO_ANG
        else:
            client_socket.send(b"Unknown command")

finally:
    client_socket.close()
    server_socket.close()
    print("Connection closed")