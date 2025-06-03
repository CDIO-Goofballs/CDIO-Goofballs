#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, MoveSteering, MoveDifferential, MoveTank, SpeedPercent, SpeedDPM, follow_for_ms, FollowGyroAngleErrorTooFast
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import GyroSensor
from ev3dev2.wheel import EV3Tire
import socket
import time
import threading
import queue

queue = queue.Queue()

# Initialize the motors.
motorA = LargeMotor(OUTPUT_A)
motorB = LargeMotor(OUTPUT_B)

tank = MoveTank(OUTPUT_A, OUTPUT_B)
steering = MoveSteering(OUTPUT_A, OUTPUT_B)
move_diff = MoveDifferential(OUTPUT_A, OUTPUT_B, EV3Tire, 170)

# Initialize the tank's gyro sensor
tank.gyro = GyroSensor()
move_diff.gyro = GyroSensor()

# Setup EV3 server
#server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
#server_socket.bind(('', 12346))
#server_socket.listen(1)

shutdown_event = threading.Event()
stop_event = threading.Event()

#print("Waiting for connection...")
#client_socket, addr = server_socket.accept()
#print("Connected to {}".format(addr)) 

# Calibrate the gyro to eliminate drift, and to initialize the current angle as 0
tank.gyro.calibrate()

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


def stop():
    print("Stopped")
    tank.stop()
    tank.off()
    stop_event.is_set()
    while (not queue.empty()):
        queue.get()

def adjust_to_gyro(target_deg, speed=1):
    while abs(tank.gyro.angle - target_deg) > 1:  # allow a tiny error range
        current_angle = tank.gyro.angle

        if current_angle < target_deg:
            # turn right (clockwise)
            tank.on(SpeedPercent(speed), SpeedPercent(-speed))
        else:
            # turn left (counter-clockwise)
            tank.on(SpeedPercent(-speed), SpeedPercent(speed))
        time.sleep(0.01)
    tank.off()

def rotate(deg):
    tank.gyro.reset()
    tank.turn_degrees(speed=SpeedPercent(30), target_angle=deg)
    time.sleep(0.5)
    adjust_to_gyro(deg)
    

def straight(distance):
    tank.gyro.reset()
    direction = 1
    if(distance < 0): 
        direction = -1
    speed_mm_per_sec = 185      # estimate this from testing | 92.5 cm / 5 seconds = 18.5
    time_sec = abs(distance) / speed_mm_per_sec
    time_ms = time_sec * 1000
    try:
        tank.follow_gyro_angle(
            kp=11.3, ki=0.05, kd=3.2,
            speed=SpeedPercent(direction*30),
            target_angle=0,
            follow_for=follow_for_ms,
            ms=time_ms
        )
    except FollowGyroAngleErrorTooFast:
        tank.stop()
        raise
    finally:
        tank.stop()

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
        elif(cmd == "turn"): rotate(-float(arg))
        #elif(cmd== "test"): test()
        time.sleep(0.1)


def check_for_commands(conn):
    try:
        while not shutdown_event.is_set():
            data = conn.recv(1024)
            if not data:
                break

            commands = data.decode().strip().split(',')
            print("Command received:", commands)
            command = commands.pop(0)
            
            if(command == "stop"): 
                stop()
                continue
            else:
                stop_event.clear()
            if(len(commands) > 0):
                arg = commands.pop(0)
                queue.put((command, arg))
            else: queue.put((command, None))
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