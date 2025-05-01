#!/usr/bin/env pybricks-micropython
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, MoveSteering, MoveDifferential, MoveTank, SpeedPercent, SpeedDPM, follow_for_ms, FollowGyroAngleErrorTooFast
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import GyroSensor
from ev3dev2.wheel import EV3Tire
import threading
import socket
import time

class MessageQueue:
    def __init__(self):
        self.queue = []
        
    def print_queue(self):
        """Print the entire queue."""
        if not self.is_empty():
            print("Queue contents:")
            for message in self.queue:
                # Assuming message is a tuple (value1, value2)
                print("- Value 1:", message[0], ", Value 2:", message[1])
        else:
            print("Queue is empty!")
       
    def peek(self):
        """Return the first tuple without removing it."""
        if not self.is_empty():
            return self.queue[0]
        else:
            return None
         
    def enqueue(self, message):
        """Add a message to the queue."""
        execute = False
        if self.is_empty:
            execute = True
        self.queue.append(message)
        if(execute): execute_command(self.peek())
        print("Message added: ", message)
        self.print_queue()
    
    def dequeue(self):
        """Remove and return the first message from the queue."""
        if not self.is_empty():
            message = self.queue.pop(0)
            print("Message removed: ", message)
            return message
        else:
            print("Queue is empty!")
            return None
        
    def is_empty(self):
        """Check if the queue is empty."""
        return len(self.queue) == 0

queue = MessageQueue()

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
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(('', 12346))
server_socket.listen(1)

print("Waiting for connection...")
client_socket, addr = server_socket.accept()
print("Connected to {}".format(addr)) 

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
    steering.stop()

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
        stop()

def test():
    time.sleep(10)

def execute_command(command):
    cmd = command[0]
    arg = command[1]
    if(cmd == "stop"): stop()
    elif(cmd == "drive"): straight(-float(arg)) # Invert directionen since front is back.
    elif(cmd == "turn"): rotate(-float(arg))
    elif(cmd== "test"): test()
    queue.dequeue()
    if(not queue.is_empty()): execute_command(queue.peek())

def check_for_commands():
    try:
        while True:
            data = client_socket.recv(1024)
            if not data:
                break

            commands = data.decode().strip().split(',')
            print("Command received:", commands)
            command = commands.pop(0)
            if(len(commands) > 0):
                arg = commands.pop(0)
                t1 = threading.Thread(target = queue.enqueue((command, arg)))
            else: t1 = threading.Thread(target = queue.enqueue((command, None)))
            t1.start()
           
            
    finally:
        stop()
        client_socket.close()
        server_socket.close()
        print("Connection closed")

check_for_commands()

def long_straight_test():
    straight(10000)
    rotate(180)
    straight(10000)

def advanced_combined_test():
    straight(840)
    straight(840/2)
    rotate(-90)
    straight(840)
    rotate(-90)
    straight(840)
    straight(840/2)
    rotate(-90)
    straight(840)
    rotate(-90)

def combined_test_shear():
    rotate(-45)
    straight(840)
    rotate(-180)
    straight(840)
    rotate(-180)
    rotate(45)

def combined_test():
    straight(840)
    straight(840)
    rotate(180)
    straight(840)
    straight(840)
    rotate(180)

def rotate_test():
    rotate(90)