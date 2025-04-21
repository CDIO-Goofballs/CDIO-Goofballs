#!/usr/bin/env pybricks-micropython
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, MoveSteering, MoveDifferential, MoveTank, SpeedPercent, SpeedDPM, follow_for_ms, FollowGyroAngleErrorTooFast
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import GyroSensor
from ev3dev2.wheel import EV3Tire
import time

# Initialize the motors.
motorA = LargeMotor(OUTPUT_A)
motorB = LargeMotor(OUTPUT_B)

tank = MoveTank(OUTPUT_A, OUTPUT_B)
steering = MoveSteering(OUTPUT_A, OUTPUT_B)
move_diff = MoveDifferential(OUTPUT_A, OUTPUT_B, EV3Tire, 170)

# Initialize the tank's gyro sensor
tank.gyro = GyroSensor()
move_diff.gyro = GyroSensor()

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
    while abs(tank.gyro.angle - target_deg) > 0:  # allow a tiny error range
        current_angle = tank.gyro.angle
        print(current_angle)

        if current_angle < target_deg:
            # turn right (clockwise)
            tank.on(SpeedPercent(speed), SpeedPercent(-speed))
        else:
            # turn left (counter-clockwise)
            tank.on(SpeedPercent(-speed), SpeedPercent(speed))
        time.sleep(0.01)
    tank.off()
    # while(tank.gyro.angle != deg):
    #     if(tank.gyro.angle < deg):
    #         tank.turn_degrees(speed=SpeedPercent(5), target_angle=deg)
    #     else:
    #         tank.turn_degrees(speed=SpeedPercent(5), target_angle=deg)
    #     print(tank.gyro.angle)

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
        # motorA.on_for_degrees(20, mm_to_degrees(distance+error), block=False)
        # motorB.on_for_degrees(20, mm_to_degrees(distance+error), block=False)
        # while(motorA.speed != 0 or motorB.speed != 0):
        #     if(tank.gyro.angle):
        #         print(tank.gyro.angle)
    finally:
        stop()


    # degrees = mm_to_degrees(distance+error)
    # motorA.on_for_degrees(50, degrees, brake=True, block=False)
    # motorB.on_for_degrees(50, degrees, brake=True, block=False)
    # print()

def rotate(deg):
    tank.gyro.reset()
    tank.turn_degrees(speed=SpeedPercent(30), target_angle=deg)
    time.sleep(0.5)
    adjust_to_gyro(deg)
    #move_diff.turn_degrees(SpeedPercent(50), deg, brake=True, block=True, error_margin=2, use_gyro=True)

def long_straight_test():
    straight(595*5)
    straight(-595*5)

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

advanced_combined_test()