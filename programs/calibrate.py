#!/usr/bin/env pybricks-micropython
from pybricks.ev3devices import (ColorSensor, GyroSensor, Motor,
                                 UltrasonicSensor)
from pybricks.hubs import EV3Brick
from pybricks.media.ev3dev import Image
from pybricks.parameters import Button, Direction, Port
from pybricks.robotics import DriveBase

from comboFour import comboFour
#import the combos
from comboOne import comboOne
from comboThree import comboThree
from comboTwo import comboTwo
#import the library
from library import FUNCTION_LIBRARY

# This program requires LEGO EV3 MicroPython v2.0 or higher.
# For more information:
# https://pybricks.github.io/ev3-micropython/

# Create your objects here.
ev3 = EV3Brick()

#--MOTORS--
#Left Large Motor: C
#Right Large Motor: B
#Left Medium Motor: D
#Right Medium Motor: A
#--SENSORS--
#Left Color Sensor: 2
#Right Color Sensor: 1
#Gyroscope: 3
#Ultrasonic: 4


try:
    leftMotor = Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE)
except:
    leftMotor = None
    print("Library error: Could not find LEFT LARGE MOTOR on PORT B")

try:
    rightMotor = Motor(Port.C, positive_direction=Direction.COUNTERCLOCKWISE)
except:
    rightMotor = None
    print("Library error: Could not find RIGHT LARGE MOTOR on PORT C")

try:
    mediumMotorA = Motor(Port.A)
except:
    mediumMotorA = None
    print("Library error: Could not find RIGHT MEDIUM MOTOR on PORT A")

try:
    mediumMotorD = Motor(Port.D)
except:
    mediumMotorD = None
    print("Library error: Could not find RIGHT MEDIUM MOTOR on PORT D")

try:
    colorSensor1 = ColorSensor(Port.S1)
except:
    colorSensor1 = None
    print("Library error: Could not find COLOR SENSOR on PORT 1")

try:
    colorSensor2 = ColorSensor(Port.S2)
except:
    colorSensor2 = None
    print("Library error: Could not find COLOR SENSOR on PORT 2")

try:
    gyro3 = GyroSensor(Port.S3)
except:
    gyro3 = None
    print("Library error: Could not find GYROSCOPE on PORT 3")

try:
    ultrasonicSensor4 = UltrasonicSensor(Port.S4)
except:
    ultrasonicSensor4 = None
    print("Library error: Could not find ULTRASONIC SENSOR on PORT 4")

# Initialize the drive base.
robot = DriveBase(leftMotor, rightMotor, wheel_diameter=50, axle_track=115)
 
# init the library
library = FUNCTION_LIBRARY(robot, ev3, leftMotor, rightMotor, mediumMotorD, mediumMotorA, colorSensor1, colorSensor2, gyro3, ultrasonicSensor4)

while True:
    if Button.CENTER in ev3.buttons.pressed():
        robot.straight(100)
        robot.straight(-100)
        break
while True:
    if Button.CENTER in ev3.buttons.pressed():
        robot.turn(90)
        robot.turn(-90)
        break