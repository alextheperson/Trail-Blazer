#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

from library import FUNCTION_LIBRARY


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
leftMotor = Motor(Port.B)
rightMotor = Motor(Port.C)
mediumMotorA = Motor(Port.A)
mediumMotorD = Motor(Port.D)

colorSensor1 = ColorSensor(Port.S1)
colorSensor2 = ColorSensor(Port.S2)

gyro3 = GyroSensor(Port.S3)
gyro4 = GyroSensor(Port.S4)

# Initialize the drive base.
robot = DriveBase(leftMotor, rightMotor, wheel_diameter=55.5, axle_track=104)
library = FUNCTION_LIBRARY(robot, ev3, leftMotor, rightMotor, mediumMotorA, mediumMotorD, colorSensor1, colorSensor2, gyro3, gyro4)

library.lineFollowUntilShade(shade=90)
