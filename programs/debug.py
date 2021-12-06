#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import Image, SoundFile, ImageFile

leftMotor = Motor(Port.B, positive_direction=Direction.COUNTERCLOCKWISE)
rightMotor = Motor(Port.C, positive_direction=Direction.COUNTERCLOCKWISE)
mediumMotorA = Motor(Port.A)
mediumMotorD = Motor(Port.D)
colorSensor1 = ColorSensor(Port.S1)
colorSensor2 = ColorSensor(Port.S2)
gyro3 = GyroSensor(Port.S3)
ultrasonicSensor4 = UltrasonicSensor(Port.S4)

while True:
    print("Color Sensor 1: " + str(colorSensor1.reflection()))
    print("Color Sensor 2: " + str(colorSensor2.reflection()))
    print("Untrasonic Sensor 3: " + str(gyro3.distance()))
    print("Untrasonic Sensor 4: " + str(ultrasonicSensor4.distance()))

    print("\n==========================\n")

    print("Medium Motor A: " + str(mediumMotorA.angle()))
    print("Large Motor B: " + str(leftMotor.angle()))
    print("Large Motor C: " + str(rightMotor.angle()))
    print("Medium Motor D: " + str(mediumMotorD.angle()))
    wait(1000)
