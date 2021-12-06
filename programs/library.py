#!/usr/bin/env pybricks-micropython
from math import *
from time import sleep

from pybricks.media.ev3dev import Image
from pybricks.parameters import Button
from pybricks.tools import StopWatch
from math import *



class FUNCTION_LIBRARY:
    def __init__(self, robot, ev3, leftDriveMotor, rightDriveMotor, leftAttachment, rightAttachment, colorSensor1, colorSensor2, gyroscope3, ultrasonicSensor4):
        #self, DriveBase, Hub
        self.driveBase = robot
        self.hub = ev3
        
        self.leftDriveMotor = leftDriveMotor
        self.rightDriveMotor = rightDriveMotor
        self.leftAttachment = leftAttachment
        self.rightAttachment = rightAttachment

        self.leftDriveMotor.reset_angle(0)
        self.rightDriveMotor.reset_angle(0)

        self.stopWatch = StopWatch()

        self.colorSensor1 = colorSensor1 
        self.colorSensor2 = colorSensor2
        self.gyroscope3 = gyroscope3
        self.ultrasonicSensor4 = ultrasonicSensor4

        self.black = 9
        self.white = 85

        self.gyro3Drift = False

    #PURPOSE: Calibrates robot
    #PARAMS: None
    def calibrate(self):
        self.checkGyroscopes()
        self.calibrateColors()
    
    #PURPOSE: Calibrates Gyroscopes
    #PARAMS: None
    def checkGyroscopes(self):
        if self.gyroscope3 != None: self.gyroscope3.reset_angle(0)

        try: print("Gyro 3 Angle First: " + str(self.gyroscope3.angle()))
        except: print("Gyro 3 Angle First: [Error]")

        sleep(1)

        try: print("Gyro 3 Angle Second: " + str(self.gyroscope3.angle()))
        except: print("Gyro 3 Angle Second: [Error]")

        if self.gyroscope3 != None: self.gyro3Drift = (self.gyroscope3.angle() != 0)
        else: self.gyro3Drift = True

        print("Gyro 3 Drift: " + str(self.gyro3Drift))
    
    #PURPOSE: Calibrates Color Sensors
    #PARAMS: None
    def calibrateColors(self):
        blackDone = False
        whiteDone = False

        print("old black: " + str(self.black) + ", old white: " + str(self.white))
        self.hub.screen.load_image(Image('GUI/ColorCalibrateNone.PNG'))
        while True:
            if Button.LEFT in self.hub.buttons.pressed() and not blackDone:
                self.black = (self.colorSensor1.reflection() + self.colorSensor2.reflection()) / 2
                if (whiteDone): self.hub.screen.load_image(Image('GUI/ColorCalibrateBoth.PNG'))
                else: self.hub.screen.load_image(Image('GUI/ColorCalibrateBlack.PNG'))
                blackDone = True

            if Button.RIGHT in self.hub.buttons.pressed() and not whiteDone:
                self.white = (self.colorSensor1.reflection() + self.colorSensor2.reflection()) / 2
                if (blackDone): self.hub.screen.load_image(Image('GUI/ColorCalibrateBoth.PNG'))
                else: self.hub.screen.load_image(Image('GUI/ColorCalibrateWhite.PNG'))
                whiteDone = True
              
            if Button.CENTER in self.hub.buttons.pressed() and not whiteDone:
                break
                
            if blackDone and whiteDone:
                sleep(1)
                break

        print("new black: " + str(self.black) + ", new white: " + str(self.white))

    #PURPOSE: Line follows until it finds a certain shade of B&W (0 is black, 100 is white)
    #PARAMS: 
    #p: https://youtu.be/AMBWV_HGYj4?t=212
    #DRIVE_SPEED: How fast the robot should go.
    #BLACK: Black to determine treshold: https://youtu.be/AMBWV_HGYj4?t=90.
    #WHITE: White to determine treshold: https://youtu.be/AMBWV_HGYj4?t=90.
    #SHADE: Shade to stop at.
    #sensor_lf: Sensor to line follow with.
    #sensor_stop: Sensor to determine when to stop.
    #debug: Turns on print statements to see what the sensor_lf is seeing.

    def lineFollowUntilShade(self, p=1.2, DRIVE_SPEED=100, SHADE=None, sensor_lf=None, sensor_stop=None, tolerance=3, debug=False):
        if (sensor_lf == None):
            sensor_lf = self.colorSensor2
        if (sensor_stop == None):
            sensor_stop = self.colorSensor1
        if (SHADE == None):
            print("ERROR: Please define the shade that you'll be using.")
        PROPORTIONAL_GAIN = p
        threshold = ((self.white - self.black)*0.7) #the average/mean of black+white

        while True: #forever, do
            if (debug):
                print(sensor_lf.reflection()) #how bright the stuff the color sensor sees is
            #Calculate the turn rate from the devation and set the drive base speed and turn rate.
            self.driveBase.drive(DRIVE_SPEED, PROPORTIONAL_GAIN * (sensor_lf.reflection() - threshold))
            
            #stop condition
            print(sensor_stop.reflection())
            if abs(sensor_stop.reflection() - SHADE) <= tolerance: 
                self.driveBase.stop()
                break

    #PURPOSE: Line follows until a timer finishes
    #PARAMS: 
    #p: https://youtu.be/AMBWV_HGYj4?t=212
    #DRIVE_SPEED: How fast the robot should go.
    #BLACK: Black to determine treshold: https://youtu.be/AMBWV_HGYj4?t=90.
    #WHITE: White to determine treshold: https://youtu.be/AMBWV_HGYj4?t=90.
    #sensor_lf: Sensor to line follow with.
    #time: How long the timer should go for in milliseconds (1/1000 of a second).
    #debug: Turns on print statements to see what the sensor_lf is seeing and to say how long the timer went for.
    def lineFollowForTime(self, p=1, DRIVE_SPEED=100, sensor_lf=None, time=10000, debug=False):
        if (sensor_lf == None):
            sensor_lf = self.colorSensor2
        self.stopWatch.reset()
        self.stopWatch.resume()

        PROPORTIONAL_GAIN = p
        #BLACK = 9 #what is black
        #WHITE = 85 #what is white, also what is life (42)
        threshold = ((self.white - self.black)*0.7) #the center of black+white

        while True: #forever, do

            if (debug):
                print(sensor_lf.reflection()) #how bright the stuff the color sensor sees is
            #Calculate the turn rate from the devation and set the drive base speed and turn rate.
            self.driveBase.drive(DRIVE_SPEED, PROPORTIONAL_GAIN * (sensor_lf.reflection() - threshold))
            
            #stop condition 
            if self.stopWatch.time() > time: 
                break 

        self.driveBase.stop()
        if debug:
            self.hub.speaker.say("I line followed for" + str(floor(time/1000)) + "seconds")
    
    #PURPOSE: Line follows until the robot has gone a certain distance
    #PARAMS: 
    #p: https://youtu.be/AMBWV_HGYj4?t=212
    #DRIVE_SPEED: How fast the robot should go.
    #BLACK: Black to determine treshold: https://youtu.be/AMBWV_HGYj4?t=90.
    #WHITE: White to determine treshold: https://youtu.be/AMBWV_HGYj4?t=90.
    #SHADE: Shade to stop at.
    #sensor_lf: Sensor to line follow with.
    #distance: Distance to stop at in millimeters (1/1000 of a meter or 25.4 millimeters per inch)
    #debug: Turns on print statements to see what the sensor_lf is seeing.
    def lineFollowForDistance(self, p=1, DRIVE_SPEED=100, sensor_lf=None, distance=10000, debug=False):
        if (sensor_lf == None):
            sensor_lf = self.colorSensor2
        PROPORTIONAL_GAIN = p
        # threshold = (self.black + self.white) / 2 #the center of black+white
        threshold = ((self.white - self.black)*0.7)
        startingDistance = self.driveBase.distance()
        while True: #forever, do
            if (debug):
                print(sensor_lf.reflection()) #how bright the stuff the color sensor sees is
            #Calculate the turn rate from the devation and set the drive base speed and turn rate.
            self.driveBase.drive(DRIVE_SPEED, PROPORTIONAL_GAIN * (sensor_lf.reflection() - threshold))
            
            #stop condition 
            if self.driveBase.distance()-startingDistance > distance: 
                break 
        self.driveBase.stop()

    #PURPOSE: Turns the robot.
    #PARAMS:
    #degrees: The number of degrees the robot turned.
    #speed: The speed the robot turns at in mm/s.
    def turn(self, degrees, speed=100, precision=2):
        turnMode = ""
        if (self.gyro3Drift):
            self.driveBase.turn(degrees)
        elif (not self.gyro3Drift):
            self.gyroscope3.reset_angle(0)

            if degrees < 0:
                self.driveBase.drive(0, -speed)
            else:
                self.driveBase.drive(0, speed)
            while True:
                if abs(degrees) - abs(self.gyroscope3.angle()) <= precision:
                    break
            
            self.driveBase.stop()
    
    def turnMotor(self, motor, degrees, speed, standard=True, precision=2):
        if standard:
            motor.run_target(abs(speed), degrees)
            motor.stop()
        else:
            motor.run(speed)
            if degrees < 0:
                motor.run(-speed)
            else:
                motor.run(speed)
            while True:
                if abs(degrees) - abs(motor.angle()) <= precision:
                    break
            motor.stop()
    
    def mmToInch(self, mm):
        return mm/25.4
    def inchToMM(self, inch):
        return inch*25.4
    def msToSecond(self, ms):
        return ms/1000
    def secondToMS(self, s):
        return s*1000
