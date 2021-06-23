#!/usr/bin/env pybricks-micropython
#----import ev3 librairies and stuff
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
import time

#----Setting objects here----
ev3 = EV3Brick() #EV3 brick
r_m = Motor(Port.A) #Right propulsion motor
l_m = Motor(Port.B) #Left propulsion motor
m1_m = Motor(Port.C) #Right middle motor
m2_m = Motor(Port.D) #Left Middle Motor
gyro_sensor = GyroSensor(Port.S4) #Gyro Sensor Port.4
center_color_sensor = ColorSensor(Port.S1) #Center color sensor Port.1
right_color_sensor = ColorSensor(Port.S3)  #Right color sensor Port.3
left_color_sensor = ColorSensor(Port.S2)   #Left color sensor Port.2
Alpha = DriveBase(l_m, r_m, wheel_diameter=55.5, axle_track=96) #Setting Alpha
Alpha.settings(straight_speed=400) #Setting speed


#----------------------------------------------------
def centerLineFollower(DRIVE_SPEED,sec,BLACK,WHITE,pn,prop):
    PROPORTIONAL_GAIN = prop
    threshold = (BLACK + WHITE) / 2
    timeout = sec  # [seconds]
    timeout_start = time.time()
    negative_or_positive = pn
    while time.time() < timeout_start + timeout:
        # Calculate the deviation from the threshold.
        deviation = center_color_sensor.reflection() - threshold
        # Calculate the turn rate
        turn_rate = PROPORTIONAL_GAIN * deviation
        # Set the drive base speed and turn rate.
        Alpha.drive(DRIVE_SPEED, turn_rate)
        # You can wait for a short time or do other things in this loop.
        wait(20)  
    return  
#---------------------------------------------------------------------------------
def leftLineFollower(DRIVE_SPEED,sec,BLACK,WHITE,pn,prop):
    PROPORTIONAL_GAIN = prop
    threshold = (BLACK + WHITE) / 2
    timeout = sec  # [seconds]
    timeout_start = time.time()
    negative_or_positive = pn
    while time.time() < timeout_start + timeout:
        # Calculate the deviation from the threshold.
        deviation = left_color_sensor.reflection() - threshold
        # Calculate the turn rate.
        turn_rate = pn * PROPORTIONAL_GAIN * deviation
        # Set the drive base speed and turn rate.
        Alpha.drive(DRIVE_SPEED, turn_rate)
        # You can wait for a short time or do other things in this loop.
        wait(20)      
    return
#---------------------------------------------------------------------------------
def rightLineFollower(DRIVE_SPEED,sec,BLACK,WHITE,pn,prop):
    PROPORTIONAL_GAIN = prop
    threshold = (BLACK + WHITE) / 2
    timeout = sec  # [seconds]
    timeout_start = time.time()
    negative_or_positive = pn
    while time.time() < timeout_start + timeout:
        # Calculate the deviation from the threshold.
        deviation = right_color_sensor.reflection() - threshold
        # Calculate the turn rate.
        turn_rate = pn * PROPORTIONAL_GAIN * deviation
        # Set the drive base speed and turn rate.
        Alpha.drive(DRIVE_SPEED, turn_rate)
        # You can wait for a short time or do other things in this loop.
        wait(20)
    return
#--------------------------------------------------------------------------------
def gyroStraight(speed, distance):
    gyro_sensor.reset_angle(0)
    print(Alpha.distance())
    while Alpha.distance() >= distance:
        print(Alpha.distance())
        correction = (0 - gyro_sensor.angle())*3
        Alpha.drive(speed, correction)
    Alpha.stop()
    l_m.brake()
    r_m.brake()
#----------------------------------------------------------------------------------------
def gyroTurnLeft(aw, speed):
    # aw est angle wanted 
    gyro_sensor.reset_angle(0)
    while gyro_sensor.angle() >= aw : 
        l_m.run(speed)
        print(gyro_sensor.angle())
    Alpha.stop()
    l_m.brake()
    r_m.brake()
#-------------------------------------------------
def gyroTurnRight(aw, speed):
    # aw est angle wanted 
    gyro_sensor.reset_angle(0)
    while gyro_sensor.angle() <= aw : 
        r_m.run(speed)
    Alpha.stop()
    l_m.brake()
    r_m.brake()
#-----------------------------------------------------------
def lineSquaring() :
    stop1 = True
    stop2 = True
    def func1():
        if line_sensor_left.reflection()<20:
            r_m.stop()
            stop1=False
        else : 
            r_m.run(-50)
    def func2():
        if line_sensor_right.reflection()<20:
            l_m.stop()
            stop1=False
        else : 
            l_m.run(-50)
    stop1 = True
    stop2 = True
    while stop1 and stop2:
        Thread(target = func1).start()
        Thread(target = func2).start()

#----------------------------------------------------------





#----Programs-------------------------------------------------
while True :
    if Button.LEFT in ev3.buttons.pressed(): # L5
        Alpha.reset()
        wait(500)
        # Avancer jusqu'au podometre
        Alpha.straight(-1200)#4150
        # attendre 
        wait(500)
        ev3.speaker.beep()
        # retourner
        Alpha.drive_time(140, 2, 150)
        # avancer
        #Alpha.drive_time(-200, 6, 900)
        # attendre 
        wait(300)
        # tourner à gauche  
        Alpha.turn(-105)
        # qualibrage 
        Alpha.drive_time(400, 0, 900)
        #avancer 
        Alpha.drive_time(-400, 0, 1500)
        #avancer en tournant 
        Alpha.drive_time(-400, -40, 1700)
        ev3.speaker.beep()
        while True:
            m1_m.run_time(-400, 1400)
            m1_m.run_time(400, 1400)
#-------------------------------------------------------------
    if Button.RIGHT in ev3.buttons.pressed(): # L3
        Alpha.reset()
        wait(500)
        # sound effect
        ev3.speaker.beep()
        #avancer jusqu'a étre parallèle à la roue noir
        gyroStraight(-400,-1250)
        #baisser le medium motor pour faire descendre la figurine
        m2_m.run_time(500,3200)
        # monter le medium motor
        m2_m.run_time(-1000,1900)
        #avancer un peu
        gyroStraight(-400, -1400)
        #tourner le medium moteur pour prendre la healt unit
        m1_m.run_time(100, 2500)
        #avancer jusqu'au rameur
        gyroStraight(-400, -1640)
        # tourner la roue pour realiser la mission
        m1_m.run_time(1000,7500)
        #  baisser le medium motor pour attraper la roue
        m2_m.run_time(1000,1650)
        # reculer un peu
        Alpha.drive_time(100, 0, 1500)
        #tourner un peu
        Alpha.turn(-100)
        #lever le medium motor pour relacher la roue
        m2_m.run_time(-1000, 2400)
        #tourner un peu pour se remettre dans l'axe
        Alpha.turn(35)
        # reculer jusqu'a la base
        Alpha.drive_time(400,0,7000)

#-------------------------------------------------------------
    if Button.DOWN in ev3.buttons.pressed(): #L4
        Alpha.reset()
        wait(500)
        # sound effect
        ev3.speaker.beep()
        # Avancer
        Alpha.straight(-380)
        # Tourner
        Alpha.turn(10)
        # Avancer
        Alpha.drive_time(-100,0,1000)
        Alpha.drive_time(-50,0,1000)
        # Reculer
        Alpha.straight(50)
        # Avancer
        Alpha.straight(-100)
        # Reculer
        Alpha.straight(50)
        # Avancer
        Alpha.straight(-100)
        # Reculer
        Alpha.straight(50)
        # Avancer
        Alpha.straight(-100)
        # Reculer
        Alpha.straight(520)

#-------------------------------------------------------------
    if Button.UP in ev3.buttons.pressed():#L2
        Alpha.reset()
        wait(500)
        # Go forward 
        gyroStraight(-500, -700)
        ev3.speaker.beep()
        # moteur se baisse
        m1_m.run_time(400, 500)
        wait(300)
        Alpha.drive_time(150,0 , 3000)
        # turn
        Alpha.turn(240)
        # reculer
        Alpha.drive_time(400,0 , 1950)
#-------------------------------------------------------------
    if Button.CENTER in ev3.buttons.pressed():#L1
        Alpha.reset()
        wait(500)
        # sound effect
        ev3.speaker.beep()
        #avancer jusqu'a la cible
        gyroStraight(-400,-920)
        #baisser le medium motor une fois lentement puis plus rapidement
        m2_m.run_time(500,500)
        m2_m.run_time(1000,3500)
        m2_m.run_time(-1000,4000)
        #reculer un peu
        Alpha.drive_time(200, 0, 1550)
        #tourner a gauche
        gyroTurnLeft(-85, -300)
        Alpha.drive_time(-200, 0, 1000)
        #monter le medium motor (celui de droite) 
        m1_m.run_time(1000,500)
        #attendre
        wait(100)
        #monter le medium motor (celui de droite)
        m1_m.run_time(1000,500)
        #attendre
        wait(100)
        #monter le medium motor (celui de droite)
        m1_m.run_time(1000,500)
        #attendre
        wait(100)
        #monter le medium motor (celui de droite)
        m1_m.run_time(1000,500)
        #attendre
        wait(100)
        #monter le medium motor (celui de droite)
        m1_m.run_time(1000,500)
        #attendre
        wait(100)
        #baisser le medium motor (celui de droite)
        m1_m.run_time(-1000,500)
        Alpha.drive_time(200, 0, 800)
        gyroTurnLeft(-30, -200)
        Alpha.drive_time(-400, 0, 3000)
        

#-------------------------------------------------------------
        
