#!/usr/bin/env python3

from ev3dev2.motor import MoveTank, OUTPUT_A, OUTPUT_D, SpeedDPS
from ev3dev2.sensor.lego import GyroSensor, UltrasonicSensor
from ev3dev2.sensor import INPUT_1, INPUT_4
from time import sleep
from math import pi

from movementLib import EV3Rover, drive_straight, turn, gyroStraight, spin, drive_perpetually, stop

# Initialize the EV3Rover
rover = EV3Rover(kp=1, ki=0, kd=0)

from time import sleep

def test_drive_straight():
    print("Testing drive_straight...")
    rover.drive_straight(dist=20, m_speed=400)
    print("drive_straight test completed.\n")

def test_perpetual_drive_straight():
    print("Testing drive_straight...")
    rover.drive_perpetually(m_speed=400, obstacle_threshold=20)
    print("drive_straight test completed.\n")
     

def test_turn():
    print("Testing turn...")
    rover.turn(angle=90, m_speed=400, direction='left')
    print("turn test completed.\n")

def test_spin():
    print("Testing spin...")
    rover.spin(angle=180, m_speed=400, direction='right')
    print("spin test completed.\n")

def test_detect_obstacle():
    print("Testing detect_obstacle...")
    if rover.detect_obstacle():
        print("Obstacle detected!")
    else:
        print("No obstacle detected.")
    print("detect_obstacle test completed.\n")

def test_stop():
    print("Testing stop...")
    rover.stop()
    print("stop test completed.\n")



def run_tests():

    test_turn()

    sleep(10)

    test_spin()
  



if __name__ == '__main__':
    run_tests()


### PID tunning
