#!/usr/bin/env python3

from collections import deque, Counter
from movementLib import EV3Rover  # Import your EV3 movement library

class GestureToEV3Controller:
    def __init__(self):
        self.ev3_rover = EV3Rover(kp=1, ki=0, kd=0)
        self.confirmation_counter = Counter()

    def process_gesture(self, gesture):
        command = self.map_gesture_to_command(gesture)
        
        if command:
            self.handle_command(command)

    def map_gesture_to_command(self, gesture):
        command_map = {
            'hello': self.ev3_rover.greet,
            'left': lambda: self.ev3_rover.turn(90, direction='left'),
            'right': lambda: self.ev3_rover.turn(90, direction='right'),
            'forward': lambda: self.ev3_rover.drive_straight_mod(30),
            'reverse': lambda: self.ev3_rover.drive_straight_mod(30,m_speed=-30),
            'spin': lambda: self.ev3_rover.spin(300), #360
        }
        return command_map.get(gesture, None)

    def handle_command(self, command):
        self.execute_command(command)

    def execute_command(self, command):
        command()  # Execute the command
        print("Command executed ")

