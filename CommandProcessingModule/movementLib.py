#!/usr/bin/env python3
from ev3dev2.motor import MoveTank, OUTPUT_A, OUTPUT_D, SpeedDPS
from ev3dev2.sensor.lego import GyroSensor, UltrasonicSensor
from ev3dev2.sensor import INPUT_1, INPUT_4
from ev3dev2.sound import Sound

from time import sleep
from math import pi

from math import pi, inf
from time import sleep


RADIUS = 2.8
WHEELBASE = 12.5


class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.previous_error = 0

    def compute(self, target, current):
        error = target - current
        self.integral += error
        derivative = error - self.previous_error
        self.previous_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative


# PID enabled Base functions
class EV3Rover:

    def __init__(self, kp=0.8, ki=0, kd=1):
        """
        Initializes the EV3Rover with PID controller parameters.
        
        Parameters:
        - kp (float): Proportional gain for the PID controller.
        - ki (float): Integral gain for the PID controller.
        - kd (float): Derivative gain for the PID controller.
        """
        self.tank = MoveTank(OUTPUT_D, OUTPUT_A)
        self.gyro = GyroSensor(INPUT_1)
        self.ultrasonic = UltrasonicSensor(INPUT_4)
        self.sound = Sound()  # Initialize the Sound class
        self.pid = PIDController(kp, ki, kd)

    def drive_straight(self, dist, m_speed=400, obstacle_threshold=15):
        """
        Drives the robot straight for a specified distance using PID control,
        while checking for obstacles to avoid collisions.
        
        Parameters:
        - dist (float): The distance to travel in centimeters.
        - m_speed (int): The motor speed in degrees per second (default is 400).
        - obstacle_threshold (float): The distance in centimeters to stop if an obstacle is detected (default is 20cm).
        """
        # Calculate the degrees the motor must turn to cover the specified distance
        deg = 360 * (dist / (2 * pi * RADIUS))

        # Reset the gyro sensor to set the initial angle to zero
        self.gyro.reset()

        # Reset motor positions
        # self.tank.left_motor.position = 0
        # self.tank.right_motor.position = 0
        self.sound.beep() 
        # Drive the robot while checking for obstacles
        while True:
            # Check for obstacles
            if self.detect_obstacle(obstacle_threshold):
                print("Obstacle detected! Stopping.")
                self.tank.off()
                return

            # Adjust the motor speeds using PID control to maintain a straight path
            current_angle = self.gyro.angle
            correction = self.pid.compute(0, current_angle)
            self.tank.on(SpeedDPS(m_speed + correction), SpeedDPS(m_speed - correction))

            # Check if the desired distance has been covered
            degrees_turned_left = abs(self.tank.left_motor.position)
            degrees_turned_right = abs(self.tank.right_motor.position)
            if degrees_turned_left >= deg or degrees_turned_right >= deg:
                break

        # Stop the motors after reaching the target distance or detecting an obstacle
        self.tank.off()
        self.gyro.reset()

    def drive_straight_mod(self, dist, m_speed=30, obstacle_threshold=30):
        """
        Drives the robot straight for a specified distance using PID control,
        while checking for obstacles to avoid collisions.
        
        Parameters:
        - dist (float): The distance to travel in centimeters.
        - m_speed (int): The motor speed in degrees per second (default is 400).
        - obstacle_threshold (float): The distance in centimeters to stop if an obstacle is detected (default is 15cm).
        """
        # Calculate the degrees the motor must turn to cover the specified distance
        deg = 360 * (dist / (2 * pi * RADIUS))

        # Reset the gyro sensor to set the initial angle to zero
        self.gyro.reset()

        self.sound.beep()

        # Drive the robot while checking for obstacles
        while True:
            # Check for obstacles
            if self.detect_obstacle(obstacle_threshold):
                print("Obstacle detected! Stopping.")
                self.tank.off()
                return

            # Adjust the motor speeds using PID control to maintain a straight path
            current_angle = self.gyro.angle
            correction = self.pid.compute(0, current_angle)

            # Calculate left and right motor speeds with correction
            left_speed = m_speed + correction
            right_speed = m_speed - correction

            # Cap the left speed to ensure it stays within the range -100 to 100
            left_speed = max(-100, min(100, left_speed))

            # Apply the correction and drive for the calculated degrees
            self.tank.on_for_degrees(left_speed=left_speed,
                                    right_speed=right_speed,
                                    degrees=deg)

            # After driving for the calculated degrees, break out of the loop
            break

        # Stop the motors after reaching the target distance or detecting an obstacle
        self.tank.off()
        self.gyro.reset()


    def drive_perpetually(self, m_speed=400, obstacle_threshold=20):
        """
        Drives the robot forward indefinitely while checking for obstacles.
        The robot will stop if an obstacle is detected.
        
        Parameters:
        - m_speed (int): The motor speed in degrees per second (default is 400).
        - obstacle_threshold (float): The distance in centimeters to stop if an obstacle is detected (default is 20cm).
        """
        print("Starting perpetual drive...")

        # Reset the gyro sensor to set the initial angle to zero
        self.gyro.reset()

        # Reset motor positions
        self.tank.left_motor.position = 0
        self.tank.right_motor.position = 0

        while True:
            # Check for obstacles
            if self.detect_obstacle(obstacle_threshold):
                print("Obstacle detected! Stopping.")
                self.tank.off()
                self.gyro.reset()
                return

            # Adjust the motor speeds using PID control to maintain a straight path
            current_angle = self.gyro.angle
            correction = self.pid.compute(0, current_angle)
            self.tank.on(SpeedDPS(m_speed + correction), SpeedDPS(m_speed - correction))

            # Small sleep to avoid CPU overuse
            sleep(0.01)

    def turn(self, angle, m_speed=400, direction='right'):
        """
        Turns the robot by a specified angle using differential steering.
        
        Parameters:
        - angle (float): The angle to turn in degrees.
        - m_speed (int): The motor speed in degrees per second (default is 400).
        - direction (str): The direction of the turn, 'right' or 'left' (default is 'right').
        """
        # Calculate the degrees the motor must turn to achieve the specified angle
        deg = ( WHEELBASE / RADIUS ) * angle

        # Reset the gyro sensor to set the initial angle to zero
        self.gyro.reset()
        
        self.sound.beep() 
        self.sound.beep() 

        # Execute the turn based on the specified direction
        if direction == 'left':
            self.tank.on_for_degrees(left_speed=0, right_speed=SpeedDPS(m_speed), degrees=deg)
        else:
            self.tank.on_for_degrees(left_speed=SpeedDPS(m_speed), right_speed=0, degrees=deg)

        self.gyro.reset()

    def turnPID(self, angle, m_speed=400, direction='right'):
        """
        Turns the robot by a specified angle using PID control.
        
        Parameters:
        - angle (float): The angle to turn in degrees.
        - m_speed (int): The base motor speed in degrees per second (default is 400).
        - direction (str): The direction of the turn, 'right' or 'left' (default is 'right').
        """
        # Reset the gyro sensor to set the initial angle to zero
        self.gyro.reset()

        while True:
            # Calculate the error (difference between target angle and current angle)
            current_angle = self.gyro.angle + 30
            print("Current angle; ",  self.gyro.angle)
            error = angle - abs(current_angle)
            
            # Break if the target angle is reached
            if 20 > error >-20 :
                break
            
            # Calculate correction using PID control
            correction = self.pid.compute(angle, abs(current_angle))
            print("Correction ; ",  correction)

            if direction == 'left':
                self.tank.on(SpeedDPS(m_speed - correction), 0)
            else:
                self.tank.on(0, SpeedDPS(m_speed - correction))

            # Small sleep to avoid CPU overuse
            sleep(0.01)

        # Stop the motors after reaching the target angle
        self.tank.off()
        self.gyro.reset()

    def spinPID(self, angle, m_speed=400, direction="right"):
        """
        Spins the robot in place by a specified angle using PID control.
        
        Parameters:
        - angle (float): The angle to spin in degrees.
        - m_speed (int): The base motor speed in degrees per second (default is 400).
        - direction (str): The direction of the spin, 'right' or 'left' (default is 'right').
        """
        # Reset the gyro sensor to set the initial angle to zero
        self.gyro.reset()

        while True:
            # Calculate the error (difference between target angle and current angle)
            current_angle = self.gyro.angle + 20
            error = angle - abs(current_angle)
            
            # Break if the target angle is reached
            if 20 > error >-20 :
                break
            
            # Calculate correction using PID control
            correction = self.pid.compute(angle, abs(current_angle))

            if direction == 'left':
                self.tank.on(SpeedDPS(-m_speed + correction), SpeedDPS(m_speed - correction))
            elif direction == 'right':
                self.tank.on(SpeedDPS(m_speed - correction), SpeedDPS(-m_speed + correction))

            # Small sleep to avoid CPU overuse
            sleep(0.01)

        # Stop the motors after completing the spin
        self.tank.off()
        self.gyro.reset()

    def spin(self, angle, m_speed=400, direction="right"):
        """
        Spins the robot in place by a specified angle.
        
        Parameters:
        - angle (float): The angle to spin in degrees.
        - m_speed (int): The motor speed in degrees per second (default is 400).
        - direction (str): The direction of the spin, 'right' or 'left' (default is 'right').
        """
        # Calculate the degrees the motor must turn to achieve the specified angle

        deg = ( (WHEELBASE/2) / RADIUS ) * angle


        # Reset the gyro sensor to set the initial angle to zero
        self.gyro.reset()

        
        self.sound.beep() 

        # Execute the spin based on the specified direction
        if direction == 'left':
            self.tank.on_for_degrees(left_speed=SpeedDPS(-m_speed), right_speed=SpeedDPS(m_speed), degrees=deg)
        elif direction == 'right':
            self.tank.on_for_degrees(left_speed=SpeedDPS(m_speed), right_speed=SpeedDPS(-m_speed), degrees=deg)

        # Stop the motors after completing the spin
        self.tank.off()

    def detect_obstacle(self, distance_threshold=20):
        """
        Detects an obstacle within the specified distance threshold.
        
        Parameters:
        - distance_threshold (float): The distance in centimeters below which an obstacle is considered detected.
        
        Returns:
        - bool: True if an obstacle is detected within the threshold distance, False otherwise.
        """
        # Measure the distance to the nearest object using the ultrasonic sensor
        distance = self.ultrasonic.distance_centimeters
        # Return True if the distance is less than the threshold, indicating an obstacle
        if distance < distance_threshold:
            self.sound.beep() 
            self.sound.beep() 
            return True
        else:
            return False

    def stop(self):
        """
        Stops the robot by turning off the motors.
        """
        self.tank.off()
        self.sound.beep() 

    def greet(self):
            """
            Performs a greeting by spinning 65 degrees left and right, and playing a sound.
            """
            # Spin left 65 degrees
            self.spin(65, m_speed=400, direction="left")
            # Spin right 65 degrees
            self.spin(65, m_speed=400, direction="right")
            # Play the greeting audio file
            self.sound.speak('Hello')

    def drive_reverse(self, dist, m_speed=400, obstacle_threshold=15):
        """
        Drives the robot in reverse for a specified distance using PID control,
        while checking for obstacles to avoid collisions.

        Parameters:
        - dist (float): The distance to travel in centimeters.
        - m_speed (int): The motor speed in degrees per second (default is 400).
        - obstacle_threshold (float): The distance in centimeters to stop if an obstacle is detected (default is 15cm).
        """
        # Calculate the degrees the motor must turn to cover the specified distance
        deg = 360 * (dist / (2 * pi * RADIUS))

        # Reset the gyro sensor to set the initial angle to zero
        self.gyro.reset()

        # Reset motor positions
        # self.tank.left_motor.position = 0
        # self.tank.right_motor.position = 0
        self.sound.beep() 
        
        # Drive the robot while checking for obstacles
        while True:
            # Check for obstacles
            if self.detect_obstacle(obstacle_threshold):
                print("Obstacle detected! Stopping.")
                self.tank.off()
                return

            # Adjust the motor speeds using PID control to maintain a straight path
            current_angle = self.gyro.angle
            correction = self.pid.compute(0, current_angle)
            self.tank.on(SpeedDPS(-m_speed + correction), SpeedDPS(-m_speed - correction))

        
            # Check if the desired distance has been covered
            degrees_turned_left = abs(self.tank.left_motor.position)
            degrees_turned_right = abs(self.tank.right_motor.position)
            if degrees_turned_left >= deg or degrees_turned_right >= deg:
                break

        # Stop the motors after reaching the target distance or detecting an obstacle
        self.tank.off()
        self.gyro.reset()



# Base functions 

def drive_straight(dist, tank, m_speed=400 ):
    """
    A function that takes two parameters, dist (distance in centimeters) and m_speed (motor speed
    between 0 and 1000) and enables the robot to drive straight at the given motor speed and for the 
    given distance. Forward: positive motor speed. Backward: negative motor speed.
    """
    if m_speed >= 0 and m_speed <= 1000:

        # Calculating the degrees the motor turns to cover the given distance
        deg = 360 * (dist / (2 * pi * RADIUS))

        # Moving the robot at speed m_speed for deg degrees
        tank.on_for_degrees(left_speed=SpeedDPS(m_speed), right_speed=SpeedDPS(m_speed), degrees=deg)
        print('Done!')
    else:
        print('Speed is not within the range')
        print('Speed must be between 0 and 1000')
    return


def turn(angle, tank, m_speed=400,direction='right'):
    """
    A function that takes two parameters, angle (angle in degrees) and m_speed (motor speed
    between 0 and 1000) and enables the robot to turn at the given motor speed and for the 
    given angle. 
    """
    if m_speed >= 0 and m_speed <= 1000:
        # Calculating the degrees the motor turns to turn the robot by the given angle
        deg = WHEELBASE / RADIUS * angle

        if direction == 'left':
            # turning robot left by angle
            tank.on_for_degrees(left_speed=0, right_speed=SpeedDPS(m_speed), degrees=deg)
        else:
            # # turning robot right by angle
            tank.on_for_degrees(left_speed=SpeedDPS(m_speed), right_speed=0, degrees=deg)
    else:
        print('Speed is not within the range')
        print('Speed must be between 0 and 1000')
    return


def gyroStraight(dist, m_speed, gyro, tank):
    gyro.reset()

    drive_straight(dist, m_speed)

    while tank.is_running:
        current_angle = gyro.angle
        # Calculate the error (difference between target angle and current angle)
        error = 0 - current_angle
        if current_angle < 0:
            turn(error, 50)
        elif current_angle > 0:
            turn(error, 50, direction='left')


def spin(angle, tank, m_speed=400, direction="right" ):
    """
    A function that takes three parameters angle, speed and direction and spins the robot 
    through the given angle at the given motor speed in the given direction.
    """
    if m_speed >= 0 and m_speed <= 1000:
        # Calculating the degrees the motor turns to turn the robot by the given angle
        deg = WHEELBASE / (RADIUS * 2) * angle

        if direction == 'left':
            # spinning the robot left(anticlockwise) by angle
            tank.on_for_degrees(left_speed=SpeedDPS(-m_speed), right_speed=SpeedDPS(m_speed), degrees=deg)
        elif direction == 'right':
            # spinning the robot left(clockwise) by angle
            tank.on_for_degrees(left_speed=SpeedDPS(m_speed), right_speed=SpeedDPS(-m_speed), degrees=deg)
    else:
        print('Speed is not within the range')
        print('Speed must be between 0 and 1000')
    return


def drive_perpetually(s, tank):
    """Function to drive tank forward for 30 seconds"""
    tank.on(SpeedDPS(s), SpeedDPS(s))
    sleep(0.01)
    return


def stop(tank):
    """Function to stop the robot"""
    tank.off()
    return
