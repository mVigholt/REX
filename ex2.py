from time import sleep

import robot
import numpy as np
# Create a robot object and initialize
arlo = robot.Robot()

print("Running ...")

leftSensor = arlo.read_left_ping_sensor
frontSensor = arlo.read_front_ping_sensor
rightSensor = arlo.read_right_ping_sensor

print(leftSensor)
print(frontSensor)
print(rightSensor)