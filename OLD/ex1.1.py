from time import sleep

import robot
import numpy as np
# Create a robot object and initialize
arlo = robot.Robot()

print("Running ...")

# Measurements
tickPrRevolution = 144
wheelToWheelDistance = 40
wheelDiameter = 15

# Other variables
speed = 64
overshoot = 0
cmPrTick = (wheelDiameter * np.pi) / tickPrRevolution
driveStraight = (100 / cmPrTick) - overshoot
rotate = ((wheelToWheelDistance * np.pi) / (4 * cmPrTick)) - overshoot

straightTime = 2
rotateTime = 2

# start driving pattern
for i in range(0, 1):
    
    print(arlo.go_diff(speed, speed, 1, 1))
    sleep(straightTime)
    print(arlo.stop())
    sleep(0.5)
    print(arlo.go_diff(speed, speed, 1, 0))
    sleep(rotateTime)
    print(arlo.stop())
    sleep(0.5)
    