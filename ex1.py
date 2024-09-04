from time import sleep

import robot
import numpy as np
# Create a robot object and initialize
arlo = robot.Robot()

print("Running ...")

# Measurements
tickPrRevolution = 36
wheelToWheelDistance = 40
wheelDiameter = 15

# Other variables
speed = 64
overshoot = 0
cmPrTick = (wheelDiameter * np.pi) / tickPrRevolution
driveStraight = (100 / cmPrTick) - overshoot
rotate = ((wheelToWheelDistance * np.pi) / (4 * cmPrTick)) - overshoot

# start driving pattern
for i in range(0, 4):
    
    arlo.reset_encoder_counts
    sleep(0.5)

    # send a go_diff command to drive forward
    print(arlo.go_diff(speed, speed, 1, 1))

    while(int.from_bytes(arlo.read_left_wheel_encoder) < driveStraight):
        sleep(0.5)

    # send a stop command
    print(arlo.stop())

    arlo.reset_encoder_counts
    sleep(0.5)
    
    # send a go_diff command to drive forward
    print(arlo.go_diff(speed, speed, 1, -1))

    while(int.from_bytes(arlo.read_left_wheel_encoder) < rotate):
        sleep(0.5)

    # send a stop command
    print(arlo.stop())
    