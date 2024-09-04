from time import sleep

import robot
import numpy as np
# Create a robot object and initialize
arlo = robot.Robot()

print("Running ...")

speed = 64

arlo.go_diff(speed-3, speed/2, 1, 1)
sleep(3)
arlo.stop()