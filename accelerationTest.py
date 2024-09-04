from time import sleep

import robot
import numpy as np
# Create a robot object and initialize
arlo = robot.Robot()

arlo.go_diff(60-3, 60, 1, 1)
sleep(1)