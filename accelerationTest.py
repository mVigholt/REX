from time import sleep

import robot
import numpy as np
# Create a robot object and initialize
arlo = robot.Robot()

arlo.go_diff(30, 31, 1, -1)
sleep(0.1)
arlo.stop()