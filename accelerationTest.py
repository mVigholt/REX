from time import sleep

import robot
import numpy as np
import cv2
# Create a robot object and initialize
arlo = robot.Robot()

i = 0

while 1 == 1:
  
  i += 0.1
  input(f"you are testing i = {i}. Waiting for input")
  arlo.go_diff(30, 31, 1, -1)
  sleep(i)
  arlo.stop()