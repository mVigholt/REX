from time import sleep

import robot
import numpy as np
import cv2
import help
# Create a robot object and initialize
arlo = help.Arlo()

i = 0

while 1 == 1:
  
  i += 0.1
  input(f"you are testing i = {i}. Waiting for input")
  arlo.Right()
  sleep(i)
  arlo.Stop()