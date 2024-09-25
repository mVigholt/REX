import cv2 # Import the OpenCV library
import time as t
import robot
import numpy as np
import itertools
import help

# Open a camera device for capturing
cam = help.Cam()

X = 145
f = 1138
ottoRadius = 25
boxRadius = 35

while cv2.waitKey(4) == -1: # Wait for a key pressed event
  ids, tvecs = cam.next_map()
  
  print(tvecs)
  