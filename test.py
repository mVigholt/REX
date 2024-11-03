import numpy as np
import time
import help as h
import math

otto = h.Arlo()

cam = h.Cam()

speed = 60
error = 2
rotateSpeed = 31
i = 4.0

while True:
  otto.Turn(math.pi)