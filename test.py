import numpy as np
import time
import help as h

otto = h.Arlo()

cam = h.Cam()

speed = 60
error = 2
rotateSpeed = 31

while True:
  input()
  i = 4.33
  otto.arlo.go_diff(rotateSpeed-1, rotateSpeed, 0, 1)
  print("i = ", i)
  time.sleep(i)
  otto.arlo.stop()