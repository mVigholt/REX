import numpy as np
import time
import help as h

otto = h.Arlo()

cam = h.Cam()

speed = 60
error = 1
rotateSpeed = 31
i = 0

while True:
  input()
  i += 0.1
  otto.arlo.go_diff(speed-error, speed, 1, 1)
  print("i = ", i)
  time.sleep(i)