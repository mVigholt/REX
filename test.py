import numpy as np
import time
import help as h

otto = h.Arlo()

cam = h.Cam()

speed = 60
error = 2
rotateSpeed = 31
i = 5

while True:
  input()
  i += 0.5
  otto.arlo.go_diff(speed-error, speed, 1, 1)
  print("i = ", i)
  time.sleep(i)
  otto.arlo.stop()