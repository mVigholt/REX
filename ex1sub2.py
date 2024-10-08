from time import sleep

import robot
import numpy as np
# Create a robot object and initialize
arlo = robot.Robot()

print("Running ...")

speed = 60
k=5
time = 0
for i in range(0, 2*k):

  if not (i % 2):
    if i == 0:
      time = 6.45
    else:
      time = 6.2
    arlo.go_diff(speed*2-3, speed, 1, 1)
  else:
    arlo.go_diff(speed-3, speed*2, 1, 1)
    time = 6.3
  
  sleep(time)

arlo.stop()
