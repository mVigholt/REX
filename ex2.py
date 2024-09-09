from time import sleep

import time as t
import robot
import threading
import numpy as np
# Create a robot object and initialize
arlo = robot.Robot()

print("Running ...")

speed = 60
error = 2

def GetSensorData():
  leftSensor = arlo.read_left_ping_sensor()
  frontSensor = arlo.read_front_ping_sensor()
  rightSensor = arlo.read_right_ping_sensor()
  return (leftSensor, frontSensor, rightSensor)

def DriveStraight():
  arlo.go_diff(60-error, 60, 1, 1)

# left: dir = 0
# right: dir = 1
def Rotate(dir):
  if dir != 0 and dir != 1:
    print(dir)
    print("dir has to be 1 or 0")
    arlo.stop()
    return
  
  if (int(dir) == 1):
    arlo.go_diff(60-error, 60, 1, 0)
  else:
    arlo.go_diff(60-error, 60, 0, 1)

def run():
  safetyDistance = 500
  starttime = t.time()
  
  while (t.time() - starttime < 20):
    data = GetSensorData()
    if (data[1] > safetyDistance and data[0] > safetyDistance and data[2] > safetyDistance):
      DriveStraight()
    else:
      if (data[0] > data[2]):
        Rotate(0)
      else:
        Rotate(1)
    sleep(0.1)

run()