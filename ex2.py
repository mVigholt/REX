from time import sleep

import robot
import threading
import numpy as np
# Create a robot object and initialize
arlo = robot.Robot()

print("Running ...")

speed = 60
error = 3

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
  if (dir != 0 & dir != 1):
    print("dir has to be 1 or 0")
    return
  
  if (dir == 1):
    arlo.go_diff(60-error, 60, 1, 0)
  else:
    arlo.go_diff(60-error, 60, 0, 1)

def run():
  # DriveStraight()
  data = GetSensorData()
  print(data)
  
  

run()