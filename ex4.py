import cv2 # Import the OpenCV library
import time as t
import robot
import numpy as np
import itertools
import help
import rrt_mod as rt
import robot_models
import map as m

# Open a camera device for capturing
cam = help.Cam()

X = 145
f = 1138
ottoRadius = 25
boxRadius = 35

while cv2.waitKey(4) == -1: # Wait for a key pressed event
  ids, tvecs = cam.next_map()
  path_res = 0.05
  
  map = m.landmark_map(high=(200, 1000), landMarks=tvecs)
  
  robot = robot_models.PointMassModel(ctrl_range=[-path_res, path_res])
  
  rrt = rt.RRT(
    start=[0, 0],
    goal=[0, 1000],
    robot_model=robot,
    map=tvecs,
    expand_dis=0.2,
    path_resolution=path_res,
    )
  
  path = rrt.planning(animation=False)

  if path is None:
      print("Cannot find path")
  else:
      print("found path!!")
  # mangler alt rrt pisset
  