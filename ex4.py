import cv2 # Import the OpenCV library
import time as t
import robot
import numpy as np
import itertools
import help
import rrt as rt
import robot_models
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
import grid_occ

# Open a camera device for capturing
cam = help.Cam()

X = 145
f = 1138
ottoRadius = 25
boxRadius = 35

while cv2.waitKey(4) == -1: # Wait for a key pressed event
  ids, tvecs = cam.next_map()
  path_res = 0.05

  robot = robot_models.PointMassModel(ctrl_range=[-path_res, path_res])
  
  rrt = rt.RRT(
    start=[0, 0],
    goal=[0, 100],
    robot_model=robot,
    map=tvecs,
    expand_dis=0.2,
    path_resolution=path_res,
    )
  
  # mangler alt rrt pisset
  