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

# load dictionary and parameters
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

# initialize camera transformation matrix
camera_matrix = np.array([[f, 0, help.cw/2],
                         [0, f, help.ch/2],
                         [0, 0, 1]])


def getCoordinates(frameReference, aruco_dict, camera_matrix, X):
  # Assuming no lens distortion for now
  distCoeffs = np.zeros((5, 1))
  coordinates = []
  corners, ids, _ = cv2.aruco.detectMarkers(frameReference, aruco_dict)
  _, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, X, camera_matrix, distCoeffs)

  ids_flattened = list(itertools.chain(*ids))
  tvecs_flattened = list(itertools.chain(*tvecs))
  
  return ids_flattened, [tvecs_flattened[0], tvecs_flattened[2]]

while cv2.waitKey(4) == -1: # Wait for a key pressed event
  
  retval, frameReference = cam.read() # Read frame
  
  if not retval: # Error
      print(" < < <  Game over!  > > > ")
      exit(-1)
  
  ids, tvecs = getCoordinates(frameReference, aruco_dict, camera_matrix, X)
  
  print(tvecs)
  