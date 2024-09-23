# This script shows how to open a camera in OpenCV and grab frames and show these.
# Kim S. Pedersen, 2022

import cv2 # Import the OpenCV library
import time as t
import robot
import numpy as np
import glob

# Create a robot object and initialize
arlo = robot.Robot()

rotateSpeed = 40
speed = 60
error = 2
safetyStraightDistance = 500
safetySideDistance = 400
capture_width = 1024
capture_height = 720


def gstreamer_pipeline(capture_width=capture_width, capture_height=capture_height, framerate=2):
    """Utility function for setting parameters for the gstreamer camera pipeline"""
    return (
        "libcamerasrc !"
        "videobox autocrop=true !"
        "video/x-raw, width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "videoconvert ! "
        "appsink"
        % (
            capture_width,
            capture_height,
            framerate,
        )
    )

# Open a camera device for capturing
cam = cv2.VideoCapture(gstreamer_pipeline(), apiPreference=cv2.CAP_GSTREAMER)

if not cam.isOpened(): # Error
    print("Could not open camera")
    exit(-1)
 
 
##-----------------------------------------------------------------------------------------   
X = 145
f = 1138

def calc_distance(x):
    return (X*f)/x
  
def calibratecamera():
    # 3D points in real world space (the position of the markers relative to the marker origin)
  # This assumes the marker is on a flat plane (z = 0), and each marker is known to be 0.05 meters on each side.
  objp = np.array([[0, 0, 0], [X, 0, 0], [X, X, 0], [0, X, 0]], dtype=np.float32)

  # Arrays to store object points and image points from all images
  objpoints = []  # 3D points in real world space
  imgpoints = []  # 2D points in image plane

  # Load the calibration images (images that contain ArUco markers at different positions/angles)
  images = glob.glob('../images/REX/*.png')

  for fname in images:
      img = cv2.imread(fname)
      gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
      
      # Detect ArUco markers
      corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict)
      
      if len(corners) > 0:
          # For each marker, store the object points and image points
          for marker_corner in corners:
              imgpoints.append(marker_corner)
              objpoints.append(objp)

  # Perform camera calibration
  ret, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(
      objpoints, imgpoints, gray.shape[::-1], None, None)

  return ret, cameraMatrix, distCoeffs, rvecs, tvecs

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
  
  if (dir == 1):
    arlo.go_diff(rotateSpeed-error, rotateSpeed, 1, 0)
  else:
    arlo.go_diff(rotateSpeed-error, rotateSpeed, 0, 1)

print("OpenCV version = " + cv2.__version__)


# Open a window
WIN_RF = "Example 1"
cv2.namedWindow(WIN_RF)
cv2.moveWindow(WIN_RF, 100, 100)

# load dictionary and parameters
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

ret, cameraMatrix, distCoeffs, rvecs, tvecs = calibratecamera()

Rotate(1)

while cv2.waitKey(4) == -1: # Wait for a key pressed event
    arlo.stop()
    starttime = t.time()
  
    cam.read()  
    retval, frameReference = cam.read() # Read frame
    
    if not retval: # Error
        print(" < < <  Game over!  > > > ")
        exit(-1)
    
    corners, ids, _ = cv2.aruco.detectMarkers(frameReference, aruco_dict)
    
    # if (len(corners) > 0):
    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, f, camera_matrix, distCoeffs)
        # for i in range(len(ids)):
        #   cv2.aruco.drawAxis(frameReference, camera_matrix, distCoeffs, rvecs[i], tvecs[i], f / 2)
    print(rvecs, '\n')
    print("-----------------------------------------\n")
    print(tvecs)
        # print("stop")
        # arlo.stop()
        # break
    
    # Draw markers on the frame if found
    frameReference = cv2.aruco.drawDetectedMarkers(frameReference, corners, ids)
    print(t.time() - starttime, "\n")
        
    # Rotate(1)
    t.sleep(0.3)

    # Stream frames
    cv2.imshow(WIN_RF, frameReference)    

cv2.imwrite("OttosView.png", frameReference)
# Finished successfully
