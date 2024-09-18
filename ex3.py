# This script shows how to open a camera in OpenCV and grab frames and show these.
# Kim S. Pedersen, 2022

import cv2 # Import the OpenCV library
import time as t
import robot

# Create a robot object and initialize
arlo = robot.Robot()

rotateSpeed = 30
speed = 60
error = 2
safetyStraightDistance = 500
safetySideDistance = 400



def gstreamer_pipeline(capture_width=1024, capture_height=720, framerate=30):
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
    
def calc_distance(real_size, pixel_size):
    f = 145
    return (f*real_size)/pixel_size

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
    arlo.go_diff(rotateSpeed, rotateSpeed, 1, 0)
  else:
    arlo.go_diff(rotateSpeed, rotateSpeed, 0, 1)

print("OpenCV version = " + cv2.__version__)
# Open a camera device for capturing
cam = cv2.VideoCapture(gstreamer_pipeline(), apiPreference=cv2.CAP_GSTREAMER)


if not cam.isOpened(): # Error
    print("Could not open camera")
    exit(-1)

# Open a window
WIN_RF = "Example 1"
cv2.namedWindow(WIN_RF)
cv2.moveWindow(WIN_RF, 100, 100)

# load dictionary and parameters
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

Rotate(1)

ctime = t.time()

while cv2.waitKey(4) == -1: # Wait for a key pressed event
    retval, frameReference = cam.read() # Read frame
    
    if not retval: # Error
        print(" < < <  Game over!  > > > ")
        exit(-1)
    
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(frameReference, aruco_dict)

    
    # Draw markers on the frame if found
    frameReference = cv2.aruco.drawDetectedMarkers(frameReference, corners, ids)
        
    if (len(corners) > 0):
        break
    
    # [0][0][0] = top left corner
    # [0][1][0] = bottom left corner
    
    # if corners:
        # distance = calc_distance(145, corners[0][0][1][0] - corners[0][0][0][0])
        
    # cv2.aruco.estimatePoseSingleMarkers(corners, 145, )
    
    # print(distance)
    # print(corners, '\n')
    
    # Show frames
    cv2.imshow(WIN_RF, frameReference)

    #t.sleep(1)

cv2.imwrite("OttosView.png", frameReference)
exit(-1)
# Finished successfully
