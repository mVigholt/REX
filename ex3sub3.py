import cv2 # Import the OpenCV library
import time as t
import robot
import numpy as np
import itertools

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
    
X = 145
f = 1138

print("OpenCV version = " + cv2.__version__)

# Open a window
WIN_RF = "Example 1"
cv2.namedWindow(WIN_RF)
cv2.moveWindow(WIN_RF, 100, 100)

# load dictionary and parameters
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

# initialize camera transformation matrix
camera_matrix = np.array([[f, 0, capture_width/2],
                         [0, f, capture_height/2],
                         [0, 0, 1]])

# Assuming no lens distortion for now
distCoeffs = np.zeros((5, 1))

coordinates = []

while cv2.waitKey(4) == -1: # Wait for a key pressed event
    retval, frameReference = cam.read() # Read frame
    
    if not retval: # Error
        print(" < < <  Game over!  > > > ")
        exit(-1)
    
    corners, ids, _ = cv2.aruco.detectMarkers(frameReference, aruco_dict)
    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, X, camera_matrix, distCoeffs)
    
    ids_flattened = list(itertools.chain(*ids))
    tvecs_flattened = list(itertools.chain(*tvecs))
    
    print(ids_flattened)
    print("_____________")
    print(tvecs_flattened)
    break
      
      
        
    
    print(coordinates)
    
    cv2.imshow(WIN_RF, frameReference)  