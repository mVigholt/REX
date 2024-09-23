import cv2 # Import the OpenCV library
import numpy as np
import robot

cw = 1024
ch = 720
fr = 5
def openCam():
    def gstreamer_pipeline(capture_width=cw, capture_height=ch, framerate=fr):
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
    print("OpenCV version = " + cv2.__version__)
    return cam  

openWindow = False
def streamCam(frameReference, corners, ids):
    if not openWindow:
        # Open a window
        WIN_RF = "Example 1"
        cv2.namedWindow(WIN_RF)
        cv2.moveWindow(WIN_RF, 100, 100)
        openWindow = True
    
    # Draw markers on the frame if found
    frameDrawing = cv2.aruco.drawDetectedMarkers(frameReference, corners, ids)
    # Stream frames
    cv2.imshow(WIN_RF, frameDrawing) 
    

def distAndDir(corners):
    X = 145
    f = 1138
    #TODO find out which corner is which
    ul = corners[0]
    ur = corners[1]
    ll = corners[2]
    lr = corners[3]
    x =  (np.linalg.norm(ul - ll) + np.linalg.norm(ur - lr)) / 2
    Z = (f/x) * X
    
    xPos = (np.linalg.norm(ul - ur) + np.linalg.norm(ll - lr)) / 2
    dir = 1 if xPos > cw/2 else 0
    
    return Z, dir

##------------------------------------------------------------------------------------------
# Create a robot object and initialize
arlo = robot.Robot()
rotateSpeed = 40
speed = 60
error = 2
safetyStraightDistance = 500
safetySideDistance = 400

def Go():
  arlo.go_diff(speed-error, speed, 1, 1)

def Left():
    arlo.go_diff(rotateSpeed-1, rotateSpeed, 0, 1)
    
def Right():
    arlo.go_diff(rotateSpeed-1, rotateSpeed, 1, 0)

def Turn(dir):
    if dir == 1:
        Right()
    else:
        Left()

def Stop():
    arlo.stop()
