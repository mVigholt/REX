import itertools
import cv2 # Import the OpenCV library
import numpy as np
import time as t
import robot
import math

CW = 1024
CH = 720

X = 145
f = 1138

landmarkRadius = 330
box_x = 145/2
box_z = 115
box_c = math.dist([0,0],[box_x,box_z])
box_v = math.acos(box_x/box_c)
robotRadius = 450/2
buffer = 100

# initialize camera transformation matrix
cam_matrix = np.array([[f, 0, CW/2],
                         [0, f, CH/2],
                         [0, 0, 1]])
distCoeffs = np.zeros((5, 1))

class Cam (object):
    def __init__(self):
        def gstreamer_pipeline(capture_width=CW, capture_height=CH, framerate=30):
            """Utility function for setting parameters for the gstreamer camera pipeline"""
            return (
                "libcamerasrc !"
                "videobox autocrop=true !"
                "video/x-raw, width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
                "videoconvert ! "
                "appsink drop=true sync=false" #Fjerner måske buffer
                % (
                    capture_width,
                    capture_height,
                    framerate,
                )
            )
        # Open a camera device for capturing
        self.cam = cv2.VideoCapture(gstreamer_pipeline(), apiPreference=cv2.CAP_GSTREAMER)
        if not self.cam.isOpened(): # Error
            print("Could not open camera")
            exit(-1)
        print("OpenCV version = " + cv2.__version__)
        self.cam.set(cv2.CAP_PROP_BUFFERSIZE, 1) #Fjerner måske buffer
        
        #--------------------------------------------------------------------------------
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.WIN_RF = None
        self.frameReference = None
        self.corners = None 
        self.ids = None
        self.next_frame_with_detection()
        #--------------------------------------------------------------------------------
    
    def flatten(self, matLike):
        return None if matLike is None else list(itertools.chain(*matLike))     
    
    def next_frame(self, ret_frame = False):
        retval, self.frameReference = self.cam.read()
        if not retval: # Error
            print(" < < <  Game over!  > > > ")
            exit(-1)
        if ret_frame:
            return self.frameReference
    
    def next_frame_with_detection(self, ret_frame = False, ret_corner = False, ret_id = False):
        self.next_frame()
        self.corners, self.ids, _ = cv2.aruco.detectMarkers(self.frameReference, self.aruco_dict)
        if ret_frame or ret_corner or ret_id:
            return (self.frameReference if ret_frame else None, 
                    self.flatten(self.corners) if ret_corner else None, 
                    self.flatten(self.ids) if ret_id else None)
    
    def next_map(self):
        self.next_frame_with_detection()
        rvecs, tvecs, _  = cv2.aruco.estimatePoseSingleMarkers(self.corners, X, cam_matrix, distCoeffs)
        #tvec = [with, height, debth] ???
        flat_tvecs = self.flatten(tvecs)
        flat_rvecs = self.flatten(rvecs)
        if flat_tvecs is not None:
            flat_tvecs = np.delete(np.array(flat_tvecs), 1, 1)
            flat_tvecs[:, 1] = flat_tvecs[:, 1] + robotRadius
            test = []
            print(flat_rvecs)
            for rvec in flat_rvecs: 
                test.append([math.cos(box_v-rvec[1])*box_c, math.sin(box_v-rvec[1])*box_c])
            print(test)
        return self.flatten(self.ids), flat_tvecs
            
    def __setup_stream(self):
        # Open a window
        self.WIN_RF = "Example 1"
        cv2.namedWindow(self.WIN_RF)
        cv2.moveWindow(self.WIN_RF, 100, 100)
        
    def stream(self):
        if self.WIN_RF is None:
            self.__setup_stream()
        # Draw markers on the frame if found
        frameDrawing = cv2.aruco.drawDetectedMarkers(self.frameReference, self.corners, self.ids)
        # Stream frames
        cv2.imshow(self.WIN_RF, frameDrawing)
        
  
class Timed_lap (object):
    def __init__(self):   
        self.before = None
        self.now = t.time()
        
    def time(self):
        self.before = self.now
        self.now = t.time()
        return self.now - self.before               
    

def distAndDir(corners):
    ul = corners[0] 
    ur = corners[1] 
    ll = corners[3] 
    lr = corners[2] 
    x =  (np.linalg.norm(ul - ll) + np.linalg.norm(ur - lr)) / 2
    Z = (f/x) * X
    xPos = (np.linalg.norm(ul - ur) + np.linalg.norm(ll - lr)) / 2
    dir = 1 if xPos > CW/2 else 0
    return Z, dir

##------------------------------------------------------------------------------------------
# Create a robot object and initialize
arlo = robot.Robot()
rotateSpeed = 31
speed = 60
error = 2
safetyStraightDistance = 500
safetySideDistance = 400

def Forward():
  arlo.go_diff(speed-error, speed, 1, 1)
  
def Right():
    arlo.go_diff(rotateSpeed-1, rotateSpeed, 1, 0)

def Left():
    arlo.go_diff(rotateSpeed-1, rotateSpeed, 0, 1)

def Turn(dir):
    if dir == 1:
        Right()
    else:
        Left()

def Stop():
    arlo.stop()

#----------------------------------------------------------------
class robot:
  def __init__(self, x = 0, y = 0, theta = 0):
    self.x = x
    self.y = y
    self.theta = theta

#----------------------------------------------------------------
def collission(landMarks, node): # input er en liste af obj objekter
    hasCollided = False
    if landMarks is not None and node is not None:
        for i in landMarks:
            if euclidean([node.pos[0], node.pos[1]], i) <= robotRadius + landmarkRadius:
                hasCollided = True
                break
    return hasCollided

def euclidean(a, b):
    return math.dist(a,b) #sqrt((x_1 - x_2)**2 + (x_2 - y_2)**2)

def normalize(vector):
    magnitude = math.sqrt(sum(x**2 for x in vector))
    if magnitude == 0:
        return vector  # or handle zero magnitude case as needed
    return [x / magnitude for x in vector]

# def __collission(dest, landMarks):
#     hasCollided = False
#     for i in landMarks:
#         if math.dist(dest, i) <= robotRadius + buffer + landmarkRadius:
#             hasCollided = True
#             print("Collission detected!!!")
#             break
#     return hasCollided

# def collission(dest, landMarks):
#     hasCollided = False
#     if landMarks is not None:
#         dir = normalize(dest) * 100
#         interval = dir.copy()
#         while interval < dest:
#             hasCollided = __collission(interval, landMarks)
#             if hasCollided: break
#             interval += dir
#         hasCollided = __collission(dest, landMarks)
#     return hasCollided