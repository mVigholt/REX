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

landmarkRadius = 350
robotRadius = 450/2
robotBuffer = 50


# initialize camera transformation matrix
cam_matrix = np.array([ [f, 0, CH/2],
                        [0, f, CW/2],
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
        def localCoordinates(rvec,v):
            t = math.dist([0,0,0],rvec)
            k = np.array(rvec/t)
            v = np.array(v)
            #vec = v * math.cos(t) + np.cross(k, v) * math.sin(t) + np.matmul(k.T,np.matmul(k,v)) * (1 - math.cos(t))
            vec = v * math.cos(t) + np.cross(k, v) * math.sin(t) + np.dot(k,v) * k * (1 - math.cos(t))
            return vec
        if flat_tvecs is not None:
            for rvec in flat_rvecs: 
                v = localCoordinates(rvec, [145/2, 0, 115])
                print(f"tvecs: {tvecs}")
                print(v)
                flat_tvecs += v
            flat_tvecs = np.delete(np.array(flat_tvecs), 1, 1)
            flat_tvecs[:, 1] = flat_tvecs[:, 1] + robotRadius
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
rotateSpeed = 31
speed = 60
error = 2
safetyStraightDistance = 500
safetySideDistance = 400

class Arlo (object):
    def __init__(self):
        self.arlo = robot.Robot()
        
    def degreeFunction(self, radian):
        return (radian * 180 / math.pi - 0.23)/44.15
    
    def distanceFunction(self, distance):
        return (distance + 1.7)/355.3
    
    def Forward(self, distance): 
        self.arlo.go_diff(speed-error, speed, 1, 1)
        if distance is not None:
            t.sleep(self.distanceFunction(distance))
            self.Stop()
        
  
    def Right(self, degrees = None):
        self.arlo.go_diff(rotateSpeed-1, rotateSpeed, 1, 0)
        if degrees is not None:
            t.sleep(self.degreeFunction(degrees))
            self.Stop()

    def Left(self, degrees = None):
        self.arlo.go_diff(rotateSpeed-1, rotateSpeed, 0, 1)
        if degrees is not None:
            t.sleep(self.degreeFunction(degrees))
            self.Stop()

    def Turn(self, angle):
        if angle < 0: 
            self.Right(abs(angle))
        elif angle > 0:
            self.Left(abs(angle))

    def Stop(self):
        self.arlo.stop()

#----------------------------------------------------------------
def collission(landMarks, dest): # input er en liste af obj objekter
    hasCollided = False
    if landMarks is not None and dest is not None:
        for i in landMarks:
            if math.dist(dest, i) <= robotRadius + robotBuffer + landmarkRadius:
                hasCollided = True
                break
    return hasCollided


# def __collission(dest, landMarks):
#     hasCollided = False
#     for i in landMarks:
#         if math.dist(dest, i) <= robotRadius + robotBuffer + landmarkRadius:
#             hasCollided = True
#             print("Collission detected!!!")
#             break
#     return hasCollided

# def collission(dest, landMarks):
#     hasCollided = False
#     if landMarks is not None:
#         dir = np.linalg.norm(dest) * 100
#         interval = dir.copy()
#         while interval < dest:
#             hasCollided = __collission(interval, landMarks)
#             if hasCollided: break
#             interval += dir
#         hasCollided = __collission(dest, landMarks)
#     return hasCollided