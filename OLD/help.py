import itertools
import cv2 # Import the OpenCV library
import numpy as np
import time as t
import robot
import math
import sys
import os

# Define the path to the directory where the desired module is located
directory_path = os.path.abspath(os.path.join(os.path.dirname(__file__), 'ex5'))

sys.path.insert(0, directory_path)

import camera

CW = 1640
CH = 1232
# old_CW = 1024
# old_CH = 720

X = 145
# old_f = 1138
# their original f = 1687
f = 1318

landmarkRadius = 200
robotRadius = 450/2
robotBuffer = 50

def RotationMatrix(angle): 
    return np.array([[np.cos(angle), -np.sin(angle)],
                     [np.sin(angle), np.cos(angle)]])
def ToGlobal(point,angle,localPoint):
    return np.dot(RotationMatrix(angle), localPoint) + point
def ToLocal(point,angle,globalPoint):
    return np.dot(RotationMatrix(-angle), (globalPoint - point))

def rotation_matrix_to_euler_angles(R):
    sy = np.sqrt(R[0, 0] * R[0, 0] +  R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = np.arctan2(R[2, 1], R[2, 2])
        y = np.arctan2(-R[2, 0], sy)
        z = np.arctan2(R[1, 0], R[0, 0])
    else:
        x = np.arctan2(-R[1, 2], R[1, 1])
        y = np.arctan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])

# initialize camera transformation matrix
cam_matrix = np.array([ [f, 0, CW/2],
                        [0, f, CH/2],
                        [0, 0, 1]])
distCoeffs = np.zeros((5, 1))

class Cam (camera.Camera):
    def __init__(self, robottype='arlo', useCaptureThread=False):
        super().__init__(0, robottype=robottype, useCaptureThread=useCaptureThread)
        self.intrinsic_matrix = cam_matrix
        # def gstreamer_pipeline(capture_width=CW, capture_height=CH, framerate=30):
        #     """Utility function for setting parameters for the gstreamer camera pipeline"""
        #     return (
        #         "libcamerasrc !"
        #         "videobox autocrop=true !"
        #         "video/x-raw, width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        #         "videoconvert ! "
        #         "appsink drop=true sync=false" #Fjerner måske buffer
        #         % (
        #             capture_width,
        #             capture_height,
        #             framerate,
        #         )
        #     )
        # # Open a camera device for capturing
        # self.cam = cv2.VideoCapture(gstreamer_pipeline(), apiPreference=cv2.CAP_GSTREAMER)
        # if not self.cam.isOpened(): # Error
        #     print("Could not open camera")
        #     exit(-1)
        # print("OpenCV version = " + cv2.__version__)
        # self.cam.set(cv2.CAP_PROP_BUFFERSIZE, 1) #Fjerner måske buffer
        
        # #--------------------------------------------------------------------------------
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.WIN_RF = None
        self.frameReference = None
        self.landmarkCorners = None 
        self.landmarkIds = None
        self.next_frame_with_detection(True)
        #--------------------------------------------------------------------------------
    
    def flatten(self, matLike):
        return None if matLike is None else list(itertools.chain(*matLike))     
    
    def next_frame(self):
        self.frameReference = self.get_next_frame()
    
    def next_frame_with_detection(self, new_frame = False):
        if new_frame: self.next_frame()
        self.landmarkCorners, self.landmarkIds, _ = cv2.aruco.detectMarkers(self.frameReference, self.aruco_dict)
    
    def next_map(self, new_frame = False):
        self.next_frame_with_detection(new_frame)
        rvecs, tvecs, _  = cv2.aruco.estimatePoseSingleMarkers(self.landmarkCorners, X, cam_matrix, distCoeffs)
        print("natty: ", tvecs)
        #tvec = [with, height, debth] ???
        flat_tvecs = self.flatten(tvecs)
        flat_rvecs = self.flatten(rvecs)
        if flat_tvecs and flat_rvecs is not None:
            flat_tvecs = np.delete(np.array(flat_tvecs), 1, 1)
            for rvec, tvec in zip(flat_rvecs, flat_tvecs): 
                print("local tvec:", tvec)
                print("local distance: ", np.linalg.norm(tvec))
                rotation_matrix, _ = cv2.Rodrigues(rvec)
                euler_angles = rotation_matrix_to_euler_angles(rotation_matrix)
                tvec = ToGlobal(tvec, euler_angles[1], np.array([0, 115]))
                print("angle: ", euler_angles)
                print("global tvec: ", tvec)
                print("global distance: ", np.linalg.norm(tvec))
            flat_tvecs[:, 1] = flat_tvecs[:, 1] + robotRadius
        else:
            print("flat_tvecs or flat_rvecs was None")
        return self.flatten(self.landmarkIds), flat_tvecs
            
    def __setup_stream(self):
        # Open a window
        self.WIN_RF = "Example 1"
        cv2.namedWindow(self.WIN_RF)
        cv2.moveWindow(self.WIN_RF, 100, 100)
        
    def stream(self):
        if self.WIN_RF is None:
            self.__setup_stream()
        # Draw markers on the frame if found
        hej = cv2.aruco.drawDetectedMarkers(self.frameReference, self.landmarkCorners, self.landmarkIds)
        # cv2.aruco.drawDetectedMarkers(self.frameReference, self.landmarkCorners, self.landmarkIds)
        # rvecs, tvecs, _  = cv2.aruco.estimatePoseSingleMarkers(self.landmarkCorners, X, cam_matrix, distCoeffs)
        # for i in range(len(self.landmarkIds)):
        #     cv2.drawFrameAxes(self.frameReference, cam_matrix, distCoeffs, rvecs[i], tvecs[i], 100, 2)
        # # Stream frames
        
        cv2.imshow(self.WIN_RF, hej)
        
  
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