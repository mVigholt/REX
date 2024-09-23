import help as otto
import cv2
import time as t

landMarks = [2,3,4]
cam = otto.openCam()
# load dictionary and parameters
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

searching = True
dir = 1

time = t.time()
def lapTime():
    last = time
    time = t.time()
    return time - last

while cv2.waitKey(4) == -1: # Wait for a key pressed event
    print(lapTime(), "\n")
    if searching:
        otto.Stop()
    
    cam.read()  
    retval, frameReference = cam.read() # Read frame
    if not retval: # Error
        print(" < < <  Game over!  > > > ")
        exit(-1)
    
    corners, ids, _ = cv2.aruco.detectMarkers(frameReference, aruco_dict)
    otto.streamCam(frameReference, corners, ids)
    
    #TODO Check for specific marker
    markFound = (len(ids) > 0)
    if markFound:
        Z, dir = otto.distAndDir(corners[0])
        if Z < 1000:
            otto.Stop()
            exit(-1)
        elif searching:
            searching = False

    else:
        searching = True
    
    if searching:
        otto.Turn(dir)
        t.sleep(0.2)
    else:
        otto.Go()
        t.sleep(0.1)
    
#cv2.imwrite("OttosView.png", frameReference)
# Finished successfully

