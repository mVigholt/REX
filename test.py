import help as otto
import cv2
import time as t

i = 0
landMarks = [2,3,4]
cam = otto.openCam()
# load dictionary and parameters
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

searching = True
dir = 1

time = t.time()
def lapTime():
    global time
    last = time
    time = t.time()
    return time - last

while cv2.waitKey(4) == -1: # Wait for a key pressed event
    print(f"lapTime() = {lapTime()}\n")
    if searching:
        otto.Stop()
        t.sleep(0.1)
    
    cam.read()  
    retval, frameReference = cam.read() # Read frame
    if not retval: # Error
        print(" < < <  Game over!  > > > ")
        exit(-1)
    
    corners, ids, _ = cv2.aruco.detectMarkers(frameReference, aruco_dict)
    otto.streamCam(frameReference, corners, ids)
    
    #TODO Check for specific marker
    markFound = landMarks[i] in ids
    j = 0
    print(ids)
    if markFound:
        j = ids.index(landMarks[i])
        Z, dir = otto.distAndDir(corners[0][0])
        print(f"dist = {Z}")
        if Z < 500:
            otto.Stop()
            i += 1
            searching = True
            if i >= len(landMarks):
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

