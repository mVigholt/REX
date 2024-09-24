import help
import numpy as np
import time as t
import cv2

i = 0
landMarks = [2,3,4]
CAM = help.Cam
LAP = help.Timed_lap

searching = True
dir = 1

while cv2.waitKey(4) == -1: # Wait for a key pressed event
    print(f"lapTime() = {LAP.time()}\n")
    if searching:
        help.Stop()
        t.sleep(0.1)
    
    _, corners, ids = CAM.next_frame_with_detection(ret_corner=True, ret_id=True)
    #help.streamCam(frameReference, corners, ids)
    print(f"looking for {landMarks[i]} with index {i}")

    markFound = False
    if ids is not None:
        markFound = [landMarks[i]] in ids
        
    j = 0
    if markFound:
        j = list(ids).index([landMarks[i]])
        _, tvecs, _  = cv2.aruco.estimatePoseSingleMarkers(corners, help.X, help.cam_matrix, np.zeros((5, 1)))
        Z, dir = help.distAndDir(corners[j][0])
        if Z < 500:
            help.Stop()
            print("in pos")
            i += 1
            searching = True
            if i >= len(landMarks):
                exit(-1)
        elif searching:
            searching = False

    else:
        searching = True
    
    if searching:
        help.Turn(dir)
        t.sleep(0.2)
    else:
        help.Go()
    
#cv2.imwrite("OttosView.png", frameReference)
# Finished successfully

