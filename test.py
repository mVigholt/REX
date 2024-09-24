import help
import numpy as np
import time as t
import cv2

cam = help.Cam
lap = help.Timed_lap

i = 0
landMarks = [2,3,4]
searching = True
dir = 1

while cv2.waitKey(4) == -1: # Wait for a key pressed event
    print(f"lapTime() = {lap.time()}\n")
    if searching:
        help.Stop()
        t.sleep(0.1)
    
    #_, corners, ids = cam.next_frame_with_detection(ret_corner=True, ret_id=True)
    ids, tvecs = cam.next_map()
    cam.stream()
    print(f"looking for {landMarks[i]} with index {i}")

    if ids is not None:
        markFound = landMarks[i] in ids
    else:
        markFound = False
        
    if markFound:
        id_index = list(ids).index(landMarks[i])
        Z = tvecs[id_index][2]
        dir = 1 if tvecs[id_index][0] > 0 else 0
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

