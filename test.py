import help
import math
import time as t
import cv2

otto = help.Arlo()
cam = help.Cam()
lap = help.Timed_lap()

i = 0
landMarks = [3]
searching = True
dir = 1
print(f"looking for {landMarks[i]} with index {i}")
while cv2.waitKey(4) == -1: # Wait for a key pressed event
    # print(f"lapTime() = {lap.time()}\n")
    # if searching:
    #     help.Stop()
    #     t.sleep(0.2)
    
    #_, corners, ids = cam.next_frame_with_detection(ret_corner=True, ret_id=True)
    ids, tvecs = cam.next_map()
    print(tvecs)
    t.sleep(1)

    # if ids is not None:
    #     markFound = landMarks[i] in ids
    # else:
    #     markFound = False
        
    # if markFound:
    #     id_index = list(ids).index(landMarks[i])
    #     Z = math.dist([tvecs[id_index][0]], [tvecs[id_index][2]]) 
    #     dir = 1 if tvecs[id_index][0] > 0 else 0
    #     print(Z)
    #     if Z < 750:
    #         help.Stop()
    #         i += 1
    #         searching = True
    #         print(f"looking for {landMarks[i]} with index {i}")
    #         if i >= len(landMarks):
    #             exit(-1)
    #     elif searching:
    #         searching = False
    # else:
    #     searching = True
    
    # if searching:
    #     help.Turn(dir)
    #     t.sleep(0.2)
    # else:
    #     help.Forward()
    # cam.stream()
    
#cv2.imwrite("OttosView.png", frameReference)
# Finished successfully

# path = []
# theta = 0
# for i in range(len(path),0,-1):
#     dxy = path[i-1] - path[i]
#     theta += (math.pi/2) - math.atan2(dxy[0],dxy[1]) + theta
#     dist = math.dist([0,0],dxy)
#     otto.Turn(theta)
#     otto.Forward(dist)
