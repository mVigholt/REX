import cv2
import time as t

def gstreamer_pipeline(capture_width=1024, capture_height=720, framerate=30):
    """Utility function for setting parameters for the gstreamer camera pipeline"""
    return (
        "libcamerasrc ! "
        "videobox autocrop=true ! "
        "video/x-raw, width=(int)%d, height=(int)%d, framerate=(fraction)%d/1 ! "
        "videoconvert ! "
        "appsink"
        % (capture_width, capture_height, framerate)
    )

print("OpenCV version = " + cv2.__version__)

# Open a camera device for capturing
cam = cv2.VideoCapture(gstreamer_pipeline(), apiPreference=cv2.CAP_GSTREAMER)

if not cam.isOpened():  # Error
    print("Could not open camera")
    exit(-1)

# Open a window
WIN_RF = "Example 1"
cv2.namedWindow(WIN_RF)
cv2.moveWindow(WIN_RF, 100, 100)

# Load ArUco dictionary and detector parameters
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters_create()

# Start capturing frames from the camera
while cv2.waitKey(4) == -1:  # Wait for a key pressed event
    retval, frame = cam.read()  # Read frame

    if not retval:  # Error
        print(" < < <  Game over!  > > > ")
        exit(-1)
    
    # Perform ArUco detection on the frame
    corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

    # Print detected marker IDs if found
    if ids is not None:
        print(f"Detected markers: {ids}")

    # Draw markers on the frame if found
    cv2.aruco.drawDetectedMarkers(frame, corners, ids)

    # Show the frame
    cv2.imshow(WIN_RF, frame)

    # You can add a small delay if needed (e.g., to slow down the loop)
    # t.sleep(0.1)

# Release the camera and close the window
cam.release()
cv2.destroyAllWindows()
