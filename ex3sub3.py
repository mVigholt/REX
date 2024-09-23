import cv2  # Import the OpenCV library
import time as t
import robot
import numpy as np
import itertools
import matplotlib.pyplot as plt  # Import Matplotlib

capture_width = 1024
capture_height = 720

def gstreamer_pipeline(capture_width=capture_width, capture_height=capture_height, framerate=2):
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

if not cam.isOpened():  # Error
    print("Could not open camera")
    exit(-1)
    
X = 145
f = 1138

print("OpenCV version = " + cv2.__version__)

# Open a window
WIN_RF = "Example 1"
cv2.namedWindow(WIN_RF)
cv2.moveWindow(WIN_RF, 100, 100)

# Load dictionary and parameters
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)

# Initialize camera transformation matrix
camera_matrix = np.array([[f, 0, capture_width / 2],
                          [0, f, capture_height / 2],
                          [0, 0, 1]])

# Assuming no lens distortion for now
distCoeffs = np.zeros((5, 1))

coordinates = []

# Set up Matplotlib figure and axis
plt.ion()  # Turn on interactive mode for Matplotlib
fig, ax = plt.subplots()
sc = ax.scatter([], [])  # Initialize an empty scatter plot
ax.set_xlim(-500, 500)  # Adjust as needed for your specific data range
ax.set_ylim(-500, 500)
ax.set_title("Marker Coordinates")
ax.set_xlabel("X-axis")
ax.set_ylabel("Y-axis")

annotations = []  # To store the text annotations

while cv2.waitKey(4) == -1:  # Wait for a key press
    retval, frameReference = cam.read()  # Read frame
    
    if not retval:  # Error
        print(" < < <  Game over!  > > > ")
        exit(-1)
    
    # Detect ArUco markers
    corners, ids, _ = cv2.aruco.detectMarkers(frameReference, aruco_dict)
    rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, X, camera_matrix, distCoeffs)
    
    if ids is not None:
        ids_flattened = list(itertools.chain(*ids))
        tvecs_flattened = list(itertools.chain(*tvecs))
        
        # Clear previous coordinates and annotations
        coordinates.clear()
        for ann in annotations:
            ann.remove()  # Remove old annotations from the plot
        annotations.clear()
        
        # Append the coordinates along with their respective IDs
        for i in range(len(tvecs_flattened)):
            coordinates.append((tvecs_flattened[i][0], tvecs_flattened[i][2]))  # Assuming 2D XZ only
            
            # Add annotations (ID tags)
            ann = ax.annotate(f"ID: {ids_flattened[i]}", (tvecs_flattened[i][0], tvecs_flattened[i][2]), color='red')
            annotations.append(ann)
        
        # Update the scatter plot data
        x_coords, y_coords = zip(*coordinates)
        sc.set_offsets(np.c_[x_coords, y_coords])
        
        # Redraw the plot with updated data
        plt.draw()
        plt.pause(0.001)  # Pause to update the plot
    
    # Show the frame in OpenCV window
    cv2.imshow(WIN_RF, frameReference)

# Cleanup
cam.release()
cv2.destroyAllWindows()
