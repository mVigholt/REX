import cv2
import time as t
import robot
import threading

# Try to import picamera2
try:
    import picamera2
    print("Camera.py: Using picamera2 module")
except ImportError:
    print("Camera.py: picamera2 module not available")
    exit(-1)

# Create a robot object and initialize
arlo = robot.Robot()

rotateSpeed = 30
error = 2

def gstreamer_pipeline(capture_width=1024, capture_height=720, framerate=10):
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

if not cam.isOpened(): # Error
    print("Could not open camera")
    exit(-1)

# Global variable to store the frame
frame = None
frame_lock = threading.Lock()
frame_skip = 2
frame_count = 0

# Define a function to capture frames in a separate thread
def capture_frames():
    global frame
    while True:
        retval, temp_frame = cam.read()
        if not retval:
            continue
        with frame_lock:
            frame = temp_frame

# Start the thread to capture frames
capture_thread = threading.Thread(target=capture_frames)
capture_thread.daemon = True  # Daemonize the thread to exit when the main thread exits
capture_thread.start()

# Function to rotate the robot
def Rotate(dir):
    if dir != 0 and dir != 1:
        print("Invalid direction!")
        arlo.stop()
        return
    if dir == 1:
        arlo.go_diff(rotateSpeed, rotateSpeed, 1, 0)
    else:
        arlo.go_diff(rotateSpeed, rotateSpeed, 0, 1)

# Function to process frames (detect markers)
def process_frame(input_frame):
    gray_frame = cv2.cvtColor(input_frame, cv2.COLOR_BGR2GRAY)  # Convert to grayscale for faster processing
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
    corners, ids, _ = cv2.aruco.detectMarkers(gray_frame, aruco_dict)
    gray_frame = cv2.aruco.drawDetectedMarkers(gray_frame, corners, ids)
    
    if len(corners) > 0:
        print("Box detected, stopping.")
        arlo.stop()
        return True
    
    return False

# Initialize rotation and start the main loop
Rotate(1)

while cv2.waitKey(4) == -1:
    frame_count += 1
    if frame_count % frame_skip != 0:
        continue

    with frame_lock:
        if frame is not None:
            start_time = t.time()

            # Process the frame to detect markers
            if process_frame(frame):
                break  # Stop rotating if a box is detected

            print(f"Processing time: {t.time() - start_time} seconds")
    
    Rotate(1)
    t.sleep(0.2)  # Small delay to avoid overwhelming the processor

cv2.imwrite("OttosView.png", frame)
cv2.destroyAllWindows()
