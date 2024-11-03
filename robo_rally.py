import copy
import math
import random
import cv2
import particle
import camera
import numpy as np
import time
from timeit import default_timer as timer
import sys
import help as h
import rrt_mod as rt
import map as m
import robot_models

# Flags
showGUI = True  # Whether or not to open GUI windows
onRobot = True  # Whether or not we are running on the Arlo robot

def isRunningOnArlo():
    """Return True if we are running on Arlo, otherwise False.
      You can use this flag to switch the code from running on you laptop to Arlo - you need to do the programming here!
    """
    return onRobot


if isRunningOnArlo():
    # XXX: You need to change this path to point to where your robot.py file is located
    sys.path.append("../../../../Arlo/python")


try:
    import robot
    onRobot = True
except ImportError:
    print("selflocalize.py: robot module not present - forcing not running on Arlo!")
    onRobot = False


# Some color constants in BGR format
CRED = (0, 0, 255)
CGREEN = (0, 255, 0)
CBLUE = (255, 0, 0)
CCYAN = (255, 255, 0)
CYELLOW = (0, 255, 255)
CMAGENTA = (255, 0, 255)
CWHITE = (255, 255, 255)
CBLACK = (0, 0, 0)

# Landmarks.
# The robot knows the position of 2 landmarks. Their coordinates are in the unit centimeters [cm].
landmarkIDs = [6, 2, 3, 4]
landmarks = {
    6: [300, 0.0],  # Coordinates for landmark 1
    2: [0.0, 0.0],  # Coordinates for landmark 2
    3: [300, 400.0],  # Coordinates for landmark 3
    4: [0, 400.0]  # Coordinates for landmark 4
}
landmark_colors = [CRED, CGREEN, CBLUE, CYELLOW] # Colors used when drawing the landmarks

#============================================================================
# Allocate space for world map
world = np.zeros((1000,1000,3), dtype=np.uint8)

sequence = [6,2,3,4,6]
si = 0
# Initialize particles
num_particles = 1000
pc = [0,0] #landmarks[sequence[si]]
pr = 400

lap = h.Timed_lap()
measurements = dict()
rotation_so_far = 0
#============================================================================

if onRobot:
    otto = h.Arlo()

def jet(x):
    """Colour map for drawing particles. This function determines the colour of 
    a particle from its weight."""
    r = (x >= 3.0/8.0 and x < 5.0/8.0) * (4.0 * x - 3.0/2.0) + (x >= 5.0/8.0 and x < 7.0/8.0) + (x >= 7.0/8.0) * (-4.0 * x + 9.0/2.0)
    g = (x >= 1.0/8.0 and x < 3.0/8.0) * (4.0 * x - 1.0/2.0) + (x >= 3.0/8.0 and x < 5.0/8.0) + (x >= 5.0/8.0 and x < 7.0/8.0) * (-4.0 * x + 7.0/2.0)
    b = (x < 1.0/8.0) * (4.0 * x + 1.0/2.0) + (x >= 1.0/8.0 and x < 3.0/8.0) + (x >= 3.0/8.0 and x < 5.0/8.0) * (-4.0 * x + 5.0/2.0)

    return (255.0*r, 255.0*g, 255.0*b)

def draw_world(est_pose, particles, world, path=None):
    """Visualization.
    This functions draws robots position in the world coordinate system."""

    # Fix the origin of the coordinate system
    offsetX = 500
    offsetY = 500

    # Constant needed for transforming from world coordinates to screen coordinates (flip the y-axis)
    ymax = world.shape[0]

    world[:] = CWHITE # Clear background to white

    # Find largest weight
    max_weight = 0
    for particle in particles:
        max_weight = max(max_weight, particle.getWeight())

    # Draw particles
    for particle in particles:
        x = int(particle.getX() + offsetX)
        y = ymax - (int(particle.getY() + offsetY))
        colour = jet(particle.getWeight() / max_weight)
        cv2.circle(world, (x,y), 2, colour, 2)
        b = (int(particle.getX() + 15.0*np.cos(particle.getTheta()))+offsetX, 
                                     ymax - (int(particle.getY() + 15.0*np.sin(particle.getTheta()))+offsetY))
        cv2.line(world, (x,y), b, colour, 2)

    # Draw landmarks
    for i in range(len(landmarkIDs)):
        ID = landmarkIDs[i]
        lm = (int(landmarks[ID][0] + offsetX), int(ymax - (landmarks[ID][1] + offsetY)))
        cv2.circle(world, lm, 5, landmark_colors[i], 2)

    # Draw estimated robot pose
    a = (int(est_pose.getX())+offsetX, ymax-(int(est_pose.getY())+offsetY))
    b = (int(est_pose.getX() + 15.0*np.cos(est_pose.getTheta()))+offsetX, 
                                 ymax-(int(est_pose.getY() + 15.0*np.sin(est_pose.getTheta()))+offsetY))
    
    if path is not None:
        for i in range(len(path)):
            path_point = (int(path[i][0] + offsetX), int(path[i][1] + offsetY))
            cv2.circle(world, path_point, 2, CBLACK, 2)
        for i in range(len(landmarks)):
            id = landmarkIDs[i]
            lmm = (int(landmarks[id][0] + offsetX), int(landmarks[id][1] + offsetY))
            cv2.circle(world, lmm, 15, CRED, 2)
    
    cv2.circle(world, a, 5, CMAGENTA, 2)
    cv2.line(world, a, b, CMAGENTA, 2)
    

def initialize_particles(num_particles, c=[150,200], r=600):
    particles = []
    for i in range(num_particles):
        # Random starting points. 
        p = particle.Particle(600.0*np.random.ranf() - 100.0, 600.0*np.random.ranf() - 250.0, np.mod(2.0*np.pi*np.random.ranf(), 2.0*np.pi), 1.0/num_particles)
        # p = particle.Particle(np.random.uniform(c[0]-r, c[0]+r), np.random.uniform(c[1]-r, c[1]+r), np.mod(2.0*np.pi*np.random.ranf(), 2.0*np.pi), 1.0/num_particles)    
        particles.append(p)

    return particles

# Main program #
try:
    if showGUI:
        # Open windows
        # WIN_RF1 = "Robot view"
        # cv2.namedWindow(WIN_RF1)
        # cv2.moveWindow(WIN_RF1, 50, 50)

        WIN_World = "World view"
        cv2.namedWindow(WIN_World)
        cv2.moveWindow(WIN_World, 500, 50)
        
#=============================================================================================================
    particles = initialize_particles(num_particles, c=pc, r=pr)
#=============================================================================================================

    # Driving parameters
    velocity = 0.0 # cm/sec
    angular_velocity = 0.0 # radians/sec

    # Initialize the robot (XXX: You do this)
    print("Opening and initializing camera")
    if isRunningOnArlo():
        #cam = camera.Camera(0, robottype='arlo', useCaptureThread=True)
        cam = h.Cam()
    else:
        #cam = camera.Camera(0, robottype='macbookpro', useCaptureThread=True)
        cam = h.Cam(onRobot)
    

    while True:
        # Move the robot according to user input (only for testing)
        action = cv2.waitKey(10)
        if action == ord('q'): # Quit
            break
    
        # if not isRunningOnArlo():
        #     if action == ord('w'): # Forward
        #         velocity += 4.0
        #     elif action == ord('x'): # Backwards
        #         velocity -= 4.0
        #     elif action == ord('s'): # Stop
        #         velocity = 0.0
        #         angular_velocity = 0.0
        #     elif action == ord('a'): # Left
        #         angular_velocity += 0.2
        #     elif action == ord('d'): # Right
        #         angular_velocity -= 0.2

#=============================================================================================================        
        #particles = initialize_particles(num_particles, c=pc, r=pr)
        # add some noise??
        particle.noise(particles, pd_noise=[5, 0.1])
#=============================================================================================================
        
        # Use motor controls to update particles
        # XXX: Make the robot drive     
        # XXX: You do this
        # dt = lap.time()
        # for p in particles:
        #     theta = p.getTheta()
        #     deltaX = math.cos(theta) * velocity * dt
        #     deltaY = math.sin(theta) * velocity * dt
        #     deltaTheta = angular_velocity * dt
        #     particle.move_particle(p, deltaX, deltaY, deltaTheta) 

        # Fetch next frame
        colour = cam.get_next_frame()
        
        # Detect objects
        objectIDs, dists, angles = cam.detect_aruco_objects(colour)
        if not isinstance(objectIDs, type(None)):
            # List detected objects
            for i in range(len(objectIDs)):
                # print("Object ID = ", objectIDs[i], ", Distance = ", dists[i], ", angle = ", angles[i])
                # XXX: Do something for each detected object - remember, the same ID may appear several times
                if (objectIDs[i] in landmarks and 
                    (objectIDs[i] not in measurements or 
                    measurements[objectIDs[i]][1] > dists[i])):
                    measurements[objectIDs[i]] = [objectIDs[i], dists[i], angles[i]] 
        
        # print landmark positions
        for me in measurements:
          print(f"landMark {measurements[me][0]}: {measurements[me][1]}")
        
        # If more than 1 object, converge
        if len(measurements) == 2:
            def angle_propability(particle: particle.Particle, measurement):
                sigma = 0.05 #rad
                di = math.sqrt(((landmarks[measurement[0]][0] - particle.getX())**2) + 
                               ((landmarks[measurement[0]][1] - particle.getY())**2))
                uov = np.array([math.cos(particle.getTheta()), math.sin(particle.getTheta())])
                ulv = np.array([landmarks[measurement[0]][0] - particle.getX(), 
                                landmarks[measurement[0]][1] - particle.getY()]) / di
                ouov = np.array([- math.sin(particle.getTheta()), math.cos(particle.getTheta())])
                uv = np.sign(np.dot(ulv, ouov)) * math.acos(np.dot(ulv, uov))
                return  ((1 / (2*math.pi * (sigma**2))) * 
                            math.exp(
                                -   ((measurement[2]- uv)**2) /
                                    (2 * math.pi * (sigma**2))
                            )
                        )
                
            def dist_propability(particle: particle.Particle, measurement):
                sigma = 3 #cm
                di = math.sqrt(((landmarks[measurement[0]][0] - particle.getX())**2) + 
                               ((landmarks[measurement[0]][1] - particle.getY())**2))
                
                return  ((1 / (2*math.pi * (sigma**2))) * 
                            math.exp(
                                -   ((measurement[1]- di)**2) /
                                    (2 * math.pi * (sigma**2))
                            )
                        )

            # Compute particle weights
            # XXX: You do this
            weights = []
            
            for p in particles:
                p: particle.Particle
                w = 1
                for key in measurements:
                    w *= angle_propability(p,measurements[key]) * dist_propability(p,measurements[key])
                p.setWeight(w)#*p.getWeight())
                weights.append(w)

            
            weight_sum = sum(weights)
            if weight_sum > 0:
                for p in particles:
                    p.setWeight(p.getWeight() / weight_sum)
            print(f"len(weights) = {len(weights)}")
            print(f"weight_sum = {weight_sum}")        
            # Resampling
            # XXX: You do this
            weighted_choice = random.choices(particles, weights, k = num_particles)
            particles = [copy.deepcopy(p) for p in weighted_choice]

            # Draw detected objects
            cam.draw_aruco_objects(colour)
            
        else:
            # No observation - reset weights to uniform distribution
            for p in particles:
                p.setWeight(1.0/num_particles)

        if len(measurements) < 2 and rotation_so_far != 2*3.14:
            # rotate
            otto.Turn(math.pi/8)
            for p in particles:
              particle.turn(p, math.pi/8)
            rotation_so_far += math.pi/8
        
        est_pose = particle.estimate_pose(particles) # The estimate of the robots current pose    
        print(f"est_pose: \n{[est_pose.getX()*10, est_pose.getY()*10]}")    
        if showGUI:
            # Draw map
            draw_world(est_pose, particles, world)
    
            # # Show frame
            # cv2.imshow(WIN_RF1, colour)

            # Show world
            cv2.imshow(WIN_World, world)
        
        #=======================================================================
        # TRY TO DRIVE
        #=======================================================================
        accepltable, pos_var = particle.accepltable_robot_pos_estimate(particles)
        if accepltable:
            print("Starting path planning")
            path_res = 15
            expand_dis = 1000
            rob = robot_models.PointMassModel(ctrl_range=[-path_res, path_res])
            
            _, local_coords = cam.next_map(True) 
            
            # her indsætter vi det globale koordinat system konverteret til lokalt
            local_goal = h.ToLocal(np.array([est_pose.getX()*10, est_pose.getY()*10]), est_pose.getTheta()-(math.pi/2), np.array(landmarks[sequence[si]])*10) # her konverterer vi (75, 0) til et eller andet lokalt koordinat
            local_goal = local_goal - (local_goal * 40 / np.linalg.norm(local_goal))
            print(f"local_goal: {local_goal}")
            
            print(local_coords)
            
            local_low= (0 - 0*1000 - est_pose.getX() , 0 - 0*1000 - est_pose.getY())
            local_high= (3000 + 0*1000 - est_pose.getX(), 4000 + 0*1000 - est_pose.getY())
            # map = m.landmark_map(low=(-4000, 0), high=(4000, 4000), landMarks=local_coords)
            map = m.landmark_map(low=local_low, high=local_high, landMarks=[])
            rrt = rt.RRT(start=[0, 0],
                        goal=local_goal,
                        robot_model=rob,
                        map=map,
                        expand_dis=expand_dis,
                        path_resolution=path_res,
                        )
            path = rrt.planning(animation=False)
            if path is not None:
                print("Beginning drive sequence.")
                print("global robot pos: ", [est_pose.getX(), est_pose.getY(), est_pose.getTheta()])
                print(f"local goal: {local_goal}")
                print(f"path = {path}")
                cur = np.array([0,1])
                for i in range(len(path)-1,0,-1):
                    next = path[i-1] - path[i]
                    theta = math.acos(np.dot(cur,next)/(math.dist([0,0],cur)* math.dist([0,0],next)))
                    theta = theta * np.sign(np.cross(cur,next))
                    dist = math.dist([0,0],next)
                    print(f"turn: {theta}")
                    print(f"move: {dist}")
                    otto.Turn(theta)
                    otto.Forward(dist)
                    cur = next
                
                si += 1
                if si >= len(sequence): break
            
        # Clear seen objects
        measurements.clear()
        pc = (est_pose.getX(), est_pose.getY())#, est_pose.getTheta())
        pr = pos_var + 10

finally: 
    # Make sure to clean up even if an exception occurred
    
    # Close all windows
    cv2.destroyAllWindows()

    # Clean-up capture thread
    cam.terminateCaptureThread()