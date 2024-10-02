import cv2 # Import the OpenCV library
import time as t

#from matplotlib import pyplot as plt
import numpy as np
import help
import rrt_mod as rt
import robot_models
import map as m
import math

otto = help.Arlo()
cam = help.Cam()

# def DrawPath(path):

#   # Extract x and y coordinates
#   x_coords = [node[0] for node in path]
#   y_coords = [node[1] for node in path]

#   # Define the center and radius of the circle
#   circle_center = [0, 1000]
#   circle_radius = 525

#   # Plot the path and connect them in order
#   plt.figure(figsize=(10, 8))
#   plt.plot(x_coords, y_coords, '-o', color='b', label='Path')

#   # Mark start and end points for clarity
#   plt.scatter(path[-1][0], path[-1][1], color='r', label='Start', s=100)
#   plt.scatter(path[0][0], path[0][1], color='g', label='End', s=100)

#   # Draw the circle representing an obstacle
#   circle = plt.Circle(circle_center, circle_radius, color='orange', fill=False, linestyle='--', linewidth=2, label='Obstacle')

#   # Add the circle to the plot
#   plt.gca().add_patch(circle)

#   # Set labels and legend
#   plt.xlabel('X Coordinate')
#   plt.ylabel('Y Coordinate')
#   plt.title('Nodes Path with Obstacle')
#   plt.grid(True)
#   plt.legend()
#   plt.axis('equal')

#   # Show plot
#   plt.show()

# while cv2.waitKey(4) == -1: # Wait for a key pressed event
  # ids, tvecs = cam.next_map()

path_res = 200
expand_dis = 2000

#tvecs = [[0, 1000]]
ids, tvecs = cam.next_map()

map = m.landmark_map(low=(-1000, 0), high=(1000, 2000), landMarks=tvecs)

robot = robot_models.PointMassModel(ctrl_range=[-path_res, path_res])

rrt = rt.RRT(
  start=[0, 0],
  goal=[0, 2000],
  robot_model=robot,
  map=map,
  expand_dis=expand_dis,
  path_resolution=path_res,
  )

path = rrt.planning(animation=False)

if path is None:
    print("Cannot find path")
else:
    print("found path!!")

print(path)
#DrawPath(path=path)

if path is not None:
  path.append(np.array([0,-1]))
  theta = 0
  for i in range(len(path)-1,0,-1):
      cur = path[i-1] - path[i]
      next = path[i-2] - path[i-1]
      theta = math.acos(np.dot(cur,next)/(math.dist([0,0],cur)* math.dist([0,0],next)))
      theta = theta * np.sign(np.cross(cur,next)[0])
      print(f"relativ: {theta}")
      dist = math.dist([0,0],next)
      otto.Turn(theta)
      otto.Forward(dist)