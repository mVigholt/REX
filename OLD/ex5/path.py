import math as m
import particle
import numpy as np
import sys
import os

# Define the path to the directory where the desired module is located
directory_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))

sys.path.insert(0, directory_path)

import robot_models
import rrt_mod as rt
import help as h

otto = h.Arlo()

def Generate(landMarks, est_pos: particle.Particle, goal):
  
  goal = 10 * goal
  
  for i in landMarks:
    i = 10 * i
  
  path_res = 200
  expand_dis = 2000
  
  affine_matrix = np.array([[m.cos(-est_pos.getTheta()), -m.sin(-est_pos.getTheta()), -est_pos.getX()],
                            [m.sin(-est_pos.getTheta()), m.cos(-est_pos.getTheta()), -est_pos.getY()],
                            0, 0, 1])
  for landmark in landMarks:
    landmark = affine_matrix @ np.vstack((landmark, [[1]]))
  
  goal = affine_matrix @ np.vstack((goal, [[1]]))
  
  map = m.landmark_map(low=(-1500, 0), high=(1500, 3000), landMarks=landMarks)

  robot = robot_models.PointMassModel(ctrl_range=[-path_res, path_res])

  rrt = rt.RRT(
    start=[0, 0],
    goal=goal,
    robot_model=robot,
    map=map,
    expand_dis=expand_dis,
    path_resolution=path_res,
    )

  path = rrt.planning(animation=False)
  
  if path is not None:
    cur = np.array([0,1])
    for i in range(len(path)-1,0,-1):
        next = path[i-1] - path[i]
        theta = m.acos(np.dot(cur,next)/(m.dist([0,0],cur)* m.dist([0,0],next)))
        theta = theta * np.sign(np.cross(cur,next))
        dist = m.dist([0,0],next)
        print(f"turn: {theta}")
        print(f"move: {dist}")
        otto.Turn(theta)
        otto.Forward(dist)
        cur = next