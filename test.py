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

otto = h.Arlo()

cam = h.Cam()

otto.Forward(100)