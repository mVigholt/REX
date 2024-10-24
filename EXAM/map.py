"""
Module for interfacing a 2D Map in the form of Grid Occupancy
"""

import numpy as np
import help

class landmark_map(object):
    """

    """
    def __init__(self, low=(0, 0), high=(2, 2), res=0.05, landMarks=[]) -> None:
        self.map_area = [low, high]    #a rectangular area    
        self.map_size = np.array([high[0]-low[0], high[1]-low[1]])
        self.resolution = res
        self.landMarks = landMarks

        self.extent = [self.map_area[0][0], self.map_area[1][0], self.map_area[0][1], self.map_area[1][1]]

    # Andreas WIP
    def in_collision(self, landMark, pos):
        """
        pos is a nodes pos
        """
        return help.collission(landMark, pos)