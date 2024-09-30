"""
Module for interfacing a 2D Map in the form of Grid Occupancy
"""

import numpy as np
# import matplotlib.pyplot as plt
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
    def in_collision(self, pos):
        """
        pos is a nodes pos
        """
        return help.collission(pos)
    
    # def draw_map(self):
    #     #note the x-y axes difference between imshow and plot
    #     plt.imshow(self.grid.T, cmap="Greys", origin='lower', vmin=0, vmax=1, extent=self.extent, interpolation='none')

# if __name__ == '__main__':
    # map = GridOccupancyMap()
    # map.populate()

    # plt.clf()
    # map.draw_map()
    # plt.show()