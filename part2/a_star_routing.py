# import picar_4wd as fc
# import time
import sys
import numpy as np
import matplotlib.pyplot as plt


#point object that stores the x,y coords and the key and object value flag
class Point:
    x = -1
    y = -1
    key = ""
    value = -1

    def __init__(self, x, y, value):
        
        self.x = int(x)
        self.y = int(y)
        self.value = int(value)
        self.key = str(x) + "," + str(y)


def convert_grid(grid):
    closed = {}

    temp = None

    for idx, val in np.ndenumerate(grid):
        temp = Point(idx[0], idx[1], val)
        closed[temp.key] = temp.value
        temp = None
        # break;

    # print(closed["99,99"])
    return closed

def manhattan_dist(point1, point2):
    dist = abs(point1.x - point2.x) + abs(point1.y - point2.y)

    return dist




def a_star():
    pass;





if __name__ == "__main__":
    start = Point(49, 9, 0)
    goal = Point(99, 99, 0)
    
    grid = np.zeros((100,100))
    grid[99,99] = 1

    closed = convert_grid(grid)
    a_star(closed)


    # plt.imshow(grid, origin="lower")
    # plt.show()
