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
    successors = []

    def __init__(self, x, y, value):
        
        self.x = int(x)
        self.y = int(y)
        self.value = int(value)
        self.key = str(x) + "," + str(y)

    def __eq__(self, other):
        return self.key == other.key




def convert_grid(grid):
    all_nodes_list = {}

    temp = None

    for idx, val in np.ndenumerate(grid):
        temp = Point(idx[0], idx[1], val)
        all_nodes_list[temp.key] = temp.value
        temp = None
        # break;

    # print(all_nodes_list["99,99"])
    return all_nodes_list

def manhattan_dist(point1, point2):
    dist = abs(point1.x - point2.x) + abs(point1.y - point2.y)

    return dist

def compute_successors(node):
    # successors = 
    pass;




def a_star(all_nodes_list, start, goal):
    open = []
    open.append(start, 0)

    while open:
        open.sort(key= lambda tup: tup[1])
        curr_node = open.pop(0)





if __name__ == "__main__":
    start = Point(50, 0, 0)
    goal = Point(89, 89, 0)
    
    grid = np.zeros((90,90))
    # grid[99,99] = 1


    all_nodes_list = convert_grid(grid)
    # a_star(all_nodes_list, start, goal)


    # plt.imshow(grid, origin="lower")
    # plt.show()
