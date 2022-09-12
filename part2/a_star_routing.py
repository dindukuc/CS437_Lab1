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
    parent = None
    dist = -1

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
        temp = key_gen(idx[0], idx[1])
        all_nodes_list[temp] = int(val)
        # temp = None
        # break;

    # print(all_nodes_list["89,89"])
    return all_nodes_list

def manhattan_dist(point1, point2):
    dist = abs(point1.x - point2.x) + abs(point1.y - point2.y)

    return dist

def key_gen(x, y):
    return str(x) + "," + str(y)



def compute_successors(node, all_nodes_list, goal):
    successors = [] 
    coords = []
    

    #first successor x-1
    if node.x-1 >= 0:
        coords.append((node.x-1, node.y))    

    #second successor x+1
    if node.x+1 <= 89:
        coords.append((node.x+1, node.y))   

    #third successor y-1
    if node.y-1 >= 0:
        coords.append((node.x, node.y-1))


    #fourth successor y+1
    if node.y+1 <= 89:
        coords.append((node.x, node.y+1))
    
    # print(coords)

    for coordinate in coords:
        key = key_gen(coordinate[0], coordinate[1])
        obj_detect_flag = all_nodes_list[key]
        if obj_detect_flag == 0: # only if there is no object in that node
            temp_point = Point(coordinate[0], coordinate[1], all_nodes_list[key])
            temp_point.dist = manhattan_dist(node, temp_point) + manhattan_dist(temp_point, goal) #get distance of parent to successor  and successor to the goal and add them up
            temp_point.parent = node
            successors.append(temp_point)

        
    # for point in successors:
    #     print(point.key, point.dist)

    return successors


#checks to see if a lower version of the node is already in a list and replaces it if it's not
#returns new version of list
def replace_node_in_open(node, open):
    
    node_idx = open.index(node)
    node_from_list = open[node_idx]
    
    if node.dist < node_from_list.dist:
        open.pop(node_idx)
        open.append(node)
    
    return open


def lower_node_in_list(node, closed):
    node_idx = closed.index(node)
    node_from_list = closed[node_idx]
    
    if node.dist < node_from_list.dist:
        return False
    
    return True




def a_star(all_nodes_list, start, goal):
    closed = []
    open = []
    start.dist = 0
    open.append(start)

    successors = []
    
    while open:
        open.sort(key= lambda x: x.dist)
        curr_node = open.pop(0)
        successors = compute_successors(curr_node, all_nodes_list, goal)

        for node in successors:
            if node == goal:
                # print(node.key)
                return node

            if node in open:
                open = replace_node_in_open(node, open)

            elif node in closed:
                if lower_node_in_list(node, closed) == False:
                    open.append(node)    
            else:
                open.append(node)
        
        # print(curr_node.key)
        closed.append(curr_node)
        
        
        

def get_path(goal):
    path = []
    curr_node = goal

    while curr_node is not None:
        path.append((curr_node.x, curr_node.y))
        curr_node = curr_node.parent
    
    path.reverse()
    print(path)
    return path







if __name__ == "__main__":
    start = Point(50, 0, 0)
    goal = Point(89, 89, 0)
    
    grid = np.zeros((90,90))
    # grid[89,89] = 4


    all_nodes_list = convert_grid(grid)
    goal = a_star(all_nodes_list, start, goal)

    get_path(goal)

    # print(all_nodes_list["50,0"].key)

    # plt.imshow(grid, origin="lower")
    # plt.show()
