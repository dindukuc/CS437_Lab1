import time
import picar_4wd as fc              #import picar object
import numpy as np 
import math
import matplotlib.pyplot as plt



def calculate_coords(obj_dist, curr_pos):
    obj_coords = []
    obj_x = -1
    obj_y = -1

    for obj in obj_dist:
        obj_x = curr_pos[0] + obj[1] * math.sin(math.radians(obj[0]))
        obj_y = curr_pos[1] + obj[1] * math.cos(math.radians(obj[0]))
        obj_coords.append((obj_x, obj_y))
    

    return obj_coords
    

def measure_dist():
    obj_dist = []
    for i in range(-80, 90, 10):
        dist_val = fc.get_distance_at(i)
        time.sleep(.05)
        obj_dist.append((i, dist_val))
    
    # print(obj_dist)
    return obj_dist


def valid_coord(coord):
    x = coord[0]
    y = coord[1]

    if x >= 0 and x < 90 and y >= 0 and y < 100:
        return True
    else:
        return False


def place_objs(grid, obj_coords):
    x = -1
    y = -1
    
    for obj in obj_coords:
        x = int(obj[0])
        y = int(obj[1])
        if valid_coord((x, y)) == True:
            grid[x, y] = 1        
        else:
            continue;
    
    return grid

    

def mapping(grid, curr_pos):
    obj_dist = measure_dist()
    obj_coords = calculate_coords(obj_dist, curr_pos)
    grid = place_objs(obj_coords)
    # do interpolation and padding

    return grid





def main():
    grid = np.array((90, 100))
    curr_pos = (50, 0)
    grid = mapping(grid, curr_pos)

    plt.imshow(grid, origin="lower")
    plt.show()




if __name__ == "__main__":
    try:
        main()
    finally:
        fc.stop()