import time
import picar_4wd as fc              #import picar object
import numpy as np 
import math
import matplotlib.pyplot as plt




def euclidean_dist(point1, point2):
    return math.dist(point1, point2)

def calc_slope_intercept(point1, point2):
    x1 = point1[0]
    y1 = point1[1]

    x2 = point2[0]
    y2 = point2[1] 
    
    if x2-x1 != 0:
        slope = (y2-y1)/(x2-x1)
    else:
        slope = 0

    intercept = slope*x1 - y1

    return slope, intercept


def calculate_coords(obj_dist, curr_pos):
    obj_coords = []
    obj_x = -1
    obj_y = -1

    for obj in obj_dist:
        obj_x = round(curr_pos[0] + obj[1] * math.sin(math.radians(obj[0])))
        obj_y = round(curr_pos[1] + obj[1] * math.cos(math.radians(obj[0])))
        
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


def gen_interp_points(point1, point2):
    slope, intercept = calc_slope_intercept(point1, point2)
    
    x1 = point1[0]
    y1 = point1[1]

    x2 = point2[0]
    y2 = point2[1] 

    y = -1
    interp_coords = []

    for x in range(x1, x2):
        y = round(x*slope + intercept)
        interp_coords.append((x, y))



def interpolation(obj_coord):
    next_obj = (-1,-1)
    
    interp_coords = []

    for idx, obj in enumerate(obj_coord):
        if idx+1 < len(obj_coord):
            next_obj = obj_coord[idx+1]

            if euclidean_dist(obj, next_obj) <= 5:
                interp_coords.append(gen_interp_points(obj, next_obj))
    
    return interp_coords





def mapping(grid, curr_pos):
    obj_dist = measure_dist()
    obj_coords = calculate_coords(obj_dist, curr_pos)
    obj_coords.append(interpolation(obj_coords))
    grid = place_objs(grid, obj_coords)

    return grid





def main():
    grid = np.zeros((90, 100))
    curr_pos = (50, 0)
    grid = mapping(grid, curr_pos)

    plt.imshow(grid, origin="lower")
    plt.show()




if __name__ == "__main__":
    try:
        main()
    finally:
        fc.stop()