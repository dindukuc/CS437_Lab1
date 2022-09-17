import time
import picar_4wd as fc              #import picar object
import numpy as np 
import math
import matplotlib.pyplot as plt






def measure_dist():
    obj_dist = []
    for i in range(-80, 90, 10):
        dist_val = fc.get_distance_at(i)
        time.sleep(.05)
        obj_dist.append((i, dist_val))
    
    print(obj_dist)
    return obj_dist







def mapping(grid, curr_pos):
    pass;




def main():
    grid = np.array((90, 100))
    curr_pos = (50, 0)
    grid = mapping(grid, curr_pos)

    plt.imshow(grid, origin="lower")
    plt.show()




if __name__ == "__main__":
    try:
        measure_dist()
    finally:
        fc.stop()