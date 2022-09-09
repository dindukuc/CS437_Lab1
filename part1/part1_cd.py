import time
import random
import picar_4wd as fc #import picar object as fc

#seed random functions
random.seed()

def backup(scanned_area_flags, speed):

    while scanned_area_flags != [2, 2, 2]:
        fc.backward(speed)    
        
        scan_list = fc.scan_step(35)
        if not scan_list:
            continue;
        elif len(scan_list) != 10:
            continue;

        scanned_area_flags = scan_list[3:6]

        


def avoid_obstacle(speed):
    direction = 0 #0 is left and 1 is right

    while(True):
        scan_list = fc.scan_step(35)
        if not scan_list:
            continue;
        elif len(scan_list) != 10:
            continue;

        scanned_area_flags = scan_list[3:6]
        
        if scanned_area_flags != [2, 2, 2]:
            
            fc.stop()
            backup(scanned_area_flags, speed)
            fc.stop()
            
            direction = random.randint(0, 1) % 2
            if direction == 0:
                fc.turn_left(speed)
                time.sleep(0.4)
                print("left")
            else:
                fc.turn_right(speed)
                time.sleep(0.4)
                print("right")

        else:
            fc.forward(speed)





if __name__ == "__main__":
    speed = 30
    try:
        avoid_obstacle(speed)
    finally:
        fc.stop()