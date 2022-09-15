import picar_4wd as fc
import time
import sys
# import numpy as np
# import matplotlib.pyplot as plt



def move_1cm():
    fc.forward(10)
    time.sleep(1)
    fc.stop()


def turn_90_left():
    fc.turn_left(33)
    time.sleep(1)
    fc.stop()

def turn_90_right():
    fc.turn_right(33)
    time.sleep(1)
    fc.stop()



if __name__ == "__main__":
    # turn_90_right()
    # turn_90_left()
    move_1cm()

