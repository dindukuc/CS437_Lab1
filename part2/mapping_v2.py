import time
import picar_4wd as fc              #import picar object
import numpy as np 
import math
import matplotlib.pyplot as plt

from picar_4wd.servo import Servo   #import picar servo object
from picar_4wd.pwm import PWM       #import picar PWM object 
from picar_4wd.filedb import FileDB #import picar config file 

# Config File:
config = FileDB("config")
ultrasonic_servo_offset = int(config.get('ultrasonic_servo_offset', default_value = 0)) 


servo = Servo(PWM("P0"), offset=ultrasonic_servo_offset)

# Ultrasonic
intial_angle = 80

# Create an empty array to get values
AD=[]   # Angel and Distance
AX=[]   # Axis
CRD=[]  # Coordinate
IP=[]   # Interpolate 
test=[] # Test array


#main function
def main():
    servo.set_angle(intial_angle)
    time.sleep(1)
    AD=mesure_dist()
    IP=interpolate(AD)
    AX=calculate_xy(IP)
    CRD=coordinates(AX)
    mapping(CRD)


            
def mesure_dist():
    for i in range(intial_angle, -90, -10):
        dist = fc.get_distance_at(i)
        time.sleep(0.2)
        AD.append([i, dist])
        print(i, dist)        
    print(AD)
    return(AD)
    
def interpolate(AD):
    # converting list to array
    arr5 = np.array(AD) 
    nsteps = 10
    arr6 = arr5
    add_arr = 0
    
    #arr6 = np.array(IP)
    
    for i in range(len(AD)-1):
        if ((arr5[i,1]>0 and arr5[i,1]<100) and (arr5[i+1,1]>0 and arr5[i+1,1]<100)):
            #Angel
            start_ang = arr5[i,0]
            end_ang = arr5[i+1,0]                   
            ang_range = end_ang - start_ang
            ang_step = ang_range/nsteps
            #Distance
            start_dist = arr5[i,1]
            end_dist = arr5[i+1,1]
            dist_range = end_dist - start_dist
            dist_step = dist_range/nsteps
            
            #interp_arr = []
            ip1=[]
            ip2=[]
            #ip1 = np.array(ip1)
            
            for j1 in range(nsteps):
                #ip1[j] = round(start_ang) + round(ang_step)
                ip1.append(start_ang + (ang_step*j1))
            
            for j2 in range(nsteps):
                ip2.append(start_dist + (dist_step*j2))
                
            print(ip1)
            print(ip2)
            
            ip3 = np.stack((ip1,ip2), axis=1)
            ip3 = np.array(ip3)
            print(ip3)
            
            arr7 = np.insert(arr6,i+1+add_arr*(nsteps),ip3,axis=0)
            add_arr += 1
            arr6 = arr7
            
            IP = arr6 # Function return values
    
            #interp_arr[j,1] = start_dist + dist_step
            #print(start_ang)
            #arr6 = np.insert(arr5,i+1+add_arr*(nsteps-1),interp_arr,axis=0)
            #add_arr += 1
        else:
            continue
    print(IP)        
    return(IP)    

def calculate_xy(IP):
    # converting list to array
    arr = np.array(IP)
    for i in range(len(IP)):
        if (arr[i,1]>0 and arr[i,1]<100):
            x_axis = arr[i,1]*math.sin(math.radians(arr[i,0]))
            y_axis = arr[i,1]*math.cos(math.radians(arr[i,0]))
            x1 = 50 - round(x_axis)         # assuming car is at (50, 0) -ve are in the right  
            y1 = round(y_axis)            
            #print (x_axis, y_axis)
        else:
            continue
        AX.append([x1, y1])     
    print(AX) 
    return(AX)
  
  
def calculate_xy(AD):
    # converting list to array
    arr = np.array(AD)
    for i in range(len(AD)):
        if (arr[i,1]>0 and arr[i,1]<100):
            x_axis = arr[i,1]*math.sin(math.radians(arr[i,0]))
            y_axis = arr[i,1]*math.cos(math.radians(arr[i,0]))
            x1 = 50 - round(x_axis)         # assuming car is at (50, 0) -ve are in the right  
            y1 = round(y_axis)            
            #print (x_axis, y_axis)
        else:
            continue
        AX.append([x1, y1])     
    print(AX) 

    return(AX)


def coordinates(AX):
    arr1 = np.array(AX) 
    np_shape = (90, 100)
    CRD = np.zeros(np_shape)
    
    for i in range(len(AX)):
    
        test_x = AX[i][0]
        test_y = AX[i][1]
        if (test_x > 0 and test_x < 100):
           CRD[test_x,test_y] = 1
        #test.append([test_x,test_y])
        CRD_int = CRD.astype(int)
        CRD = CRD_int
        # np.savetxt('/home/pi/picar-4wd/labwork/crd_withIP.txt', CRD)
    print(CRD)
    
    return(CRD)
    
def mapping(CRD):
    arr2 = np.array(CRD)
    print(arr2.shape)
    plt.imshow(arr2, origin="lower")
    #plt.plot(arr2)
    plt.show()  
    

        
#code block that runs the main function
if __name__ == "__main__":
    try:
        main()
    finally:
        fc.stop()