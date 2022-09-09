import time
import picar_4wd as fc #import picar object
from picar_4wd.servo import Servo #import picar servo object
from picar_4wd.pwm import PWM     #import picar PWM object 
from picar_4wd.filedb import FileDB #import picar config file 


#read configuration values from config file
config = FileDB("config")

#set servo offset value from config file
ultrasonic_servo_offset = int(config.get('ultrasonic_servo_offset', default_value = 0))

#initial variables setup
#Most of this setup was taken from each of the initialization of the servo, PWM and __init__ files
speed = 1
# Ultrasonic
ANGLE_RANGE = 180 #degrees; range of angle for servo
STEP = 18 #degrees; takes a reading every 18 degrees with current value
us_step = STEP 
angle_distance = [0,0]
current_angle = 0
max_angle = ANGLE_RANGE/2 #setting the max angle that the servo moves
min_angle = -ANGLE_RANGE/2 #setting the min angle that the servo moves
scan_list = []          
obstacle_direction=""   
errors = []
scanned_area_flags =[]

# print("Init Servo: %s" % ultrasonic_servo_offset)
#set offset for servo, based on value read from config file
servo = Servo(PWM("P0"), offset=ultrasonic_servo_offset)

#main function
def main():
    global scanned_area_flags
    servo.set_angle(0) #set current angle of servo to point straight forward (at 0 degrees)
    time.sleep(0.1) 
    #main loop that runs the 
    while True:
        scan_list = scan_step(35)
        if not scan_list:
            continue
        elif len(scan_list) != 10:
            continue
        scanned_area_flags = scan_list[3:6]

        if scanned_area_flags != [2,2,2]:
            fc.stop()
            backup()
            chooseOtherPath()
        else:
            fc.forward(speed)

#function that moves the picar backwards
def backup():
    global scanned_area_flags
    
    #keeps moving backwards until the picar doesn't detect anything infront of it
    while scanned_area_flags != [2,2,2]:
        fc.backward(1) 
        #get ultrasonic values
        scan_list = scan_step(35)
        if not scan_list:
            continue
        elif len(scan_list) != 10:
            continue
        #only keep the ultrasonic values that are in-front of the picar
        scanned_area_flags = scan_list[3:6]


#function that chooses which way to turn based on the direction of the obstacle
def chooseOtherPath():
    global obstacle_direction
    fc.stop()
    time.sleep(0.40)

    #global variable stores where the obstacle was detected
    # Turns away from the direction of the obstacle    
    if obstacle_direction=="left":
        fc.turn_right(0.1)
    if obstacle_direction=="right":
        fc.turn_left(0.1)
    obstacle_direction=""
    time.sleep(0.30)


#scan step function that gets and stores the distance value from the ultrasonic sensor at each step
def scan_step(ref):
    global scan_list, current_angle, us_step, obstacle_direction
    current_angle += us_step
    if current_angle >= max_angle:
        current_angle = max_angle
        us_step = -STEP
    elif current_angle <= min_angle:
        current_angle = min_angle
        us_step = STEP
    status = fc.get_status_at(current_angle, ref1=ref)#ref1
    scan_list.append(status)

    #determines which direction an obstacle exists and then sets the obstacle direction global variable
    if current_angle == min_angle or current_angle == max_angle:
        if us_step < 0:
            print(current_angle)
            # print("reverse")
            obstacle_direction="left"
            print("left")
            scan_list.reverse()
        # print(scan_list)
        else:
            print("right")
            obstacle_direction="right"

        tmp = scan_list.copy()
        scan_list = []
        return tmp
    else:
        return False


#code block that runs the main function
if __name__ == "__main__":
    try:
        main()
    finally:
        fc.stop()