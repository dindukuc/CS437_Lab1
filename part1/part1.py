import time
import picar_4wd as fc #import picar librarie
from picar_4wd.servo import Servo #import picar servo object #do we need this?
from picar_4wd.pwm import PWM     #import picar PWM object #do we need this?
from picar_4wd.filedb import FileDB #import picar config file #do we need this?

#do we need all of this initial value setup stuff? got it we need it because scan_step was redefined in this program

#read configuration values from config file
config = FileDB("config")

#set servo offset value from config file
ultrasonic_servo_offset = int(config.get('ultrasonic_servo_offset', default_value = 0))


speed = 1
# Ultrasonic
ANGLE_RANGE = 180
STEP = 18
us_step = STEP
angle_distance = [0,0]
current_angle = 0
max_angle = ANGLE_RANGE/2
min_angle = -ANGLE_RANGE/2
scan_list = []
obstacle_direction=""
errors = []
scanned_area_flags =[]
# Init Servo
# print("Init Servo: %s" % ultrasonic_servo_offset)

servo = Servo(PWM("P0"), offset=ultrasonic_servo_offset)


def main():
    global scanned_area_flags
    servo.set_angle(0)
    time.sleep(0.1)
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

def backup():
    global scanned_area_flags
    while scanned_area_flags != [2,2,2]:
        fc.backward(1)
        scan_list = scan_step(35)
        if not scan_list:
            continue
        elif len(scan_list) != 10:
            continue
        scanned_area_flags = scan_list[3:6]


def chooseOtherPath():
    global obstacle_direction
    fc.stop()
    time.sleep(0.40)
    if obstacle_direction=="left":
        fc.turn_right(0.1)
    if obstacle_direction=="right":
        fc.turn_left(0.1)
    obstacle_direction=""
    time.sleep(0.30)

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



if __name__ == "__main__":
    try:
        main()
    finally:
        fc.stop()