import picar_4wd as fc
import time
import sys
import math
import numpy as np
import matplotlib.pyplot as plt
import stop_sign_detection as st


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


# ***************** Mapping functions***************

#main function
def generate_grid(curr_pos):
    servo.set_angle(intial_angle)
    time.sleep(1)
    AD=mesure_dist()
    IP=interpolate(AD)
    AX=calculate_xy(IP, curr_pos)
    CRD=coordinates(AX)
    grid = mapping(CRD)
    return grid


            
def mesure_dist():
    for i in range(intial_angle, -90, -10):
        dist = fc.get_distance_at(i)
        time.sleep(0.2)
        AD.append([i, dist])
        # print(i, dist)        
    # print(AD)
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
                
            # print(ip1)
            # print(ip2)
            
            ip3 = np.stack((ip1,ip2), axis=1)
            ip3 = np.array(ip3)
            # print(ip3)
            
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
    # print(IP)        
    return(IP)    

def calculate_xy(IP, curr_pos):
    # converting list to array
    arr = np.array(IP)
    for i in range(len(IP)):
        if (arr[i,1]>0 and arr[i,1]<100):
            x_axis = arr[i,1]*math.sin(math.radians(arr[i,0]))
            y_axis = arr[i,1]*math.cos(math.radians(arr[i,0]))
            x1 = curr_pos[0] - round(x_axis)         # assuming car is at (50, 0) -ve are in the right  
            y1 = curr_pos[1] - round(y_axis)            
            #print (x_axis, y_axis)
        else:
            continue
        AX.append([x1, y1])     
    # print(AX) 
    return(AX)
  

def coordinates(AX):
    arr1 = np.array(AX) 
    np_shape = (90, 100)
    CRD = np.zeros(np_shape)
    
    for i in range(len(AX)):
    
        test_x = AX[i][0]
        test_y = AX[i][1]
        if (test_x > 0 and test_x < 90):
           CRD[test_x,test_y] = 1
        #test.append([test_x,test_y])
        CRD_int = CRD.astype(int)
        CRD = CRD_int
        # np.savetxt('/home/pi/picar-4wd/labwork/crd_withIP.txt', CRD)
    # print(CRD)
    
    return(CRD)
    
def mapping(CRD):
    arr2 = np.array(CRD)
    # print(arr2.shape)
    # plt.imshow(arr2, origin="lower")
    #plt.plot(arr2)
    # plt.show() 

    return arr2
    




# ***************** Mapping functions end***************




# ***************** Movement functions***************
def move_1cm():
    fc.forward(5)
    time.sleep(.05)
    fc.stop()


def turn_90_left():
    fc.turn_left(33)
    time.sleep(1)
    fc.stop()

def turn_90_right():
    fc.turn_right(33)
    time.sleep(1)
    fc.stop()

def move_1cm_back():
    fc.backward(5)
    time.sleep(.05)
    fc.stop()

def move(cmd):
    if cmd == "forward":
        move_1cm()
    
    elif cmd == "left":
        turn_90_left()
        move_1cm()
    
    elif cmd == "right":
        turn_90_right()
        move_1cm()
    
    elif cmd == "backward":
        move_1cm_back()
    
def reset_heading(curr_heading):

    if curr_heading == "forward":
        pass;
    elif curr_heading == "left":
        turn_90_right()
    elif curr_heading == "right":
        turn_90_left()
    elif curr_heading == "backward":
        turn_90_left()
        time.sleep(.1)
        turn_90_left()

    return "forward"



# ***************** Movement functions end***************

# ***************** A* functions***************
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
    if node.y+1 <= 99:
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
    # print(path)
    return path


def forward_heading(coord, next_coord):

    if next_coord[0] > coord[0]:
        return "right", "right"
    elif next_coord[0] < coord[0]:
        return "left", "left"
    elif next_coord[1] > coord[1]:
        return "forward", "forward"
    elif next_coord[1] < coord[1]:
        return "backward", "forward"

def right_heading(coord, next_coord):

    if next_coord[0] > coord[0]:
        return "forward", "right"
    elif next_coord[0] < coord[0]:
        return "backward", "right"
    elif next_coord[1] > coord[1]:
        return "left", "forward"
    elif next_coord[1] < coord[1]:
        return "right", "backward"

def left_heading(coord, next_coord):

    if next_coord[0] > coord[0]:
        return "backward", "left"
    elif next_coord[0] < coord[0]:
        return "forward", "left"
    elif next_coord[1] > coord[1]:
        return "right", "forward"
    elif next_coord[1] < coord[1]:
        return "left", "backward"

def backward_heading(coord, next_coord):

    if next_coord[0] > coord[0]:
        return "left", "right"
    elif next_coord[0] < coord[0]:
        return "right", "left"
    elif next_coord[1] > coord[1]:
        return "backward", "backward"
    elif next_coord[1] < coord[1]:
        return "forward", "backward"


def commands(path, heading):
    command_list = []
    next_coord = (-1, -1)
    temp = ""
    # heading = "forward"
    heading_list = []

    # print(len(path))

    for idx, coord in enumerate(path):
        

        if idx + 1 >= len(path):
            return command_list, heading_list
        
        next_coord = path[idx+1]

        # print("here at idx: ", idx, coord, next_coord)


        if heading == "forward":
            temp, heading = forward_heading(coord, next_coord)
            command_list.append(temp)
            heading_list.append(heading)
            # print("here at idx: ", idx, temp)
            # print(temp)
            

        elif heading == "right":
            temp, heading = right_heading(coord, next_coord)
            command_list.append(temp)
            heading_list.append(heading)
            # print("here at idx: ", idx, temp)
            # print(temp)
        
        elif heading == "left":
            temp, heading = left_heading(coord, next_coord)
            command_list.append(temp)
            heading_list.append(heading)
            # print("here at idx: ", idx, temp)
            # print(temp)
        
        elif heading == "backward":
            temp, heading = backward_heading(coord, next_coord)
            command_list.append(temp)
            heading_list.append(heading)
            # print("here at idx: ", idx, temp)
            # print(temp)            
        



def search(grid, start, goal):
    start = Point(start[0], start[1], 0)
    goal = Point(goal[0], goal[1], 0)
    
    # grid = np.zeros((90,100))
    # grid[51, 0:50] = 1


    all_nodes_list = convert_grid(grid)
    goal = a_star(all_nodes_list, start, goal)

    path = get_path(goal)
    # print(path)

    for coord in path:
        grid[coord[0], coord[1]] = 2 # have to swap x and y because python is row major and just for visualization's sake

    # grid[89, 99] = 4
    # print("Lenght of path: ", len(path))
    # print(grid[89,8])

    curr_heading = "forward"
    
    command_path, heading_path = commands(path, curr_heading)

    # plt.imshow(grid, origin="lower")
    # plt.show()

    return path, command_path, heading_path
    # print(command_path)
    # print(heading_path)
    # print("left" in command_path)
    

# ***************** A* functions end***************

# ***************** object detection functions***************

def object_detected(fps):
    return st.detect_stop_sign(fps)

# ***************** object detection functions end***************





if __name__ == "__main__":
    actual_path_taken = []
    
    grid = np.zeros((90,100))
    # grid[51, 0:50] = 1
    path = []
    cmd_list = []
    heading_list = []

    curr_pos = (50, 0)
    goal = (89, 99)
    curr_heading = "forward"

    while curr_pos != goal:
        #reset heading so the car is now facing forwards
        curr_heading = reset_heading(curr_heading)
        
        print("Current Position: ", curr_pos)
        grid = generate_grid(curr_pos) #mapping function goes here, takes curr_pos and returns grid
        #print(grid)

        # grid = np.zeros((90,100))
        # grid[51, 0:50] = 1


        time.sleep(.001)

        path, cmd_list, heading_list = search(grid, curr_pos, goal)
        

        time.sleep(.001)

        # print(path)
        # print(cmd_list)
        # print(heading_list)

        for i in range(10):
            
            while object_detected(15) == True:
                print("STOP SIGN DETECTED! Waiting for 4 seconds before checking again...")
                time.sleep(4)
            
            if curr_pos == goal:
                break;

            move(cmd_list[i])
            curr_heading = heading_list[i]
            print("current position: ", curr_pos, "current i: ", i, "Len of cmd_list: ", len(cmd_list))
            time.sleep(.001)
            actual_path_taken.append(curr_pos)
            curr_pos = path[i+1]
            
            
                
    
    # print("Actual path matches a* path: ", len(actual_path_taken) == len(path))
    
    for coord in actual_path_taken:
        grid[coord[0], coord[1]] = 2 # have to swap x and y because python is row major and just for visualization's sake
    

    plt.imshow(grid, origin="lower")
    plt.show()





