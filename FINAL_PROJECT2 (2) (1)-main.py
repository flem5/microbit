from microbit import *
import robotbit_library as r
from robotbit_library import motor
from robotbit_library import motor_stop
#import numpy as np

M1A = 0x1
M1B = 0x2
M2A = 0x3
M2B = 0x4
r.setup()
#switched sizes to allow for easier comprehension of program
x_size = 9;
y_size = 6;

# GLOBAL ARRAY representation of grid world using a 2-Dimensional arrays
# 0 = open space
# 1 = barrier
# 2 = goal
# 99 = robot

#map initialized by user input
map_ = [[0, 0, 0, 0, 0, 0],
[0, 0, 0, 0, 0, 0],
[1, 1, 0, 0, 1, 0],
[1, 0, 0, 0, 1, 0],
[0, 0, 1, 1, 1, 0],
[0, 0, 1, 0, 0, 0],
[0, 0, 1, 0, 1, 1],
[0, 1, 1, 2, 0, 0],
[0, 99, 1, 1, 0, 0]];


# Function moves the robot 10 inches forward by moving the motors in the same direction at the same speed and using a length
#of time to determine how far the robot will move forward, the speeds are passed to the robotbit function ('motor')
def forward():
    r.motor(M2B, -255) #left
    r.motor(M1A, 254) #right
    sleep(880)
    motor_stop(M1A)
    motor_stop(M2B)
    sleep(500)
    return
# Function moves thr robot 90 degrees left by moving the motors in the opposite directions at the same speed and using a length
#of time to determine how far the robot will turn, the speeds are passed to the robotbit function ('motor')
def turnLeft90():
    r.motor(M2B, -258)
    r.motor(M1A, -255)
    sleep(324)
    motor_stop(M1A)
    motor_stop(M2B)
    sleep(500)
    return    
 # Function moves thr robot 90 degrees right by moving the motors in the opposite directions at the same speed and using a length
#of time to determine how far the robot will turn, the speeds are passed to the robotbit function ('motor')   
def turnRight90():
    r.motor(M2B, 255)
    r.motor(M1A, 259)
    sleep(335)
    motor_stop(M1A)
    motor_stop(M2B)
    sleep(500)
    return  
# Function finds the x and y postion of the starting position of the robot in the map

def start_pos():
    startrow = 0
    startcolumn = 0
    for x in range(x_size):    
        for y in range(y_size):
            if map_[x][y] == 99:
                startrow = x
                startcolumn = y

    return navgoal(startrow,startcolumn)
    
#Creates a set of directions for the robot to follow to reach the goal in the least amount of steps in a list called move
    # if the if the robot is on the edge  then allow other directions are checked besides the direction of the edge
#the first step is found by finding the minimum value surrounding the robot that is not 1 and obv 0(no zeroe s after wavefrontsearch function)
#follows the path by pushing robot to the route with minumum steps 

def navgoal (x, y):
    min = 99
    a = x
    b = y
    move = []
    direction = 0
    
    #UP
    if x > 0:
        if map_[x-1][y] < min and map_[x-1][y] > 1:
            min = map_[x-1][y]
            a = x - 1
            b = y
            direction = 0
    
    #DOWN
    if x < x_size - 1:
        if map_[x+1][y] < min and map_[x+1][y] > 1:
            min = map_[x+1][y]
            a = x + 1
            b = y
            direction = 2
            
    #LEFT
    if y > 0:
        if map_[x][y-1] < min and map_[x][y-1] > 1:
            min = map_[x][y-1]
            a = x
            b = y - 1
            direction = -1
            
    #RIGHT
    if y < y_size - 1:
        if map_[x][y+1] < min and map_[x][y+1] > 1:
            min = map_[x][y+1]
            a = x
            b = y + 1
            direction = 1
            
    move.append(direction)
    current = min
    
    while current != 2:
        found = 0
        #UP
        if found == 0 and a > 0:
            if map_[a-1][b] == current - 1:
                found = 1
                a -= 1
                direction = 0
    
        #DOWN
        if found == 0 and a < x_size - 1:
            if map_[a+1][b] == current - 1:
                found = 1
                a = a + 1
                direction = 2
                
        #LEFT
        if found == 0 and b > 0:
            if map_[a][b-1] == current - 1:
                found = 1
                b -= 1
                direction = -1
                
        #RIGHT
        if found == 0 and b < y_size - 1:
            if map_[a][b+1] == current - 1:
                found = 1
                b += 1
                direction = 1
        
        move.append(direction)
        current -= 1
    
    return move   
 
 #Minjoo  explain please
def moveit(move):
    #first move
    if move[0] == 0:
        forward()
    elif move[0] == 1:
        turnRight90()
        forward()
    elif move[0] == -1:
        turnLeft90()
        forward() 
    else:
        turnRight90()
        turnRight90()
        forward()
        
    #second to last move
    for i in range(1, len(move)):
        if move[i] == move[i-1]:
            forward()
        elif move[i] - move[i-1] == 1 or move[i] - move[i-1] == -3:
            turnRight90()
            forward()
        elif move[i] - move[i-1] == -1 or move[i] - move[i-1] == 3:
            turnLeft90()
            forward()
 
   
#Updates the map to create a path for the robot to travel from the end position to the start position
def WavefrontSearch():
    foundWave = True
    currentWave = 2
    
    #Looks for goal (2)
    while foundWave == True:
        foundWave = False
        for x in range (x_size):
            for y in range(y_size):
                if(map_[x][y] == currentWave):
                    goal_x = x
                    goal_y = y
                    foundWave = True
                    
                    #checks the array bounds heading NORTH
                    if goal_x > 0:
                        #checks the NORTH direcrtion
                        if map_[goal_x - 1][goal_y] == 0:
                            map_[goal_x - 1][goal_y] = currentWave + 1
                            foundWave = True
                    #checks the array bound heading SOUTH
                    if goal_x < (x_size - 1):
                        #checks the SOUTH direction
                        if map_[goal_x + 1][goal_y] == 0:
                            map_[goal_x + 1][goal_y] = currentWave + 1
                            foundWave = True
                    #checks the array bounds heading WEST
                    if goal_y > 0:
                        #checks the WEST direcrtion
                        if map_[goal_x][goal_y - 1] == 0:
                            map_[goal_x][goal_y - 1] = currentWave + 1
                            foundWave = True
                    #checks the array bounds heading EAST
                    if goal_y < (y_size - 1):
                        #checks the EAST direcrtion
                        if map_[goal_x][goal_y + 1] == 0:
                            map_[goal_x][goal_y + 1] = currentWave + 1
                            foundWave = True
        currentWave += 1
        sleep(500)    
    for row in map_:
        print(row)
    return
#runs one iteration of entire function 
while True:
    sleep(10)
    WavefrontSearch()
    moveit(start_pos())
    display.scroll('COMPLETE');
    break




