import math as mt
from math import *
import numpy as np
import helper 
from helper import *

def update(curr_pos, goal, step):
    return [curr_pos[0]+step*(goal[0]-curr_pos[0])/dist2(curr_pos, goal), curr_pos[1]+step*(goal[1]-curr_pos[1])/dist2(curr_pos, goal)]

def bug1(start,goal,obs,step):
    current_pos = start
    path = [start]
    while dist2(current_pos,goal)>step:
        current_pos = update(current_pos,goal,step)
        min = float('inf')
        closest = []
        # print("s2", start, current_pos)

        for P in obstaclesList:
            if(computeDistancePointToPolygon(P,current_pos)[0]<min):
                min = computeDistancePointToPolygon(P,current_pos)[0]
                closest = P
            else:
                None
            #print('path1:',path)
            # print("s3", start)
        if(helper.computeDistancePointToPolygon(closest,current_pos)[0]<step):
            # print("d2", start)
            # path.append(current_pos)
            print("Failure: There is an obstacle lying between the start and goal")
            return path
        else:
            current_pos = update(current_pos,goal,step)
            path.append(current_pos)
        
    path.append(goal)
    #   print("s1", start)
    print("Success")
    return path

if __name__=="__main__":
    obs = []
    with open('/home/kratik/catkin_ws/src/sc627_assignments/assignment_1/input.txt', 'r') as f:
        lines = f.readlines()
        # print(lines)
        # obs = []
        i = 0
        #x = np.zeros(n)
        #y = np.zeros(n)
        start = list(map(float,lines[0].split('\n')[0].strip().split(',')))
        goal = list(map(float,lines[1].split('\n')[0].strip().split(',')))
        step = float(lines[2].split('\n')[0])
        for line in lines[3:]:
            fields = line.split('\n')
            #print(fields)
            if(fields[0] == ''):
                obs.append([])
            else:
                    obs[len(obs)-1].append(list(map(float,fields[0].strip().split(','))))
        #print(fields)
        # print(start,goal,step)
        #print('obs =')
        # print(obs)
        #print(len(obs))
        #print(type(obs))
