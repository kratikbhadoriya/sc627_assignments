import math as mt
from math import *
import numpy as np
import helper 
from helper import *
import matplotlib.pyplot as plt 

def update(curr_pos, grad, step):
    return [curr_pos[0]-step*(grad[0]/dist2(grad,[0,0])), curr_pos[1]-step*(grad[1]/dist2(grad,[0,0]))]

def updaterep(totrep,pls):
    return[(totrep[0]+pls[0]),[(totrep[1][0]+pls[1][0]), (totrep[1][1]+pls[1][1])]]

def attr2(q,q_g,X,d_g):
  d = dist2(q,q_g)
  # return Piecewise(((0.5*X*(d**2)), d<=d_g), (((d_g*X*d)-(0.5*X*(d_g)**2)), d>d_g))
  if(d<=d_g):
    return [(0.5*X*(d**2)), [(X*(q[0]-q_g[0])), (X*(q[1]-q_g[1]))]]
  else:
    return [((d_g*X*d)-(0.5*X*(d_g)**2)), [(d_g*X*(q[0]-q_g[0])/d), (d_g*X*(q[1]-q_g[1])/d)]]

def rep2(q,P,n,Q_i):
  d = computeDistancePointToPolygon(P,q)
  #print("dPol",d)
  # return Piecewise(((0.5*n*(((1/d)-(1/Q_i))**2)), d<=Q_i), (0,d>Q_i))
  if(d[0]<=Q_i):
    # print(0.5*n*((1/d[0])-(1/Q_i))**2)
    # return [0.5*n*(1/d[0]-1/Q_i)**2, [(n*((1/d[0])-(1/Q_i))*(-1)*((1/d[0])**2)*((q[0]-d[6])/d[0])), (n*((1/d[0])-(1/Q_i))*(-1)*((1/d[0])**2)*((q[1]-d[7])/d[0]))]]
    return [0.5*n*(1/d[0]-1/Q_i)**2, [(n*((1/d[0])-(1/Q_i))*(-1)*((1/d[0])**2)*(computeTangentVectorToPolygon(P,q)[0])), (n*((1/d[0])-(1/Q_i))*(-1)*((1/d[0])**2)*(computeTangentVectorToPolygon(P,q)[1]))]]
  else:
    return [0, [0, 0]]

def pot(current_pos,goal,obs):
    totattr = attr2(current_pos,goal,0.8,2)
    totrep = [0, [0, 0]]
    for P in obs:
        pls = rep2(current_pos,P,0.8,2)
        totrep = updaterep(totrep,pls)
    return[totattr[0]+totrep[0],[totattr[1][0]+totrep[1][0],totattr[1][1]+totrep[1][1]]]

def potential(start,goal,obs,step):
    #print('s1:',start)
    current_pos = start
    path = [start]
    while(dist2(current_pos,goal)>1):
        current_pos = update(current_pos,pot(current_pos,goal,obs)[1],step)
        print('c1:',current_pos)
        #print('s2:',start)
        path.append(current_pos)
    path.append(goal)
    # print('s3:',start)
    print("Success")
    return path

def output(path):
    file = open('/home/kratik/catkin_ws/src/sc627_assignments/assignment_2/output.txt', "w")
    for pos in path:
        file.write(str(pos[0])+", "+str(pos[1])+"\n")
    file.close()

def plot():   
    obs = []
    with open('/home/kratik/catkin_ws/src/sc627_assignments/assignment_2/input.txt', 'r') as f:
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

	# plotting start, goal and obstacles
        for P in obs: 
            plt.gca().add_patch(plt.Polygon(P, color = "blue"))


        plt.scatter(start[0], start[1])
        plt.scatter(goal[0], goal[1])

        # plotting the traced path
        file = open("/home/kratik/catkin_ws/src/sc627_assignments/assignment_2/output.txt", "r")
        lines = file.readlines()
        X, Y = [], []
        for line in lines:
            x, y= line.split(',')
            X.append(float(x))
            Y.append(float(y))
        plt.plot(X, Y, color = "red", marker = ".", markersize=1)

        plt.show()

if __name__=="__main__":
    obs = []
    with open('/home/kratik/catkin_ws/src/sc627_assignments/assignment_2/input.txt', 'r') as f:
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
        #print(type(obs)) "home/kratik/catkin_ws/src/sc627_assignments/assignment_2/output.txt"

        potpath = potential(start,goal,obs,step)
        output(potpath)
        print(potpath)
        plot()