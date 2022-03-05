import math as mt
from math import *
import numpy as np
# from scipy.spatial import distance
# import sympy as sp
# from sympy import *
def computeLineThroughTwoPoints(x1,y1,x2,y2):
    d = mt.sqrt(((x2-x1)**2)+((y2-y1)**2))
    a = (y2-y1)/d
    b = (x1-x2)/d
    c = (y1*x2-x1*y2)/d
    return [a,b,c]

def computeDistancePointToLine(x_q,y_q,x1,y1,x2,y2):
    #d1 = mt.sqrt(((x2-x1)**2)+((y2-y1)**2))
    # d2 = mt.sqrt(((x_q-x1)**2)+((y_q-y1)**2))
    # th = mt.acos((((x_q-x1)*(x1-x2))+((y_q-y1)*(y1-y2)))/(d1*d2))
    # d = abs(d2*mt.sin(th))
    (a,b,c) = computeLineThroughTwoPoints(x1,y1,x2,y2)
    e = (a*x_q)+(b*y_q)+c
    d = abs(e)
    return [d,a,b,c,e]

def computeDistancePointToSegment(x_q,y_q,x1,y1,x2,y2):
    dot1 = (((x_q-x1)*(x1-x2))+((y_q-y1)*(y1-y2)))
    dot2 = (((x_q-x2)*(x1-x2))+((y_q-y2)*(y1-y2)))
    d1 = mt.sqrt(((x2-x1)**2)+((y2-y1)**2))
    d2 = mt.sqrt(((x_q-x1)**2)+((y_q-y1)**2))
    d3 = mt.sqrt(((x_q-x2)**2)+((y_q-y2)**2))
    a1 = mt.acos(dot1/(d1*d2))
    a2 = mt.acos(dot2/(d1*d3))
    
    if(dot1*dot2<0):
      m = -(mt.tan(a1)/mt.tan(a2))
      dline = computeDistancePointToLine(x_q,y_q,x1,y1,x2,y2)
      # return [dline[0],1,x1,y1,x2,y2,((-dline[2]*dline[4])/dline[0]),((dline[1]*dline[4])/dline[0])]
      if(m>10**5):
        return [dline[0],1,x1,y1,x2,y2,(x2),(y2)]
      else:
        return [dline[0],1,x1,y1,x2,y2,((m*x2+x1)/(m+1)),((m*y2+y1)/(m+1))]
    elif(dot1*dot2>0):
        if(dot1>0):
            return [d3,0,x2,y2,0,0,((x_q-x2)/d3),((y_q-y2)/d3)]
        else:
            return [d2,0,x1,y1,0,0,((x_q-x1)/d2),((y_q-y1)/d2)]
    else:
        if(d2 == min(d2,d3)):
          return [d2,0,x1,5,0,0,((x_q-x1)/d2),((y_q-y1)/d2)]
        else:
          return [d3,0,x2,y2,0,0,((x_q-x2)/d3),((y_q-y2)/d3)]
    # second variable in the output array is 1 for 'segment' and 0 for 'vertex'

def computeDistancePointToPolygon(P,q):
  d = [0,0,0,0,0,0,0,0]
  d_min = [float('inf'),0,0,0,0,0,0,0]
  for i in range(len(P)):
    if(i == (len(P)-1)):
      d = computeDistancePointToSegment(q[0],q[1],P[i][0],P[i][1],P[0][0],P[0][1])
    else:
      d = computeDistancePointToSegment(q[0],q[1],P[i][0],P[i][1],P[(i+1)][0],P[(i+1)][1])
    if(d[0]<d_min[0]):
      d_min = d
    else:
      None
  return d_min

def computeTangentVectorToPolygon(P,q):
  d = computeDistancePointToPolygon(P,q)
  u = np.zeros(2)
  if(d[1] == 1):
    u = [((d[4]-d[2])/mt.sqrt((d[4]-d[2])**2 + (d[5]-d[3])**2)),((d[5]-d[3])/mt.sqrt((d[4]-d[2])**2 + (d[5]-d[3])**2))]
  else:
    u = [(-(-q[1]+d[3])/mt.sqrt((q[1]-d[3])**2 + (q[0]-d[2])**2)),(-(q[0]-d[2])/mt.sqrt((q[1]-d[3])**2 + (q[0]-d[2])**2))]
  return u

def dist2(a,b):
  return mt.sqrt(((a[0]-b[0])**2)+((a[1]-b[1])**2))