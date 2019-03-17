from math import *
import heapq

def proj(x, y, u, v):
    n = (x*u+y*v)/(u*u+v*v)
    return (n*u, n*v)

# vector angle
# vector 
# point to line segment distance
# x axis inverse clock rotate pi/2 to y axis
# vector  
# [[c, -s]
#  [s,  c]]
# 

def norm(x, y):
    return sqrt(x*x+y*y)

def rotate(x, y, theta):
    rx=x*cos(theta)-y*sin(theta)
    ry=x*sin(theta)+y*cos(theta)
    return rx, ry

def distance(x, y, u, v):
    sq_dist = (x-u)**2+(y-v)**2
    return sqrt(sq_dist)

def vertical_dist(x, y, u, v):
    px, py = proj(x, y, u, v)
    return distance(x, y, px, py)

def perpendicular(x, y):
    return norm([-y, x])

def angle(x1, y1, x2, y2):
    return acos((x1*x2+y1*y2)/(norm(x1, y1)*norm(x2,y2)))