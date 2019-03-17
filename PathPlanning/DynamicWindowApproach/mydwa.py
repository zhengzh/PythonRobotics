
import matplotlib.pyplot as plt
from math import *
import numpy as np

# max speed, min speed, max yaw speed min yaw speed
# acceleration dt=0.1 window robot radius = 1
# world size, goal obstacles

# robot status x, y, yaw v, w
# prediction time = 3s
# prediction trajector
# cost = goal heading, distance and clearance to goal

############################
# it is stupid to sampling
# in action spaces
############################

# why python so slow
# robot will rotate or swing when can't move

# print

show_animation = True

class Config:
    def __init__(self):
        self.max_speed = 2.0
        self.min_speed = 0.0
        self.max_yaw_speed = 1
        self.min_yaw_speed = -1
        self.acc = 2
        self.dt = 0.1
        self.acc_yaw = 1
        self.prediction_time = 3
        self.v_res = 0.05
        self.yaw_res = 0.04

def move(x, u, dt):

    x=x[:]
    x[0]+=u[0]*dt*cos(x[2])
    x[1]+=u[0]*dt*sin(x[2])
    x[2]+=u[1]*dt
    x[3]=u[0]
    x[4]=u[1]

    return x

def distance(x1, y1, x2, y2):
    return sqrt((x1-x2)**2+(y1-y2)**2)

def cost_to_goal(traj, goal):
    lx, ly = traj[-1][0], traj[-1][1]
    return distance(lx, ly, *goal)

def cost_to_obstacle(traj, obs):
    for x in traj:
        if collide(x, obs):
            return False
    
    return True

def collide(x, obs):
    for o in obs:
        if distance(x[0], x[1], o[0], o[1]) < 1:
            return True
    return False

def cost_head_align(traj, map):
    pass


def cal_trajectory(x, u, dt, tt):
    traj = []
    for i in np.arange(0, tt, dt):
        x = move(x, u, dt)
        traj.append(x)
    return traj


def plot_trajectory(x, config):
    # window
    # calculate trajectory, compute cost, return min cost
    # return best action

    dt = config.dt

    vmax = min(x[3]+config.acc*dt, config.max_speed)
    vmin = max(config.min_speed, x[3]-config.acc*dt)
    ymax = min(x[4]+config.acc_yaw*dt, config.max_yaw_speed)
    ymin = max(x[4]-config.acc_yaw*dt, config.min_yaw_speed)

    lowest = 99999
    best = []
    for v in np.arange(vmin, vmax, config.v_res):
        for y in np.arange(ymin, ymax, config.yaw_res):
            traj = cal_trajectory(x, [v, y], config.dt, config.prediction_time)
            plt.plot([i[0] for i in traj], [i[1] for i in traj])
    plt.show()
    
    return lowest, best

def dwa(x, goal, map, config):
    # window
    # calculate trajectory, compute cost, return min cost
    # return best action

    dt = config.dt

    vmax = min(x[3]+config.acc*dt, config.max_speed)
    vmin = max(config.min_speed, x[3]-config.acc*dt)
    ymax = min(x[4]+config.acc_yaw*dt, config.max_yaw_speed)
    ymin = max(x[4]-config.acc_yaw*dt, config.min_yaw_speed)

    lowest = 99999
    best = []
    for v in np.arange(vmin, vmax, config.v_res):
        for y in np.arange(ymin, ymax, config.yaw_res):
            traj = cal_trajectory(x, [v, y], config.dt, config.prediction_time)
            if cost_to_obstacle(traj, map):
                c = cost_to_goal(traj, goal)
                if lowest > c:
                    lowest = c
                    best = [v, y]
    
    return lowest, best


def plot_arrow(x, y, yaw, length=1., width=0.2):
    plt.arrow(x, y, length*cos(yaw), length*sin(yaw), head_length=width, head_width=width)
    plt.plot(x, y, 'x')

if __name__ == '__main__':
    xinit = [0, 0, 0, 0, 0]
    config = Config()
    plot_trajectory(xinit, config)

if __name__ != '__main__':
    xinit = [0, 0, 0, 0, 0]
    goal = [10, 10]
    obstacles = []
    config = Config()

    fig, ax = plt.subplots(1,1)
    ob = [[-1, -1],
                    [0, 2],
                    [1, 2],
                    [2, 2],
                    [3, 2],
                    [4, 2],
                    [5, 2],
                    [5, 1],
                    [5, 0],
                    [4, 0],
                    [3, 0],
                    [2, 0],
                    [-1.5, 0],
                    [1.5, 0],
                    [0, 1.5],
                    [0, -1.5],
                    [4.0, 2.0],
                    [5.0, 4.0],
                    [5.0, 5.0],
                    [5.0, 6.0],
                    [5.0, 9.0],
                    [8.0, 9.0],
                    [10.0,10.0],
                    [7.0, 9.0],
                    [12.0, 12.0]
                    ]
    

    for step in range(500):
        if show_animation:
            plt.cla()
            plot_arrow(xinit[0], xinit[1], xinit[2])
            plt.plot(goal[0], goal[1], 'x')
            plt.plot([o[0] for o in ob], [o[1] for o in ob], 'o')
            plt.axis('equal')
            plt.pause(0.001)
        c, u = dwa(xinit, goal, ob, config)
        if u:
            xinit = move(xinit, u, config.dt)
        else:
            print("Can't Move")
    
