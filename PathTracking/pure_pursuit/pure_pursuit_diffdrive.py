import numpy as np
import math
import matplotlib.pyplot as plt
from math import *

k = 0.1  # look forward gain
Lfc = 1.0  # look-ahead distance
Kp = 1.0  # speed propotional gain
dt = 0.1  # [s]
L = 2.9  # [m] wheel base of vehicle


show_animation = True


class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v


def update(state, w, dt):
    state.x += state.v * math.cos(state.yaw)*dt
    state.y += state.v * math.sin(state.yaw)*dt
    state.yaw += w*dt
    return state


def PIDControl(target, current, dist):
    kv = 0.1
    v = target
    if dist < 3:
        v = kv * dist  # speed proportional to distance to goal
    target = copysign(min(abs(target), v), target) # add direction, yes
    a = Kp * (target - current)

    a = copysign(min(abs(a), 1), a) # acceleration limit between limit velocity
    return a


def pure_pursuit_control(state, cx, cy, pind, direction=1):

    ind = calc_target_index(state, cx, cy)

    if pind >= ind:
        ind = pind

    if ind < len(cx):
        tx = cx[ind]
        ty = cy[ind]
    else:
        tx = cx[-1]
        ty = cy[-1]
        ind = len(cx) - 1

    
    alpha = math.atan2(ty - state.y, tx-state.x) - state.yaw

    dist = sqrt((ty-state.y)**2+(tx-state.x)**2)


    if direction == 1:
        alpha = math.pi-alpha

    k = 2*sin(alpha)/dist

    # no drive back, even though it is better sometimes
    # if state.v < 0:  # back
    #     alpha = math.pi - alpha

    # when speed is 0, it's possible to drive back
    # but when driving, don't try to drive back

    w = state.v * k

    return w, ind, k


def calc_target_index(state, cx, cy):

    # search nearest point index
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]
    d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
    ind = d.index(min(d))
    L = 0.0

    Lf = k * state.v + Lfc

    # search look ahead target point index
    while Lf > L and (ind + 1) < len(cx):
        dx = cx[ind + 1] - cx[ind]
        dy = cy[ind + 1] - cy[ind]
        L += math.sqrt(dx ** 2 + dy ** 2)
        ind += 1 # index will not large than max length

    return ind

# 
def predict(cx, cy, target_speed, T = 5.0, dt=0.1):
    # initial state
    state = State(x=-0.0, y=-3.0, yaw=0.0, v=0.0)

    lastIndex = len(cx) - 1
    time = 0.0
    x = [state.x]
    y = [state.y]
    tx,ty=cx[-1],cy[-1]

    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    w = [0.0]

    target_ind = calc_target_index(state, cx, cy)

    
    dist = math.sqrt((tx-state.x)**2+(ty-state.y)**2)

    while T >= time and dist>0.1:
        dist = math.sqrt((tx-state.x)**2+(ty-state.y)**2)

        ai = PIDControl(target_speed, state.v)
        wi, target_ind = pure_pursuit_control(state, cx, cy, target_ind)

        state.v = 0.3

        state = update(state, wi, dt)

        time = time + dt

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)
        w.append(wi)

def main():
    #  target course
    # cx = np.arange(0, 5., 0.1)
    cx = np.arange(0, 0.1, 0.1)
    cy = [math.sin(ix / 5.0) * ix / 2.0 for ix in cx]

    target_speed = 10.0 / 3.6  # [m/s]

    T = 100.0  # max simulation time

    # initial state
    state = State(x=-0.0, y=-0.3, yaw=0.0, v=0.0)

    lastIndex = len(cx) - 1
    time = 0.0
    x = [state.x]
    y = [state.y]
    tx,ty=cx[-1],cy[-1]

    yaw = [state.yaw]
    v = [state.v]
    t = [0.0]
    w = [0.0]
    k = [0.0]

    target_ind = calc_target_index(state, cx, cy)

    
    dist = math.sqrt((tx-state.x)**2+(ty-state.y)**2)

    direction = -1
    while T >= time and dist>0.1:
        dist = math.sqrt((tx-state.x)**2+(ty-state.y)**2)

        ai = PIDControl(target_speed*direction, state.v, dist)
        wi, target_ind, ki = pure_pursuit_control(state, cx, cy, target_ind, direction)

        # state.v = 0.3
        print ai
        
        state.v += ai*dt # smooth increase velocity
        
        # make sure path curvature is continous, then rotation velocity will
        # not jump heavily
        if abs(wi)>45/180.*math.pi: # limit rotation speed
            wi = 1*direction
            state.v = min(state.v, wi/ki) # decrease velocity when curvature is too big
            # or keep velocity unchanged
        

        state = update(state, wi, dt)

        time = time + dt

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        t.append(time)
        w.append(wi)
        k.append(ki)

        if False:
            plt.cla()
            plt.plot(cx, cy, ".r", label="course")
            plt.plot(x, y, "-b", label="trajectory")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
            plt.pause(0.001)

    if show_animation:
        plt.plot(cx, cy, ".r", label="course")
        plt.plot(x, y, "-b", label="trajectory")
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.axis([-2, 2, -2, 2])
        plt.grid(True)

        fig, ax = plt.subplots(1)
        plt.scatter(t, [iv * 3.6 for iv in v])
        plt.xlabel("Time[s]")
        plt.ylabel("Speed[km/h]")
        plt.grid(True)

        fig, ax = plt.subplots(1)
        plt.scatter(t, w)
        plt.xlabel("Time[s]")
        plt.ylabel("w")
        plt.grid(True)

        fig, ax = plt.subplots(1)
        plt.scatter(t, k)
        plt.xlabel("Time[s]")
        plt.ylabel("k")
        plt.grid(True)
        plt.show()

if __name__ == '__main__':
    main()
    