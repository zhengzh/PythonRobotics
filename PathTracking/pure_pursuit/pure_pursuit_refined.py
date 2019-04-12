from __future__ import division, print_function
from math import *

import numpy as np
import matplotlib.pyplot as plt


k_ = 0.1  # look forward gain
kp_ = 1  # speed propotional gain
kdist_ = 0.1  # distance gain
ld_ = 1.
dt_ = 0.1
v_max_ = 1.5
w_max_ = 1.
acc_max_ = 2.


class State:

    def __init__(self, x, y, yaw, v):

        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v

    def move(self, v, w, dt):

        x = self.x + v * dt * cos(self.yaw)
        y = self.y + v * dt * sin(self.yaw)
        yaw = self.yaw + w * dt

        return State(x, y, yaw, v)


# class Trajectory:

#     def __init__(self):

#         self.xs = []
#         self.ys = []
#         self.yaws = []
#         self.vs = []
#         self.ts = []
#         self.ws = []
#         self.ks = []

#     def append(self, x, y, yaw, v, k, w, t):

#         pass


def pid(current, target_speed, dist_to_end):

    target = min(abs(target_speed), dist_to_end)
    target = copysign(target, target_speed)

    acc = kp_ * (target - current)
    acc = max(min(acc, acc_max_), -acc_max_)

    return acc


def calculate_kappa(state, tx, ty):

    dist = sqrt((ty - state.y)**2 + (tx - state.x)**2)
    alpha = atan2(ty - state.y, tx - state.x) - state.yaw
    k = 2 * sin(alpha) / dist  # calculate curvature

    return k


def calculate_vw(prev_v, acc, kappa, dt):

    v = prev_v + acc * dt

    if kappa > 1e-5:
        w = v * kappa
        w = max(min(w, w_max_), -w_max_)
        v = w / kappa
    else:
        w = 0.

    return v, w


def calculate_target_index(state, cx, cy, prev_index):

    N = len(cx)
    index = prev_index

    look_ahead_dist = ld_ + k_ * state.v
    while index < N - 1:

        dx = cx[index] - state.x
        dy = cy[index] - state.y
        dist = sqrt(dx**2 + dy**2)

        if dist > look_ahead_dist: break

        index += 1

    return index


def calculate_nearest_index(state, cx, cy):
    # search nearest point index
    dx = [state.x - icx for icx in cx]
    dy = [state.y - icy for icy in cy]
    d = [abs(math.sqrt(idx**2 + idy**2)) for (idx, idy) in zip(dx, dy)]
    ind = d.index(min(d))

    return ind


def distance(x1, y1, x2, y2):

    return sqrt((x1 - x2)**2 + (y1 - y2)**2)


def pure_pursuit(state, cx, cy, prev_ind, target_speed, dt):

    dist_to_end = distance(cx[-1], cy[-1], state.x, state.y)
    target_index = calculate_target_index(state, cx, cy, prev_ind)

    kappa = calculate_kappa(state, cx[target_index], cy[target_index])
    acc = pid(state.v, target_speed, dist_to_end)
    v, w = calculate_vw(state.v, acc, kappa, dt)

    return v, w, kappa, target_index


def pure_pursuit_predict(state, cx, cy, start_index, target_speed, dt, T=5.0):

    x = [state.x]
    y = [state.y]
    yaw = [state.yaw]
    v = [state.v]
    w = [0.0]
    k = [0.0]
    t = [0.0]

    endx, endy = cx[-1], cy[-1]
    dist_to_end = distance(state.x, state.y, endx, endy)

    time = 0.0
    idx = start_index

    while T >= time and dist_to_end > 0.01:

        vi, wi, ki, idx = pure_pursuit(state, cx, cy, idx, target_speed, dt)
        state = state.move(vi, wi, dt)

        x.append(state.x)
        y.append(state.y)
        yaw.append(state.yaw)
        v.append(state.v)
        w.append(wi)
        k.append(ki)
        t.append(time)

        dist_to_end = distance(state.x, state.y, endx, endy)
        time += dt

    return x, y, yaw, v, w, k, t


def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
    """Plot arrow."""
    if not isinstance(x, float):
        for (ix, iy, iyaw) in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * cos(yaw), length * sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width, alpha=0.4)
        # plt.plot(x, y)

def main():
    #  target course
    # cx = np.arange(0, 5., 0.1)
    cx = np.arange(0, 0.1, 0.1)
    cy = [sin(ix / 5.0) * ix / 2.0 for ix in cx]

    target_speed = 10.0 / 3.6  # [m/s]

    T = 100.0  # max simulation time

    # initial state
    state = State(x=-0.0, y=-0.3, yaw=0.0, v=0.0)

    x, y, yaw, v, w, k, t = pure_pursuit_predict(state, cx, cy, 0, target_speed, 0.1, 100)

    for xi, yi, yawi in zip(x, y, yaw):
        plt.cla()
        plt.plot(cx, cy, ".r", label="course")
        plt.plot(x, y, "-b", label="trajectory")
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.axis([-2, 2, -2, 2])
        plt.grid(True)

        
        plot_arrow(xi, yi, yawi, length=0.2, width=0.04)

        plt.axis('equal')
        plt.pause(0.2)


    if False:
        plt.plot(cx, cy, ".r", label="course")
        plt.plot(x, y, "-b", label="trajectory")
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.axis([-2, 2, -2, 2])
        plt.grid(True)

        fig, ax = plt.subplots(1)
        plt.scatter(t, [iv for iv in v])
        plt.xlabel("Time[s]")
        plt.ylabel("Speed[m/s]")
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

        fig, ax = plt.subplots(1)
        plt.scatter(t, yaw)
        plt.xlabel("Time[s]")
        plt.ylabel("yaw")
        plt.grid(True)
        plt.show()
    

if __name__ == '__main__':
    main()