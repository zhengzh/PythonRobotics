from __future__ import division, print_function
from math import *

import numpy as np
import matplotlib.pyplot as plt

k_ = 0.1  # look forward gain
kp_ = 1  # speed propotional gain
kdist_ = 0.1  # distance gain
ld_ = 1.
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


def pid(current, target_speed, dist_to_end):

    target = min(abs(target_speed), kdist_ * dist_to_end)
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