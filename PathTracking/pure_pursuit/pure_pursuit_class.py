from __future__ import division, print_function
from math import *


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


def calculate_kappa(state, tx, ty):

    dist = sqrt((ty - state.y)**2 + (tx - state.x)**2)
    alpha = atan2(ty - state.y, tx - state.x) - state.yaw
    k = 2 * sin(alpha) / dist  # calculate curvature

    return k


def distance(x1, y1, x2, y2):

    return sqrt((x1 - x2)**2 + (y1 - y2)**2)


class PurePursuit(object):

    def __init__(self):

        self.ld = 1.0
        self.k = 0.1  # look forward gain
        self.kp = 1.  # speed propotional gain
        self.k_dist = 1.  # distance gain
        self.dt = 0.1
        self.v_max = 1.5
        self.w_max = 1.
        self.acc_max = 2.

    def set_path(self, cx, cy):

        self.index = 0
        self.cx, self.cy = cx, cy

    def pid(self, current, target_speed, dist_to_end):

        target = min(abs(target_speed), self.k_dist * dist_to_end)
        target = copysign(target, target_speed)

        acc = self.kp * (target - current)
        acc = max(min(acc, self.acc_max), -self.acc_max)

        return acc

    def calculate_vw(self, v, acc, kappa):

        v = v + acc * self.dt

        if kappa > 1e-5:
            w = v * kappa
            w = max(min(w, self.w_max), -self.w_max)
            v = w / kappa
        else:
            w = 0.

        return v, w

    def calculate_target_index(self, state):

        index = self.index

        look_ahead_dist = self.ld + self.k * state.v
        while index < len(self.cx) - 1:

            dx = self.cx[index] - state.x
            dy = self.cy[index] - state.y
            dist = sqrt(dx**2 + dy**2)
            if dist > look_ahead_dist: break

            index += 1

        return index

    def pure_pursuit(self, state, target_speed):

        dist_to_end = distance(self.cx[-1], self.cy[-1], state.x, state.y)
        self.index = self.calculate_target_index(state)

        kappa = calculate_kappa(state, self.cx[self.index],
                                self.cy[self.index])
        acc = self.pid(state.v, target_speed, dist_to_end)
        v, w = self.calculate_vw(state.v, acc, kappa)

        return v, w, kappa