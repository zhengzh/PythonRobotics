import numpy as np
import matplotlib.pyplot as plt
from math import *
from pure_pursuit_class import PurePursuit, State, distance


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

    pp = PurePursuit()

    pp.set_path(cx, cy)

    while T >= time and dist_to_end > 0.01:

        # vi, wi, ki, idx = pure_pursuit(state, cx, cy, idx, target_speed, dt)
        vi, wi, ki = pp.pure_pursuit(state, target_speed)

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
        plt.arrow(x,
                  y,
                  length * cos(yaw),
                  length * sin(yaw),
                  fc=fc,
                  ec=ec,
                  head_width=width,
                  head_length=width,
                  alpha=0.4)
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

    x, y, yaw, v, w, k, t = pure_pursuit_predict(state, cx, cy, 0, target_speed,
                                                 0.1, 100)

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
