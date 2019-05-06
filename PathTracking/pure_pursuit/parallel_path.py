# /usr/bin/python
# as simple as possible

# path, parallel, which path is longest, select that one

from math import pi, sqrt, acos, atan2
import numpy as np
import matplotlib.pyplot as plt
import bisect

# asin (-pi/2, pi/2)
# acos (0, pi)
# atan (-pi/2, pi/2)
# atan2 (-pi, pi)


def pi_2_pi(x):
    if x > pi:
        x -= 2 * pi
    if x < -pi:
        x += 2 * pi
    return x


def normal(x, y):
    n = norm(x, y)
    return [x / n, y / n]


def proj(x, y, u, v):
    n = float(x * u + y * v) / (u * u + v * v)
    return (n * u, n * v)


def distance(x, y, u, v):
    sq_dist = (x - u)**2 + (y - v)**2
    return sqrt(sq_dist)


def norm(x, y):
    return sqrt(x * x + y * y)


def get_maps(maps_x, maps_y):
    maps = zip(maps_x, maps_y)
    s = 0
    maps_s = [0]
    for i in range(len(maps) - 1):
        d = distance(maps[i][0], maps[i][1], maps[i + 1][0], maps[i + 1][1])
        s += d
        maps_s.append(s)

    return maps_s


def angle(x1, y1, x2, y2):
    return acos((x1 * x2 + y1 * y2) / (norm(x1, y1) * norm(x2, y2)))


def angle2(x1, y1, x2, y2):  # 1->2

    z1 = atan2(y1, x1)
    z2 = atan2(y2, y1)
    z = pi_2_pi(z2 - z1)
    return z


def get_closest_waypoint(x, y, maps_x, maps_y):
    closest = 999999
    closest_waypoint = 0

    for i, mxy in enumerate(zip(maps_x, maps_y)):
        mx, my = mxy
        dist = distance(x, y, mx, my)
        if closest > dist:
            closest = dist
            closest_waypoint = i

    return closest_waypoint


# Choose in the map of highway waypoints the closest before the car (that is the next).
# The actual closest waypoint could be behind the car.
def get_next_waypoint(x, y, theta, maps_x, maps_y):

    closest_idx = get_closest_waypoint(x, y, maps_x, maps_y)

    if closest_idx == len(maps_x) - 1:
        return closest_idx

    map_x = maps_x[closest_idx]
    map_y = maps_y[closest_idx]

    heading = atan2(map_y - y, map_x - x)
    angle = abs(theta - heading)

    # TODO both closest and next point are behind the car
    if (angle > pi / 4):
        closest_idx += 1

    return closest_idx


def cartesian_to_frenet(x, y, theta, maps_x, maps_y, maps_s):

    next_point = get_next_waypoint(x, y, theta, maps_x, maps_y)
    if not next_point: next_point += 1

    prev_point = next_point - 1

    nx = maps_x[next_point]
    ny = maps_y[next_point]
    px = maps_x[prev_point]
    py = maps_y[prev_point]

    tan_x, tan_y = normal(nx - px, ny - py)
    perpen_x, perpen_y = -tan_y, tan_x

    proj_x, proj_y = proj(x - px, y - py, nx - px, ny - py)

    d_x, d_y = x - proj_x, y - proj_y

    d = d_x * perpen_x + d_y * perpen_y

    s = proj_x * tan_x + proj_y * tan_y

    s += maps_s[prev_point]  # s < 0  fall off path

    return s, d


def resample(maps_x, maps_y, step=0.5):

    sample_x, sample_y = [], []
    for i in range(len(maps_x) - 1):
        dist = distance(maps_x[i], maps_y[i], maps_x[i + 1], maps_y[i + 1])

        n = normal(maps_x[i + 1] - maps_x[i], maps_y[i + 1] - maps_y[i])
        d = 0
        while d < dist:
            x = maps_x[i] + d * n[0]
            y = maps_y[i] + d * n[1]
            d += step
            sample_x.append(x)
            sample_y.append(y)

    # append last one
    if not sample_x[-1] == maps_x[-1]:
        sample_x.append(maps_x[-1])
        sample_y.append(maps_y[-1])

    return sample_x, sample_y


def frenet_to_cartesian(s, d, maps_x, maps_y, maps_s):

    s = min(s, maps_s[-1] - 0.01) # guard don't go too far

    next_point = bisect.bisect(maps_s, s)

    if not next_point: next_point += 1
    prev_point = next_point - 1

    nx = maps_x[next_point]
    ny = maps_y[next_point]
    px = maps_x[prev_point]
    py = maps_y[prev_point]

    tan = normal(nx - px, ny - py)

    ds = s - maps_s[prev_point]

    perpendicular = normal(py - ny, nx - px)
    nd = [d * perpendicular[0], d * perpendicular[1]]

    tan_s = [ds * tan[0], ds * tan[1]]

    res_pos = [tan_s[0] + nd[0], tan_s[1] + nd[1]]

    cart = [res_pos[0] + maps_x[prev_point], res_pos[1] + maps_y[prev_point]]

    return cart


def parallel_path(s, d, length, maps_x, maps_y, maps_s):
    # s, d = cartesian_to_frenet(x, y, 0., maps_x, maps_y, maps_s)

    x, y = frenet_to_cartesian(s, d, maps_x, maps_y, maps_s)
    step = 0.5
    paths = []
    for di in np.arange(-1.0, 1.0, 0.2):
        path = [[x, y]]
        for si in np.arange(s+1.0, 8.0, 0.1):
            px, py = frenet_to_cartesian(s + si, di, maps_x, maps_y, maps_s)
            path.append([px, py])
        paths.append(path)
    return paths


def get_maps(maps_x, maps_y):
    maps = zip(maps_x, maps_y)
    s = 0
    maps_s = [0]
    for i in range(len(maps) - 1):
        d = distance(maps[i][0], maps[i][1], maps[i + 1][0], maps[i + 1][1])
        s += d
        maps_s.append(s)

    return maps_s


def main():

    maps_x = [0, 1, 2, 3]
    maps_y = [0, 1, 5, 7]

    # pa = draw_road(maps_x, maps_y, maps_s)
    samples_x, samples_y = resample(maps_x, maps_y)
    plt.plot(maps_x, maps_y, 'r')
    # import ipdb; ipdb.set_trace()
    plt.scatter(samples_x, samples_y)
    maps_s = get_maps(maps_x, maps_y)

    s, d = cartesian_to_frenet(-1, -1, 0., maps_x, maps_y, maps_s)
    print(s, d)
    maps_s = get_maps(samples_x, samples_y)
    paths = parallel_path(-0., 0., 4.0, samples_x, samples_y, maps_s)
    # paths = parallel_path(0., 0., 4.0, maps_x, maps_y, maps_s)
    # pa_x, pa_y = zip(*pa)
    # plt.plot(pa_x, pa_y, 'b')
    for p in paths:
        plt.plot(*zip(*p))
        plt.scatter(*zip(*p))
    plt.axis('square')
    plt.show()


if __name__ == '__main__':
    main()
