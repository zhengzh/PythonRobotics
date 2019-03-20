from math import sqrt, atan2, pi
import bisect


def distance(x1, y1, x2, y2):
    return sqrt((x2-x1)**2+(y2-y1)**2)

def proj(x, y, u, v):
    n = (x*u+y*v)/(u*u+v*v)
    return (n*u, n*v)

def norm(x, y):
    return sqrt(x*x+y*y)

def normal(x, y):
    n = norm(x, y)
    return [x/n, y/n]

def add(x1, y1, x2, y2):
    return [x1+x2, y1+y2]

def get_maps(maps_x, maps_y):
    maps = zip(maps_x, maps_y)
    s = 0
    maps_s = [0]
    for i in range(len(maps)-1):
        d = distance(maps[i][0], maps[i][1], maps[i+1][0], maps[i+1][1])
        s += d
        maps_s.append(s)

    return maps_s

def cartesian_to_frenet(x, y, theta, maps_x, maps_y, maps_s):

    next_point = get_next_waypoint(x, y, theta, maps_x, maps_y)
    if next_point == 0:
        print "error"

    prev_point = next_point-1
    
    nx = maps_x[next_point]
    ny = maps_y[next_point]
    px = maps_x[prev_point]
    py = maps_y[prev_point]
    
    proj_x, proj_y = proj(x-px, y-py, nx-px, ny-py)

    d = [x-proj_x, y-proj_y]

    s = maps_s[prev_point]+norm(proj_x, proj_y)

    return s, d
    


def frenet_to_cartesian(s, d, maps_x, maps_y, maps_s):

    next_point = bisect.bisect(self.s, maps_s) - 1
    prev_point = next_point - 1

    nx = maps_x[next_point]
    ny = maps_y[next_point]
    px = maps_x[prev_point]
    py = maps_y[prev_point]

    n = normal(nx-px, ny-py)
    
    ds = s - map_s[prev_point]

    tan = ds*n

    return [tan[0]+d[0], tan[1]+d[1]]


def get_closest_waypoint(x, y, maps_x, maps_y):
    closest = 999999
    closest_waypoint = 0

    for i, mx, my in enumerate(zip(maps_x, maps_y)):
        dist = distance(x, y, mx, my)
        if closest > dist:
            closest = dist
            closest_waypoint = i
    
    return closest_waypoint

def get_next_waypoint(x, y, theta, maps_x, maps_y):
    closest_waypoint = get_closest_waypoint(x, y, maps_x, maps_y)

    mx, my = maps_x[closest_waypoint], maps_y[closest_waypoint]
    
    heading = atan2(my-y, mx-x)

    angle = abs(theta-heading)

    if angle > pi/4:
        closest_waypoint+=1
    
    return closest_waypoint



# draw parallel lines
def draw_road(maps_x, maps_y, maps_s):
    max_s = maps_s[-1]
    d = 1
    parallel_line = []
    for i in range(1, len(maps_s)):
        next_point, prev_point = i, i-1
        nx = maps_x[next_point]
        ny = maps_y[next_point]
        px = maps_x[prev_point]
        py = maps_y[prev_point]

        perpend = normal(py-ny, nx-px) # [-y, x]
        parallel_line.append([perpend[0]+px, perpend[1]+py])
        parallel_line.append([perpend[0]+nx, perpend[1]+ny])

    return parallel_line

if __name__ == '__main__':
    import matplotlib.pyplot as plt
    import csv

    maps_x, maps_y = [], []
    with open("highway_map.csv", 'r') as f:
        reader = csv.reader(f, delimiter=' ')
        for r in reader:
            maps_x.append(float(r[0]))
            maps_y.append(float(r[1]))

    maps_x = [0, 1, 2]
    maps_y = [0, 1, 5]
    maps_s = get_maps(maps_x, maps_y)
    pa = draw_road(maps_x, maps_y, maps_s)
    plt.plot(maps_x, maps_y, 'r')
    pa_x, pa_y = zip(*pa)
    plt.plot(pa_x, pa_y, 'b')
    plt.axis('square')
    plt.show()
    


# graph get nearest waypoint
# 1. get closest waypoint, c->n
# 2. if xcn > pi/2, xc * cn > 0
# 3. search adjecent nearest edge