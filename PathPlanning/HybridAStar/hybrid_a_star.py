"""

Hybrid A* path planning

author: Atsushi Sakai (@Atsushi_twi)

"""

import sys
sys.path.append("../ReedsSheppPath/")

import math
import numpy as np
import scipy.spatial
import matplotlib.pyplot as plt
import reeds_shepp_path_planning as rs
import heapq
from car import move, check_car_collision, MAX_STEER, WB

XY_GRID_RESOLUTION = 2.0 #[m]
YAW_GRID_RESOLUTION = np.deg2rad(15.0) #[rad]
GOAL_TYAW_TH = np.deg2rad(5.0) #[rad]
MOTION_RESOLUTION = 0.1 #[m] path interporate resolution
N_STEER = 20.0 # number of steer command
EXTEND_AREA = 5.0  # [m]
H_COST = 1.0
MOTION_RESOLUTION = 0.1
SKIP_COLLISION_CHECK = 4

SB_COST = 100.0 # switch back penalty cost
BACK_COST = 5.0 # backward penalty cost
STEER_CHANGE_COST = 5.0 # steer angle change penalty cost
STEER_COST = 1.0 # steer angle change penalty cost
JACKKNIF_COST= 200.0 # Jackknif cost
H_COST = 5.0 # Heuristic cost

show_animation = True


class Node:

    def __init__(self, xind, yind, yawind, direction, xlist, ylist, yawlist, directions, steer=None, pind=None, cost=None):
        self.xind = xind
        self.yind = yind
        self.yawind = yawind
        self.direction = direction
        self.xlist = xlist
        self.ylist = ylist
        self.yawlist = yawlist
        self.directions = directions
        self.steer = steer
        self.pind = pind
        self.cost = cost


class KDTree:
    """
    Nearest neighbor search class with KDTree
    """

    def __init__(self, data):
        # store kd-tree
        self.tree = scipy.spatial.cKDTree(data)

    def search(self, inp, k=1):
        """
        Search NN
        inp: input data, single frame or multi frame
        """

        if len(inp.shape) >= 2:  # multi input
            index = []
            dist = []

            for i in inp.T:
                idist, iindex = self.tree.query(i, k=k)
                index.append(iindex)
                dist.append(idist)

            return index, dist
        else:
            dist, index = self.tree.query(inp, k=k)
            return index, dist

    def search_in_distance(self, inp, r):
        """
        find points with in a distance r
        """

        index = self.tree.query_ball_point(inp, r)
        return index


class Config:

    def __init__(self, ox, oy, xyreso, yawreso):
        min_x_m = min(ox) - EXTEND_AREA
        min_y_m = min(oy) - EXTEND_AREA
        max_x_m = max(ox) + EXTEND_AREA
        max_y_m = max(oy) + EXTEND_AREA

        ox.append(min_x_m)
        oy.append(min_y_m)
        ox.append(max_x_m)
        oy.append(max_y_m)

        self.minx = int(min_x_m / xyreso)
        self.miny = int(min_y_m / xyreso)
        self.maxx = int(max_x_m / xyreso)
        self.maxy = int(max_y_m / xyreso)

        self.xw = int(self.maxx - self.minx)
        self.yw = int(self.maxy - self.miny)

        self.minyaw = int(- math.pi / yawreso) - 1
        self.maxyaw = int(math.pi / yawreso)
        self.yaww = int(self.maxyaw - self.minyaw)


def show_expansion_tree():
    pass


def calc_motion_inputs():
    
    for steer in np.linspace(-MAX_STEER, MAX_STEER, N_STEER):
        for d in [1, -1]:
            yield [steer, d]         


def get_neighbors(current, config, ox, oy, kdtree):
    for steer, d in calc_motion_inputs():
        node = calc_next_node(current, steer, d, config, ox, oy, kdtree)
        if node and verify_index(node):
            yield node

def calc_next_node(current, steer, direction, config, ox, oy, kdtree):
    
    x, y, yaw = current.x, current.y, current.yaw

    xlist, ylist, yawlist = [], [], []
    for dist in np.arange(0, distance, MOTION_RESOLUTION):
        nx, ny, nyaw = move(x, y, yaw, MOTION_RESOLUTION*direction, steer)
        xlist.append(nx)
        ylist.append(ny)
        yawlist.append(nyaw)
    
    if not check_car_collision(xlist, ylist, yawlist, ox, oy):
        return None

    d = direction==1
    xind = int(x/config.xyreso)
    yind = int(y/config.xyreso)
    yawind = int(ya/config.yawreso)
    
    addedcost = 0.0

    if d != current.direction:
        addedcost += SB_COST
    
    # steer penalty
    addedcost += STEER_COST*abs(u)

    # steer change penalty
    addedcost += STEER_CHANGE_COST*abs(current.steer-u)

    cost = current.cost + addedcost    

    node = Node(xind, yind, yawind, d, xlist,
                ylist, yawlist, [d],
                pind=calc_index(current),
                cost=cost)

    return node


def is_same_grid(n1, n2):
    if n1.xind==n2.xind and n1.yind==n2.yind and n1.yawind==n2.yawind:
        return True
    return False

def analytic_expantion(current, goal, c, ox, oy, kdtree):

    sx = current.xlist[-1]
    sy = current.ylist[-1]
    syaw = current.yawlist[-1]

    gx = goal.xlist[-1]
    gy = goal.ylist[-1]
    gyaw = goal.yawlist[-1]

    max_curvature = math.tan(MAX_STEER)/WB
    paths = rs.calc_paths(sx,sy,syaw,gx, gy, gyaw,
                            max_curvature, step_size=MOTION_RESOLUTION)

    if not len(paths):
        return None

    best_path, best = None, None

    for path in paths:
        if check_car_collision(path.x, path.y, path.yaw, ox, oy):
            cost = calc_rs_path_cost(path)
            if not best or best > cost:
                best = cost
                best_path = path
  
    return best_path  # no update


def update_node_with_analystic_expantion(current, goal, 
                                        c, ox, oy, kdtree):
    apath = analytic_expantion(current, goal, c, ox,oy, kdtree)

    if apath:
        return True, apath
    
    return False, None


def calc_rs_path_cost(rspath):

    cost = 0.0
    for l in rspath.lengths:
        if l >= 0: # forward
            cost += l
        else: # back
            cost += abs(l) * BACK_COST

    # swich back penalty
    for i in range(len(rspath.lengths)-1):
        if rspath.lengths[i] * rspath.lengths[i+1] < 0.0: # switch back
            cost += SB_COST

    # steer penalyty
    for ctype in rspath.ctypes:
        if ctype != "S": # curve
            cost += STEER_COST*abs(MAX_STEER)

    # ==steer change penalty
    # calc steer profile
    nctypes = len(rspath.ctypes)
    ulist = [0.0]*nctypes
    for i in range(nctypes):
        if rspath.ctypes[i] == "R" :
            ulist[i] = - MAX_STEER
        elif rspath.ctypes[i] == "L":
            ulist[i] = MAX_STEER
 
    for i in range(len(rspath.ctypes)-1):
        cost += STEER_CHANGE_COST*abs(ulist[i+1] - ulist[i])

    return cost


def hybrid_a_star_planning(start, goal, ox, oy, xyreso, yawreso):
    """
    start
    goal
    ox: x position list of Obstacles [m]
    oy: y position list of Obstacles [m]
    xyreso: grid resolution [m]
    yawreso: yaw angle resolution [rad]
    """

    start[2], goal[2] = rs.pi_2_pi(start[2]), rs.pi_2_pi(goal[2])
    tox, toy = ox[:], oy[:]

    obkdtree = KDTree(np.vstack((tox, toy)).T)

    config = Config(tox, toy, xyreso, yawreso)

    nstart = Node(int(start[0] / xyreso), int(start[1] / xyreso), int(start[2] / yawreso),
                  True, [start[0]], [start[1]], [start[2]], [True], cost=0)
    ngoal = Node(int(goal[0] / xyreso), int(goal[1] / xyreso), int(goal[2] / yawreso),
                 True, [goal[0]], [goal[1]], [goal[2]], [True])

    openList, closedList = {}, {}
    h = []

    pq = []
    openList[calc_index(nstart, config)] = nstart
    heapq.heappush(pq, (calc_cost(nstart, h, ngoal, config), calc_index(nstart, config)))

    while True:
        if not openList:
            print("Error: Cannot find path, No open set")
            return [], [], []

        cost, c_id = heapq.heappop(pq)
        current = openList.pop(c_id)
        closedList[c_id] = current

        isupdated, fpath = analytic_expantion(
            current, ngoal, config, ox, oy, obkdtree)

        if isupdated:
            break
        
        for neighbor in get_neighbors():
            neighbor_index = calc_index(neighbor, config)
            if closedList.has_key(neighbor_index):
                continue
            if not openList.has_key(neighbor_index) \
                or openList[neighbor_index].cost > neighbor.cost: # TODO huristic
                heapq.heappush(pq, (neighbor.cost, neighbor_index))
                openList[neighbor_index] = neighbor
        
        #  print(current)

    rx, ry, ryaw = [], [], []

    return rx, ry, ryaw

def get_final_path():
    pass
    

def verify_index(node, c):
    xind, yind = node.xind, node.yind
    if xind>=c.minx and xind<=c.maxx and yind >=c.miny \
        and yind<=c.maxy:
        return True
    
    return False


def calc_cost(n, h, ngoal, c):

    hcost = 1.0

    return (n.cost + H_COST * hcost)


def calc_index(node, c):
    ind = (node.yawind - c.minyaw) * c.xw * c.yw + \
        (node.yind - c.miny) * c.xw + (node.xind - c.minx)

    if ind <= 0:
        print("Error(calc_index):", ind)

    return ind


def main():
    print("Start Hybrid A* planning")

    ox, oy = [], []

    for i in range(60):
        ox.append(i)
        oy.append(0.0)
    for i in range(60):
        ox.append(60.0)
        oy.append(i)
    for i in range(61):
        ox.append(i)
        oy.append(60.0)
    for i in range(61):
        ox.append(0.0)
        oy.append(i)
    for i in range(40):
        ox.append(20.0)
        oy.append(i)
    for i in range(40):
        ox.append(40.0)
        oy.append(60.0 - i)

    # Set Initial parameters
    start = [10.0, 10.0, np.deg2rad(90.0)]
    goal = [50.0, 50.0, np.deg2rad(-90.0)]

    xyreso = 2.0
    yawreso = np.deg2rad(15.0)

    rx, ry, ryaw = hybrid_a_star_planning(
        start, goal, ox, oy, xyreso, yawreso)

    plt.plot(ox, oy, ".k")
    rs.plot_arrow(start[0], start[1], start[2])
    rs.plot_arrow(goal[0], goal[1], goal[2])

    plt.grid(True)
    plt.axis("equal")
    plt.show()

    print(__file__ + " start!!")


if __name__ == '__main__':
    main()
