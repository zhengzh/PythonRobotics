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
from a_star import dp_planning, calc_obstacle_map

XY_GRID_RESOLUTION = 2.0 #[m]
YAW_GRID_RESOLUTION = np.deg2rad(15.0) #[rad]
GOAL_TYAW_TH = np.deg2rad(5.0) #[rad]
MOTION_RESOLUTION = 0.1 #[m] path interporate resolution
N_STEER = 20.0 # number of steer command
EXTEND_AREA = 0.0  # [m]
H_COST = 1.0
MOTION_RESOLUTION = 0.1
SKIP_COLLISION_CHECK = 4
VR = 1.0 # robot radius

SB_COST = 100.0 # switch back penalty cost
BACK_COST = 5.0 # backward penalty cost
STEER_CHANGE_COST = 5.0 # steer angle change penalty cost
STEER_COST = 1.0 # steer angle change penalty cost
JACKKNIF_COST= 200.0 # Jackknif cost
H_COST = 5.0 # Heuristic cost

show_animation = True

_round = round
def round(x):
    return int(_round(x))

class Node:

    def __init__(self, xind, yind, yawind, direction, xlist, ylist, yawlist, directions, steer=0.0, pind=None, cost=None):
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

class Path:

    def __init__(self, xlist, ylist, yawlist, directionlist, cost):
        self.xlist = xlist
        self.ylist = ylist
        self.yawlist = yawlist
        self.directionlist = directionlist
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

        self.minx = round(min_x_m / xyreso)
        self.miny = round(min_y_m / xyreso)
        self.maxx = round(max_x_m / xyreso)
        self.maxy = round(max_y_m / xyreso)

        self.xw = round(self.maxx - self.minx)
        self.yw = round(self.maxy - self.miny)

        self.minyaw = round(- math.pi / yawreso) - 1
        self.maxyaw = round(math.pi / yawreso)
        self.yaww = round(self.maxyaw - self.minyaw)


def show_expansion_tree():
    pass


def calc_motion_inputs():
    
    for steer in np.linspace(-MAX_STEER, MAX_STEER, N_STEER):
        for d in [1, -1]:
            yield [steer, d]         


def get_neighbors(current, config, ox, oy, kdtree):
    # import ipdb; ipdb.set_trace()
    for steer, d in calc_motion_inputs():
        node = calc_next_node(current, steer, d, config, ox, oy, kdtree)
        if node and verify_index(node, config):
            yield node

def calc_next_node(current, steer, direction, config, ox, oy, kdtree):
    
    x, y, yaw = current.xlist[-1], current.ylist[-1], current.yawlist[-1]

    arc_l = XY_GRID_RESOLUTION * 1.5
    xlist, ylist, yawlist = [], [], []
    for dist in np.arange(0, arc_l, MOTION_RESOLUTION):
        x, y, yaw = move(x, y, yaw, MOTION_RESOLUTION*direction, steer)
        xlist.append(x)
        ylist.append(y)
        yawlist.append(yaw)
    
    # plt.plot(xlist, ylist)
    if not check_car_collision(xlist, ylist, yawlist, ox, oy, kdtree):
        # import ipdb; ipdb.set_trace()
        return None

    d = direction==1
    xind = int(x/XY_GRID_RESOLUTION)
    yind = int(y/XY_GRID_RESOLUTION)
    yawind = int(yaw/YAW_GRID_RESOLUTION)
    
    addedcost = 0.0

    if d != current.direction:
        addedcost += SB_COST
    
    # steer penalty
    addedcost += STEER_COST*abs(steer)

    # steer change penalty
    addedcost += STEER_CHANGE_COST*abs(current.steer-steer)

    cost = current.cost + addedcost    

    node = Node(xind, yind, yawind, d, xlist,
                ylist, yawlist, [d],
                pind=calc_index(current, config),
                cost=cost, steer=steer)

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
        if check_car_collision(path.x, path.y, path.yaw, ox, oy, kdtree):
            cost = calc_rs_path_cost(path)
            if not best or best > cost:
                best = cost
                best_path = path
  
    return best_path  # no update


def update_node_with_analystic_expantion(current, goal, 
                                        c, ox, oy, kdtree):
    apath = analytic_expantion(current, goal, c, ox,oy, kdtree)

    if apath:
        plt.plot(apath.x, apath.y)
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

    _, _, h_dp = dp_planning(nstart.xlist[-1], nstart.ylist[-1],
            ngoal.xlist[-1], ngoal.ylist[-1], ox, oy, xyreso, VR)

    pq = []
    openList[calc_index(nstart, config)] = nstart
    heapq.heappush(pq, (calc_cost(nstart, h_dp, ngoal, config), calc_index(nstart, config)))

    while True:
        if not openList:
            print("Error: Cannot find path, No open set")
            return [], [], []

        cost, c_id = heapq.heappop(pq)
        if openList.has_key(c_id):
            current = openList.pop(c_id)
            closedList[c_id] = current

        if show_animation:  # pragma: no cover
            plt.plot(current.xlist[-1], current.ylist[-1], "xc")
            if len(closedList.keys()) % 10 == 0:
                plt.pause(0.001)

        isupdated, fpath = update_node_with_analystic_expantion(
            current, ngoal, config, ox, oy, obkdtree)

        if isupdated:
            break
        
        for neighbor in get_neighbors(current, config, ox, oy, obkdtree):
            neighbor_index = calc_index(neighbor, config)
            if closedList.has_key(neighbor_index):
                continue
            if not openList.has_key(neighbor_index) \
                or openList[neighbor_index].cost > neighbor.cost: # TODO huristic
                heapq.heappush(pq, (calc_cost(neighbor, h_dp, ngoal, config), neighbor_index))
                openList[neighbor_index] = neighbor
        
        #  print(current)

    rx, ry, ryaw = [], [], []

    return rx, ry, ryaw

def calc_cost(n, h_dp, goal, c):
    ind = (n.yind - c.miny) * c.xw + (n.xind - c.minx)
    return (n.cost + H_COST*h_dp[ind].cost)

def get_final_path(closed, ngoal, nstart, config):
    rx, ry, ryaw = reversed(ngoal.x), reversed(ngoal.y), reversed(ngoal.yaw)
    direction = reversed(ngoal.directions)
    nid = ngoal.pind
    finalcost = ngoal.cost

    while nid != -1:
        n = closed[nid]
        rx.extend(n.xlist)
        ry.extend(n.ylist)
        ryaw.extend(n.yawlist)
        direction.extend(n.yawlist)
       
        nid = n.pind

    rx = reversed(rx)
    ry = reversed(ry)
    ryaw = reversed(ryaw)
    ryaw1 = reversed(ryaw1)
    direction = reversed(direction)

    # adjuct first direction
    direction[0] = direction[1]

    path = Path(rx, ry, ryaw, ryaw1, direction, finalcost)

    return path


def verify_index(node, c):
    xind, yind = node.xind, node.yind
    if xind>=c.minx and xind<=c.maxx and yind >=c.miny \
        and yind<=c.maxy:
        return True
    
    return False


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

    plt.plot(ox, oy, ".k")
    rs.plot_arrow(start[0], start[1], start[2])
    rs.plot_arrow(goal[0], goal[1], goal[2])

    plt.grid(True)
    plt.axis("equal")

    rx, ry, ryaw = hybrid_a_star_planning(
        start, goal, ox, oy, XY_GRID_RESOLUTION, YAW_GRID_RESOLUTION)

    plt.show()
    print(__file__ + " start!!")


if __name__ == '__main__':
    main()
