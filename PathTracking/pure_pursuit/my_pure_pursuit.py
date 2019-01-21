
import numpy as np
import matplotlib.pyplot as plt
import math

def interpolate_points(cx, cy, resolution):
    # for i in range(len(cx)):
    
    def interpolate(a, b):
        
        res = []
        angle = math.atan2(b[1]-a[1], b[0]-a[0])
        rx = r*math.cos(angle)
        ry = r*math.sin(angle)
        cur_x, cur_y = a
        while cur_x < b[0]:
            res.append([cur_x, cur_y])
            cur_x += rx
            cur_y += ry
        # res.append(b)
        return res
            

lf_dist = 1.0
dt = 0.1
L = 2.9
kp = 1.0
k=0.1

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
    
    def move(self, a, steer):
        self.x += self.v * dt * math.cos(self.yaw)
        self.y += self.v * dt * math.sin(self.yaw)
        self.yaw += self.v * dt * math.tan(steer) / L
        self.v += a * dt
        
def pure_pursuit(state, cx, cy, prev_idx):

    target_idx = calc_target_index(state, cx, cy)

    if prev_idx > target_idx:
        target_idx = prev_idx

    tx,ty=cx[target_idx], cy[target_idx]
    alpha = math.atan2(ty-state.y, tx-state.x)-state.yaw

    if state.v < 0:
        alpha = math.pi - alpha
    
    # lf = lf_dist + k * state.v

    delta = math.atan2(2*L*math.sin(alpha), math.sqrt((ty-state.y)**2+(tx-state.x)**2))
    return delta, target_idx 

def calc_target_index(state, cx, cy):
    dist = [square_dist(state.x, state.y, x, y) for x, y in zip(cx, cy)]
    idx = dist.index(min(dist))
    
    l = 0.0
    lf = lf_dist+k*state.v

    while lf > l and idx + 1 < len(cx):
        l+=math.sqrt(square_dist(cx[idx+1], cy[idx+1], cx[idx], cy[idx]))
        idx+=1
    
    return idx

def square_dist(x1, y1, x2, y2):
    return (y2-y1)**2 + (x2-x1)**2

def PControl(v, target):
    a = kp*(target-v)
    return a

if __name__ == '__main__':
    # target course
    cx = np.arange(0, 50, 0.1)
    cy = [math.sin(ix / 5.0) * ix / 2.0 for ix in cx]
    
    target_vel = 10.0/3.6
    state = State()
    dist_goal = square_dist(state.x, state.y, cx[-1], cy[-1])
    tolerance = 0.2

    lx, ly, lyaw, lv = [], [], [], []
    show_animation = True
    target_idx = 0
    while dist_goal > tolerance:
        lx.append(state.x)
        ly.append(state.y)
        lyaw.append(state.yaw)
        lv.append(state.v)

        if show_animation:
            plt.cla()
            plt.plot(cx,cy,'.r', label="course")
            plt.plot(lx, ly, 'b-', label="trajectory")
            plt.plot(cx[target_idx], cy[target_idx], 'xg', markersize=12, label='goal')
            plt.axis('equal')
            plt.grid(True)
            plt.title("Speed %s km/h" % (str(state.v*3.6)[:4]))
            plt.pause(0.01)

        a = PControl(state.v, target_vel)
        steer, target_idx = pure_pursuit(state, cx, cy, target_idx)
        state.move(a, steer)

        dist_goal = square_dist(state.x, state.y, cx[-1], cy[-1])

    print dist_goal

    if show_animation:
        pass

    # plt.plot(cx,cy, '.')
    
    # plt.show()