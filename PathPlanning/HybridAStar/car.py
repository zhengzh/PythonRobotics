
import matplotlib.pyplot as plt
from math import sqrt, cos, sin, tan, pi

WB = 3. # rear to front wheel
W = 2. # width of car
LF = 3.3 # distance from rear to vehicle front end
LB = 1.0 # distance from rear to vehicle back end
MAX_STEER = 0.6 #[rad] maximum steering angle 

#WBUBBLE_DIST = 1.5
#WBUBBLE_R = sqrt(1.5*1.5+1)

# vehicle rectangle verticles
VRX = [LF, LF, -LB, -LB, LF]
VRY = [W/2,-W/2,-W/2,W/2,W/2]


def check_car_collision(xlist, ylist, yawlist, ox, oy):
    for x, y, yaw in zip(xlist, ylist, yawlist):
        if not rectangle_check(x, y, yaw, ox, oy):
            return False # collision
    
    return True # no collision

def rectangle_check(x, y, yaw, ox, oy):
    # transform obstacles to base link frame
    c, s = cos(-yaw), sin(-yaw)
    for iox, ioy in zip(ox, oy):
        tx = iox - x
        ty = iox -y
        tx = c*tx - s*ty
        ty = s*ty + c*ty

        if tx > LF or tx < -LB or ty > W/2.0 or ty < -W/2.0:
            return True # no collision
    
    return False # collision


def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
    """Plot arrow."""
    if not isinstance(x, float):
        for (ix, iy, iyaw) in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * cos(yaw), length * sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width, alpha=0.4)
        plt.plot(x, y)

def plot_car(x, y, yaw):
    car_color = '-k'
    c, s = cos(yaw), sin(yaw)
    
    car_outline_x, car_outline_y = [], []
    for rx, ry in zip(VRX, VRY):
        tx = c*rx-s*ry + x
        ty = s*rx+c*ry + y
        car_outline_x.append(tx)
        car_outline_y.append(ty)
    
    arrow_x, arrow_y, arrow_yaw = c*1.5, s*1.5, yaw
    plot_arrow(arrow_x, arrow_y, arrow_yaw)
    
    plt.plot(car_outline_x, car_outline_y, car_color)

def pi_2_pi(angle):
    return (angle + pi) % (2 * pi) - pi

def move(x, y, yaw, distance, steer, L=WB):
    x += distance * cos(yaw)
    y += distance * sin(yaw)
    yaw += pi_2_pi(distance * tan(steer) / L) # distance/2

    return x, y, yaw

if __name__ == '__main__':
    x, y, yaw = 0., 0., 1.
    plt.axis('equal')
    plot_car(x, y, yaw)
    plt.show()
    