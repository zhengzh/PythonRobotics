
from math import *


def rectangle_check(x, y, yaw, ox, oy, LF, LB, W):
    
    # transform obstacles to base_link frame
    c, s = cos(-yaw), sin(-yaw)
    for iox, ioy in zip(ox, oy):
        tx = iox - x
        ty = ioy - y
        rx = c*tx - s*ty
        ry = s*tx + c*ty

        if not (rx > LF or rx < -LB or ry > W/2.0 or ry < -W/2.0):
            return False # no collision
    
    return True # collision

