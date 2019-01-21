
def cartesian_to_frenet(x, y, theta, mapx, mapy):
    pass
    

def frenet_to_cartesian(s, d, mapx, mapy, ):
    pass


from math import sqrt, atan2, pi
def distance(x1, y1, x2, y2):
    return sqrt((x2-x1)**2+(y2-y1)**2)

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


# graph get nearest waypoint
# 1. get closest waypoint, c->n
# 2. if xcn > pi/2, xc * cn > 0
# 3. search adjecent nearest edge