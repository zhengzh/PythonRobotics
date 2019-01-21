def bresenham_line(x0, y0, x1, y1):

    swap = False
    reverse = False
    if abs(y1-y0) >  abs(x1 - x0):
        swap = True
        x0, y0 = y0, x0
        x1, y1 = y1, x1

    if x0 > x1:
        reverse = True
        x0, x1 = x1, x0
        y0, y1 = y1, y0

    slope = abs((y1-y0)*1.0/(x1-x0))
    error = 0
    
    sign = 1 if y1>y0 else -1
    y = y0
    res = []
    for x in range(x0, x1+1):
        if swap:
            res.append([y, x])
        else:
            res.append([x, y])

        error+=slope
        if error > 0.5:
            y+=sign
            error-=1.0

    if reverse:
        res.reverse()
    return res

def get_line(start, end):
    """Bresenham's Line Algorithm
    Produces a list of tuples from start and end
 
    >>> points1 = get_line((0, 0), (3, 4))
    >>> points2 = get_line((3, 4), (0, 0))
    >>> assert(set(points1) == set(points2))
    >>> print points1
    [(0, 0), (1, 1), (1, 2), (2, 3), (3, 4)]
    >>> print points2
    [(3, 4), (2, 3), (1, 2), (1, 1), (0, 0)]
    """
    # Setup initial conditions
    x1, y1 = start
    x2, y2 = end
    dx = x2 - x1
    dy = y2 - y1
 
    # Determine how steep the line is
    is_steep = abs(dy) > abs(dx)
 
    # Rotate line
    if is_steep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2
 
    # Swap start and end points if necessary and store swap state
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True
 
    # Recalculate differentials
    dx = x2 - x1
    dy = y2 - y1
 
    # Calculate error
    error = int(dx / 2.0)
    ystep = 1 if y1 < y2 else -1
 
    # Iterate over bounding box generating points between start and end
    y = y1
    points = []
    for x in range(x1, x2 + 1):
        coord = (y, x) if is_steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx
 
    # Reverse the list if the coordinates were swapped
    if swapped:
        points.reverse()
    return points

print bresenham_line(0, 0, 3, 4)
print bresenham_line(3, 4, 0, 0)
    