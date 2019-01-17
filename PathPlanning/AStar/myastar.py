
show_animation = True
from pqdict import pqdict

class Node:

    def __init__(self, x, y, cost, pind=-1):
        self.x = x
        self.y = y
        self.cost = cost
        self.pind = pind

    @property
    def key(self):
        return self.x*10000+y


def astar(grid, sx, sy, ex, ey):
    start = Node(sx, sy, 0.0)

    pq = pqdict({start.key: start})

    while 1:
        current = pq.popitem()
        
        for n in grid.get_neighbors():
            
        

        pass


    
class Grid:

    def __init__(self, width, height):
        pass
    
    def set_obstacles(self):
        pass

    def get_neighbors(self):
        pass


def inflation(grid):
    pass



