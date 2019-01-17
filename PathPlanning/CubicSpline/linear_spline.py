
# c_i = a_i(x-x_{i-1})+y_{i-1} when x_i-1 < x <= x_i
# s f(s) = [x(s), y(s)]

import numpy as np
import bisect
from math import sqrt
def cal_s(x, y):
    dx = np.diff(x)
    dy = np.diff(y)
    ds = [sqrt(i**i+j**j) for i, j in zip(dx, dy)]
    s = [0].extend(np.cumsum(ds))
    return s

def cal_ab(x, s):
    dx = np.diff(x)
    ds = np.diff(s)
    a = ds/dx
    b = s[:-1]
    return a, b

class LinearSpline2D:
    def __init__(self, x, y):
        self.s = self.cal_s(x, y)
        self.xs = LinearSpline(x, self.s)
        self.ys = LinearSpline(y, self.s)

    def cal_s(self, x, y):
        dx = np.diff(x)
        dy = np.diff(y)
        ds = [sqrt(i**2+j**2) for i, j in zip(dx, dy)]
        s = [0]
        s.extend(np.cumsum(ds))
        return s

    def cal_position(self, s):
        x = self.xs.cal_position(s)
        y = self.ys.cal_position(s)
        return x, y
        
    
class LinearSpline:
    def __init__(self, x, s):
        self.x = x
        self.s = s

        self.a, self.b = self.cal_ab(x, s)

    def cal_ab(self, x, s):
        dx = np.diff(x)
        ds = np.diff(s)
        a = dx/ds
        b = x[:-1]
        return a, b
    
    def cal_position(self, s):
        if s < 0:
            return
        idx = self.search(s)
        x = self.a[idx]*(s-self.s[idx])+self.b[idx]
        return x

    def search(self, s):
        return bisect.bisect(self.s, s) - 1
        


if __name__ == '__main__':
    x = [0, 1, 0, 2, 1, 2]
    y = [0, 1, 6, 6, 1, 0]

    ds = 0.1
    import matplotlib.pyplot as plt
    # plt.plot(x, y)
    # plt.show()
    
    spl = LinearSpline2D(x, y)
    
    sr = np.arange(0, spl.s[-1], ds)

    lx, ly = [], []
    for s in sr:
        x, y = spl.cal_position(s) 
        lx.append(x)
        ly.append(y)
    
    plt.scatter(lx, ly)
    print lx, ly
    plt.show()
    



    
    