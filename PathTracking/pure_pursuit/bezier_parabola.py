

import numpy as np
import matplotlib.pyplot as plt

def parabola(a, b, c, t):
    a,b,c = map(np.array, [a, b, c])
    q=(1-t)*a+t*b
    r=(1-t)*b+t*c
    p=(1-t)*q+t*r
    return p

if __name__ == '__main__':
    # a=(0,8)
    # b=(0,0)
    # c=(8,0)
    # t=0.75
    a=(0,10)
    b=(0,0)
    c=(10,5)
    
    points = []
    for t in np.arange(0,1.0,0.01):
        points.append(parabola(a, b, c, t).tolist())

    plt.plot(*zip(a, b, c))
    plt.plot(*zip(*points))
    plt.show()