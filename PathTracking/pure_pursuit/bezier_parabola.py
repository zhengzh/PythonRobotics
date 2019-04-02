

import numpy as np
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
    a=(-5,10)
    b=(0,0)
    c=(10,5)
    t=0.8
    print parabola(a, b, c, t)