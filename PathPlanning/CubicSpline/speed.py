from math import sqrt
v = 10
xf = 10
dt = 0.1
x = 0
a = 2


while 1:
    x+=v*dt
    if v*v/2./a > xf:
        v = min(v, sqrt(2.*a*(xf-x)))