# import os
# from random import randint
# os.system('clear')

# m, n = 10, 10
# grid = [[0 for i in range(m)] for j in range(n)]

# def random(grid):
#     randint(m)
#     pass

# def p(grid):
#     for i in grid:
#         print i

    
# p(grid)

# distance vo
# robot velocity
# acc, deacc
# s=v*v/2/acc

# pid control

# kp = 1.0

# def p()

# 0, 5, 5, 5

import matplotlib.pyplot as plt

ob=[[5, 0, 5], [8, 2, 9], [15, 10, 30]]
v=2
s=0
sg = 30

dt = 0.1
ls = []

t = 0
while s < sg:

    ls.append([t, s])
    collide = False
    sn=s+v*dt
    t+=dt
    for o in ob:
        if t >= o[1] and t<=o[2]:
            if round(s,1)<=o[0] and round(sn,1)>=o[0]:
                # collide with obstacle
                collide = True

    if not collide:
        s = sn

plt.plot([s[0] for s in ls], [s[1] for s in ls])
for o in ob:
    plt.plot([o[1],o[2]], [o[0], o[0]])
plt.show()
print ls[200]
    
