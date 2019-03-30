

# move, speed, steer

kw = 1.
kv = 1.
dt = 0.1

def motion(x, u, dt):
    # motion model
    x[0] += u[0] * math.cos(x[2]) * dt
    x[1] += u[0] * math.sin(x[2]) * dt
    x[2] += u[1] * dt
    x[3] = u[0]
    x[4] = u[1]
    return x

def pid_v(current, target):
    return kv * (target-current)

def pid_w(current, target):
    return kw*((target-current)%pi-pi)


def main():
    start=[0.0,0.0, 0.0]
    goal=[1.0, 1.0, 1.0]
    
    time = 0.0

    while time > 0.0:
        pass

    pass

if __name__ == '__main__':
    main()
    