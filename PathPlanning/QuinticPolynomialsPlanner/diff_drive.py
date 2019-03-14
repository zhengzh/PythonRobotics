

import numpy as np

def move(x, y, theta, v, w, dt, l):
    vr = v + l/2.0 * w
    vl = v - l/2.0 * w

    vr = min(5, max(vr, -5))
    vl = min(5, max(vl, -5))    

    return diffdrive(x, y, theta, vl, vr, dt, l)


def diffdrive(x, y, theta, v_l, v_r, t, l):
    # straight line
    if (v_l == v_r):
        theta_n = theta
        x_n = x + v_l * t * np.cos(theta)
        y_n = y + v_l * t * np.sin(theta)
    # circular motion
    else:
        # Calculate the radius
        R = l/2.0 * ((v_l + v_r) / (v_r - v_l))
        # computing center of curvature
        ICC_x = x - R * np.sin(theta)
        ICC_y = y + R * np.cos(theta)
        # compute the angular velocity
        omega = (v_r - v_l) / l
        # computing angle change
        dtheta = omega * t
        # forward kinematics for differential drive
        x_n = np.cos(dtheta)*(x-ICC_x) - np.sin(dtheta)*(y-ICC_y) + ICC_x
        y_n = np.sin(dtheta)*(x-ICC_x) + np.cos(dtheta)*(y-ICC_y) + ICC_y
        theta_n = theta + dtheta
    return x_n, y_n, theta_n


from quintic_polynomials_planner import quinic_polynomials_planner


if __name__ == '__main__':
    test()