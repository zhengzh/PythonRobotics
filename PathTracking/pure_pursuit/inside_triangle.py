import numpy as np
def inside_triangle(p, a, b, c):
    # triangle vertexes (a, b, c)
    # a triangle define a plane
    T = np.array(zip(a,b,c))
    p = np.array(p)
    x = np.linalg.inv(T).dot(p)
    assert np.sum(x) == 1
    return not np.any(x<=0)

def ray_intersection_with_plan(pixel,a, b, c, d):
    # plane equation parameters (a,b,c,d)
    pixel = np.array(pixel)
    normal = np.array([a,b,c])
    t = -d*1./np.sum(pixel*normal)
    return pixel*t

if __name__ == '__main__':
    A = [1, 2, 2]
    B = [3, 3, 3]
    C = [4, 5, 4]
    I = [2.25, 3, 2.75]
    A=[2,1,1]
    B=[2,2,2]
    C=[4,4,2]
    I=[3,2.8,1.8]
    

    print inside_triangle(I, A, B, C)
    print ray_intersection_with_plan([5,2,1], 3, 3, 5, -13)
    

