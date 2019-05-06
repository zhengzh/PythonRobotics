from math import *


class Vector:

    def __init__(self, x, y):
        self.x = float(x)
        self.y = float(y)

    def dot(self, w):
        dot = self.x * w.x + self.y * w.y
        return dot

    @property
    def norm(self):
        return sqrt(self.x**2 + self.y**2)

    def normalize(self):
        return self * (1. / self.norm)

    def project_onto(self, w):
        n = self.dot(w) / self.dot(self)
        return n * w

    @property
    def corrds(self):
        return [self.x, self.y]

    def perpendicular(self):
        return Vector(-self.y, self.x)

    def angle(self, w):

        c = self.dot(w) / (self.norm * w.norm)
        return acos(c)

    def __iter__(self):
        return self.corrds.__iter__()

    def __add__(self, w):
        s = None
        if isinstance(w, Vector):
            s = [i + j for i, j in zip(self, w)]
        else:
            s = [i + w for i in self]

        return Vector(*s)

    def __sub__(self, w):
        return self + w * -1

    def __mul__(self, w):
        if isinstance(w, Vector):
            return self.dot(w)

        return Vector(*[w * i for i in self])

    def __rmul__(self, w):
        return self * w

    def __str__(self):
        return str(list(self))


def main():

    v = Vector(1, 2)
    w = Vector(-1, 3)

    print(w - v)
    print(Vector(3, 4).norm)
    print(w.angle(v))
    print(v * w == v.dot(w))
    print(w.project_onto(v))
    print(w.perpendicular())
    print(v.normalize())


if __name__ == '__main__':
    main()
